import numpy as np
from scipy.optimize import differential_evolution
from djitellopy import tello
import KeyPress_bib as kpb
from time import sleep
import cv2
import time
from threading import Thread

class PID:
    def __init__(self, P=0.2, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.clear()

    def clear(self):
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

    def update(self, feedback_value):
        error = self.SetPoint - feedback_value
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if(delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if(delta_time > 0):
                self.DTerm = delta_error / delta_time

            self.last_time = self.current_time
            self.last_error = error

        return self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)


# Define the cost function for optimization
def run_system(pid):
    # Simulate the system using the given PID controller
    errors = []
    for _ in range(100):  # Simulate for 100 time steps
        current_value = np.random.random() * 480    # Simulated curent value
        error = pid.SetPoint - current_value
        pid.update(current_value)
        errors.append(error)
    return errors

def cost_function(pid_params):
    pid = PID(P=pid_params[0], I=pid_params[1], D=pid_params[2])
    pid.SetPoint = (0 + 480) // 2   # Assuming setpoint is the middle of the frame
    error = run_system(pid)
    return np.sum(np.abs(error))

# Use differential evolution to minimize the cost function
bounds = [(0, 10), (0, 10), (0,10)]
result = differential_evolution(cost_function, bounds)
optimal_pid_params = result.x
print('Optimal PID parameters: ', optimal_pid_params)

# Initialize the PID controller with optimal parameters
pid = PID(P=optimal_pid_params[0], I=optimal_pid_params[1], D=optimal_pid_params[2])
pid.SetPoint = (0 + 480) // 2   # Assuming you want the setpoint to be the middle of the frame (xmin + xmax) / 2

me = tello.Tello()
kpb.init()                                  #init keyboard
me.connect()                                #entry SDK mode
print('Battery level : ', me.get_battery()) #displays battery status
me.streamon()                               #set video stream on
cap = cv2.VideoCapture(1)
frameWidth, frameHeight = 480, 360          #size in pixels of the video format received by the drone
xmin, xmax, ymin, ymax, ymin2, ymax2 = 0, 480, 290, 360, 0, 289 #Used to split the image in two
senstivity = 5                              #if number is high = less sensitive
curve = 0                                   #init curve
etat = [0, 0, 7, -7, -45, 45]               #different curve
speedTab = [0, 20, 15, 15, 0, 0]            #different speed ... none|-5 to 5|10 to 80|-80 to -10|<=-80|>=80
color = (225, 206, 154)
me.takeoff()                                #Tello auto takeoff
me.send_rc_control(0, 0, 0, 0)              #to stabilise drone

def thresholding(img):
    median = cv2.medianBlur(img, 5)                     #Apply median blur with 5x5 core size
    imgHsv = cv2.cvtColor(median, cv2.COLOR_BGR2HSV)    #Convert to HSV color space (Hue Saturation Value)
    h, s, v = cv2.split(imgHsv)                         #Split the image into H, S, and V channels
    ret_h, th_h = cv2.threshold(h, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    ret_s, th_s = cv2.threshold(s, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    ret_v, th_v = cv2.threshold(v, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    mask = cv2.bitwise_and(th_v, th_s)                  # Fusion th_v and th_s

    return mask

def getContours(imgThres, img):
    cx = 0
    cy = 0
    contours, hieracrhy = cv2.findContours(imgThres, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    #find contours on the image based on the threshold image

    if len(contours) != 0:
        biggest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(biggest)
        cx = x + w // 2
        cy = y + h // 2

        cv2.drawContours(img, biggest, -1, (231, 62, 1), 7) #draw contours on the image based on the threshold image
        cv2.circle(img, (cx, cy), 10, (34, 120, 15), cv2.FILLED)
        # draw a full circle at the center of the contours, this is the target to follow

    return cx, cy   # returns the coordinates of the center to follow


def getOut(cx, cy):
    dOut = 2
    xo = (xmax+xmin) // 2
    cv2.line(img, (xo, cy), (cx, cy), color, 5)
    diff = cx - xo
    len = abs(diff)

    # no detection |-5 to 5|10 to 80|-80 to -10|<=-80|>=80
    if len <= 10:       dOut = 1    #centre between -5 & 5 pxl
    elif 10 < diff < 80:
        if cy >= 350:   dOut = 5    #Right extreme
        else:           dOut = 2    #Right
    elif -80 < diff < -10:
        if cy >= 350:   dOut = 4    #Left extreme
        else:           dOut = 3    #Left
    elif diff <= -80:   dOut = 4        #Left extreme
    elif diff >= 80:    dOut = 5    #Right extreme
    else:
        dOut = 0
        print('NOO DETECTION')

    return dOut

def sendCommands(dOut, cx):
    global curve
    h = me.get_height()

    # Use the PID controller to get the control value
    control_value = pid.update(cx)

    # Adjust the control value to be used for left-right movement
    lr = int(np.clip(control_value, -5, 5))  # Limited between -5 and 5 for safety

    # ---------- SPEED SET ----------
    fSpeed = speedTab[dOut]
    # ---------- Rotation ----------
    curve = etat[dOut]
    # ---------- HEIGHT SETTING ----------
    if 5 < h <= 30:
        me.send_rc_control(0, fSpeed, 0, curve)    # sending a drone order
        print('0', fSpeed, '0', curve, ' ---- height: ', h)
    elif h > 30:
        print('Stationary mode, ↓↓↓↓ height: ', h)
        me.send_rc_control(0, 0, 0, 0)  # send command drone stationary mode
        me.move_down(20)
    elif h <= 5:
        print('Stationary mode, ↑↑↑↑ height: ', h)
        me.send_rc_control(0, 0, 0, 0)  # send command drone stationary mode
        me.move_up(20)

roll_values = []
pitch_values = []
yaw_values = []

with open("PID_case2-4.txt", "w") as file:

    while True:
        # ---------- tresholding ----------
        img0 = me.get_frame_read().frame
        img0 = cv2.resize(img0, (frameWidth, frameHeight))      #image resizing
        img = img0[ymin:ymax, xmin:xmax]                        #used cropped image for line detection
        im2 = img0[ymin2:ymax2, xmin:xmax]                      #unsed cropped image
        #img = cv2.GaussianBlur(img, (7, 7), 0)                 #Gaussian blur
        imgThres = thresholding(img)                            #thresholding
        fct = getContours(imgThres, img)        #draw contours on the image based on the threshold image
        cx, cy = fct[0], fct[1]                 # x-coordinate of contour center & y-coordinate of contour center
        dOut = getOut(cx, cy)           # setting for Rotation

        sendCommands(dOut, cx)          # sent command drone
        cv2.imshow("Img0", img)     # displays the lower part of the image
        cv2.imshow("Img2", im2)     # displays the upper part of the image
        print('Battery level : ', me.get_battery())
        print('Optimal PID parameters: ', optimal_pid_params)

        roll = me.get_roll()
        pitch = me.get_pitch()
        yaw = me.get_yaw()

        roll_values.append(roll)
        pitch_values.append(pitch)
        yaw_values.append(yaw)

        print(f'Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}')
        file.write(f'Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}\n')

        if kpb.getKey("l"):
            me.land()           # Tello auto land
            sleep(3)
            break
        elif kpb.getKey("o"):
            me.emergency()      # Stop all motors immediately
            break
        cv2.waitKey(1)

# Free up resources and close windows
cap.release()
cv2.destroyAllWindows()
print('Battery level : ', me.get_battery())
