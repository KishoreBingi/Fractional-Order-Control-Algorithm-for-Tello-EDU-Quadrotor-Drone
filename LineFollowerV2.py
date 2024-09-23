import numpy as np
from djitellopy import tello
import KeyPress_bib as kpb
from time import sleep
import cv2
from threading import Thread

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
    # ---------- TRANSLATION ----------
    #lr = (cx - (xmax+xmin)//2) // senstivity #240
    #lr = int(np.clip(lr, -5,  5)) # limited btw -5 and 5
    #if 2 > lr > -2:
    #    lr = 0
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


    #print(lr, fSpeed, '0', curve, ' --|-- hauteur: ', h)

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
