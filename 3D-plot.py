import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

roll_values = []
pitch_values = []
yaw_values = []

with open("FOPID_case3-5.txt", "r") as file:
    for line in file:
        components = line.split(", ")

        roll = float(components[0].split(": ")[1])
        pitch = float(components[1].split(": ")[1])
        yaw = float(components[2].split(": ")[1])

        roll_values.append(roll)
        pitch_values.append(pitch)
        yaw_values.append(yaw)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot(roll_values, pitch_values, yaw_values)
ax.set_xlabel('Roll')
ax.set_ylabel('Pitch')
ax.set_zlabel('Yaw')

plt.show()
