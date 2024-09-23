import matplotlib.pyplot as plt

roll_values = []
pitch_values = []
yaw_values = []

with open("FOPID_case2-6.txt", "r") as file:
    for line in file:
        components = line.split(", ")

        roll = float(components[0].split(": ")[1])
        pitch = float(components[1].split(": ")[1])
        yaw = float(components[2].split(": ")[1])

        roll_values.append(roll)
        pitch_values.append(pitch)
        yaw_values.append(yaw)

fig, axs = plt.subplots(3)

axs[0].plot(roll_values)
axs[0].set_xlabel('Time')
axs[0].set_ylabel('Roll')

axs[1].plot(pitch_values)
axs[1].set_xlabel('Time')
axs[1].set_ylabel('Pitch')

axs[2].plot(yaw_values)
axs[2].set_xlabel('Time')
axs[2].set_ylabel('Yaw')

plt.tight_layout()
plt.show()
