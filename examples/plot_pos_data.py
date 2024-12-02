import matplotlib.pyplot as plt
import numpy as np

# Create a 3d figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Read position data from a csv file
data = np.genfromtxt('position_log.csv', delimiter=',', names=['x', 'y', 'z'])

# 3d plot
ax.plot(data['x'], data['y'], data['z'])

# Set limits
ax.set_xlim([-2, 2])
ax.set_ylim([-2, 2])
ax.set_zlim([0, 2])

# Set labels
ax.set_xlabel('X (meters)')
ax.set_ylabel('Y (meters)')
ax.set_zlabel('Z (meters)')

# Show the plot
plt.show()