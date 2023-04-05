import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# load path.csv into a pandas dataframe
path_df = pd.read_csv("../path.csv")

# create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# plot the path
ax.plot(path_df['p_x'], path_df['p_y'], path_df['p_z'])

# set axis labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# show the plot
plt.show()
