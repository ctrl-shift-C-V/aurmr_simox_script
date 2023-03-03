import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

fig = plt.figure()

################################# linear weight #######################################
# draw ave_reach plot
df = pd.read_csv('bin_reach_linear.csv')
x = df['x'].to_numpy()
y = df['y'].to_numpy()
ave_plt = fig.add_subplot(221, projection='3d')
width = 20
depth = 20
ave_reach = np.zeros(len(x))
for index, row in df.iterrows():
    bin_reach = row.to_numpy()[2:]
    ave_reach[index] = np.mean(bin_reach)

# cneter bars
x -= width / 2
y -= depth / 2
ave_plt.bar3d(x, y, 0, width, depth, ave_reach, shade=True, color='y')
# add a cube as robot
ave_plt.bar3d(0, 220, 0, 50, 500, 50, shade=True, color='b')
ave_plt.set_xlabel('x')
ave_plt.set_ylabel('y')
ave_plt.set_zlabel('z')
ave_plt.set_title('average bin reach (linear weight)')


# draw min reach plot
df = pd.read_csv('bin_reach_linear_over_volumn.csv')
x = df['x'].to_numpy()
y = df['y'].to_numpy()
min_plt = fig.add_subplot(222, projection='3d')
min_reach = np.zeros(len(x))
for index, row in df.iterrows():
    bin_reach = row.to_numpy()[2:]
    min_reach[index] = np.min(bin_reach)

# cneter bars
x -= width / 2
y -= depth / 2
min_plt.bar3d(x, y, 0, width, depth, min_reach, shade=True, color='r')
# added a cube as robot
min_plt.bar3d(0, 220, 0, 50, 500, 6, shade=True, color='b')
min_plt.set_zlim(0, 10)
min_plt.set_xlabel('x')
min_plt.set_ylabel('y')
min_plt.set_zlabel('z')
min_plt.set_title('lowest bin reach (linear weight)')

################################## Non-linear weight ######################################

# draw ave_reach plot
df = pd.read_csv('bin_reach_nlin.csv')
x = df['x'].to_numpy()
y = df['y'].to_numpy()
ave_plt = fig.add_subplot(223, projection='3d')
ave_reach = np.zeros(len(x))
for index, row in df.iterrows():
    bin_reach = row.to_numpy()[2:]
    ave_reach[index] = np.mean(bin_reach)

# cneter bars
x -= width / 2
y -= depth / 2
ave_plt.bar3d(x, y, 0, width, depth, ave_reach, shade=True, color='y')
# add a cube as robot
ave_plt.bar3d(0, 220, 0, 50, 500, 50, shade=True, color='b')
ave_plt.set_xlabel('x')
ave_plt.set_ylabel('y')
ave_plt.set_zlabel('z')
ave_plt.set_title('average bin reach (non-linear weight)')


# draw min reach plot
df = pd.read_csv('bin_reach_nlin_over_volumn.csv')
x = df['x'].to_numpy()
y = df['y'].to_numpy()
min_plt = fig.add_subplot(224, projection='3d')
min_reach = np.zeros(len(x))
for index, row in df.iterrows():
    bin_reach = row.to_numpy()[2:]
    min_reach[index] = np.min(bin_reach)

# cneter bars
x -= width / 2
y -= depth / 2
min_plt.bar3d(x, y, 0, width, depth, min_reach, shade=True, color='r')
# added a cube as robot
min_plt.bar3d(0, 220, 0, 50, 500, 6, shade=True, color='b')
min_plt.set_zlim(0, 10)
min_plt.set_xlabel('x')
min_plt.set_ylabel('y')
min_plt.set_zlabel('z')
min_plt.set_title('lowest bin reach (non-linear weight)')

plt.show()
