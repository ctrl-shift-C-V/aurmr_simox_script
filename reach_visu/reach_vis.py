import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

fig = plt.figure()
fig_reach = plt.figure(figsize=(8, 6))

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
ave_plt.bar3d(0, 470, 0, 8, 8, 50, shade=True, color='black')
ave_plt.set_xlabel('x')
ave_plt.set_ylabel('y')
ave_plt.set_zlabel('reach')
ave_plt.set_zlim(0,100)
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
min_plt.bar3d(0, 470, 0, 8, 8, 8, shade=True, color='black')
min_plt.set_zlim(0, 10)
min_plt.set_xlabel('x')
min_plt.set_ylabel('y')
min_plt.set_zlabel('reach')
min_plt.set_zlim(0,10)
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
ave_plt.bar3d(0, 470, 0, 8, 8, 50, shade=True, color='black')
ave_plt.set_xlabel('x')
ave_plt.set_ylabel('y')
ave_plt.set_zlabel('reach')
ave_plt.set_zlim(0,100)
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
min_plt.bar3d(0, 470, 0, 8, 8, 8, shade=True, color='black')
min_plt.set_zlim(0, 10)
min_plt.set_xlabel('x')
min_plt.set_ylabel('y')
min_plt.set_zlabel('reach')
min_plt.set_zlim(0,10)
min_plt.set_title('lowest bin reach (non-linear weight)')


###################################### plot with 16 bins at each pose ################################
df = pd.read_csv('bin_reach_linear_over_volumn.csv')
reach_plt = fig_reach.add_subplot(111, projection='3d')
colors = colors = ['plum']*4 + ['red']*4 + ['g']*4 + ['b'] * 4
for index, row in df.iterrows():
    row_np = row.to_numpy()
    x = row_np[0]
    y = row_np[1]
    x_pos = np.zeros(16)
    y_pos = np.zeros(16)
    bin_reach = row_np[2:]
    for i in range(0,4):
        for j in range(0,4):
            x_pos[i*4+j] = x + i*10
            y_pos[i*4+j] = y + j*10
    reach_plt.bar3d(x_pos, y_pos, 0, 8, 8, bin_reach, shade=True, color=colors)
# added a cube as robot
reach_plt.bar3d(0, 470, 0, 8, 8, 20, shade=True, color='black')

reach_plt.set_xticks(np.arange(0, 901, 100))
reach_plt.set_yticks(np.arange(200, 801, 50))

plt.show()
