import h5py
import matplotlib.pyplot as plt

filename = '2016-04-06-17-56-44.hdf5'
f = h5py.File(filename)

data = f['/tracking_template/kalman_filtered_points'].value

'''
Recall from the kalman_filtering_template:

float_time = pointcloud.header.stamp.secs + pointcloud.header.stamp.nsecs*1e-9
x = xhat.item(0) # xhat is a matrix, .item() gives you the actual value
xdot = xhat.item(1)
y = xhat.item(2)
ydot = xhat.item(3)
p_vector = P.reshape(16).tolist()[0]
data = [float_time, x, xdot, y, ydot]
data.extend(p_vector)

Thus...        
'''

time = data['data_0'] 
# note: the time is represented in "time since the epoch", hence it starts at a very large number
# if you would like time to start at 0, you can subtract, the first, or the minimal, value of time from time
x = data['data_1']
xdot = data['data_2']
y = data['data_3']
ydot = data['data_4']

# extract the position and velocity covariances from the data 
# these values represent the diagonal of the 4x4 covariance matrix 
rxx = data['data_5']
rxdotxdot = data['data_10']
ryy = data['data_15']
rydotydot = data['data_20']

# and so on

fig = plt.figure()
ax1 = fig.add_subplot(121)
ax1.plot(time, x)

ax2 = fig.add_subplot(122)
ax2.plot(x, y)

# some other handy commands:

ax2.set_xlim(0,1) # set the limits on the x axis, same goes for the y axis
fig.savefig('filename.png', format='png') # save a figure as a png 
