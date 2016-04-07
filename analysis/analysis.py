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
x = data['data_1']
xdot = data['data_2']
y = data['data_3']
ydot = data['data_4']

# and so on

fig = plt.figure()
ax1 = fig.add_subplot(121)
ax1.plot(time, x)

ax2 = fig.add_subplot(122)
ax2.plot(x, y)
