#!/usr/bin/python3
import rosbag, math
import matplotlib.pyplot as plt
from pathlib import Path

classes_path = Path(__file__)

upper_directory = classes_path.parent
upper_directory = upper_directory.parent
lower_directory = upper_directory / "data" / "modelIdentification"

bag_file = lower_directory / "niu_1_tau_10.bag"

time = []
vx = []
tau = []
w_bl = []

with rosbag.Bag(bag_file, 'r') as bag:

    for topic, msg, t in bag.read_messages():

        if( topic == "/record_data/velocity_bodyFrame" ):
            vx += [msg.velocity[0].linear.x]
        
        if( topic == "/record_data/torque_bl" ):
            tau += [msg.data]

        if( topic == '/record_data/jointState' ):
            w_bl += [msg.velocity[0]]
        
        if( topic == '/record_data/clock' ):
            time += [msg.data]

index = 0
step = 10
radius = 0.111
mass = 27.4
niu = 1.0

vx_id = [vx[0]]
time_id = [time[0]]
slip_id = [w_bl[0] * radius - vx[0]]

for _ in range(500):
    now = time[index]

    time_diff = time[index + step] - time[index]

    """if(abs(slip_id[-1]) > 0.1):
        traction = mass * 9.81 / 4 * niu
    
    else:
        traction = tau[index] / radius"""

    if( tau[index] / radius > abs( mass * 9.81 / 4 * niu * math.tanh(100 * slip_id[-1]) ) ):
        traction = mass * 9.81 / 4 * niu * math.tanh(100 * slip_id[-1])
    
    else:
        traction = tau[index] / radius #* math.tanh(100 * slip_id[-1])

    next_vx = time_diff * 4 * traction / mass + vx_id[-1]

    slip_id += [w_bl[index + step] * radius - vx[index + step]]

    vx_id += [next_vx]
    time_id += [time[index + step]]

    index += step

plt.plot(time, tau, 'r')
plt.plot(time, vx, 'k')
plt.plot(time_id, vx_id)
plt.plot(time_id, slip_id)
#plt.plot(tau, vx)
#plt.plot(time, w_bl, 'g')
plt.show()