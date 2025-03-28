#!/usr/bin/python3
import rosbag
import matplotlib.pyplot as plt
from pathlib import Path

classes_path = Path(__file__)

upper_directory = classes_path.parent
upper_directory = upper_directory.parent
lower_directory = upper_directory / "data" / "modelIdentification"

bag_file = lower_directory / "niu_1.bag"

time = []
vx = []
tau = []
w_bl = []

with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages():
        time += [t]

        if( topic == "/record_data/velocity_bodyFrame" ):
            vx += [msg.velocity[0].linear.x]
        
        if( topic == "/record_data/torque_bl" ):
            tau += [msg.data]

        if( topic == '/record_data/jointState' ):
            w_bl += [msg.velocity[0]]

plt.plot(tau, 'r')
plt.plot(vx, 'k')
plt.plot(w_bl, 'g')
plt.show()