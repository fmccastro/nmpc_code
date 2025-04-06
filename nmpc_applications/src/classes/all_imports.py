#!/usr/bin/python3.8
import ctypes, rospy, pygame, time, math, sys, scipy, matplotlib, skfmm, signal, subprocess, os, pickle, json, tf2_ros, rosbag

#   ROS1 Messages
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32, Float64, Float32, Float32MultiArray, Bool, ColorRGBA, Header
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Inertia, Polygon, Point, Pose, Point, Quaternion, Vector3, Wrench, WrenchStamped
from gazebo_msgs.msg import LinkStates, PerformanceMetrics
from visualization_msgs.msg import Marker, MarkerArray
from nmpc_bringup.msg import pose3D, velocity3D, pose3DStamped, wheelTrueVelocitiesBodyFrame, referencePath

#   Services
from gazebo_msgs.srv import GetModelProperties, GetWorldProperties, GetLinkProperties, GetPhysicsProperties, ApplyJointEffort, JointRequest
from controller_manager_msgs.srv import ListControllers, UnloadController, SwitchController

import numpy as np
import casadi as ca
import cv2 as cv
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.animation as animation
import matplotlib.transforms as transforms
import scipy.spatial.distance as dist

#from tf.transformations import quaternion_from_euler
from scipy.spatial.transform import Rotation as R
from matplotlib import cm
from scipy import interpolate
from scipy.interpolate import splprep, splev, CubicSpline
from scipy.optimize import minimize
from functools import partial
from matplotlib.patches import Ellipse
from threading import Thread
from tkinter import *

from mpl_toolkits.mplot3d import Axes3D

from PIL import Image

#from simple_pid import PID

#   Acados
from acados_template import AcadosModel, AcadosOcpCost, AcadosOcpConstraints, AcadosOcpDims, AcadosOcp, AcadosOcpSolver, AcadosOcpOptions, AcadosSimSolver, AcadosSimOptions, AcadosSimDims, AcadosSim