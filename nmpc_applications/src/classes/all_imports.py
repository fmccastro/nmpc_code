#!/home/fmccastro/bin/python3.8
import ctypes, rospy, pygame, time, math, sys, scipy, matplotlib, skfmm, signal, subprocess, os, pickle, json, tf2_ros, rosbag, tf, pygame_widgets

#   ROS1 Messages
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32, Float64, Float32, Float32MultiArray, Bool, ColorRGBA, Header, Int32MultiArray
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Inertia, Polygon, Point, Pose, Point, Quaternion, Vector3, Wrench, WrenchStamped
from gazebo_msgs.msg import LinkStates, PerformanceMetrics, ContactState, ContactsState
from visualization_msgs.msg import Marker, MarkerArray
from nmpc_bringup.msg import pose3D, velocity3D, pose3DStamped, wheelTrueVelocitiesBodyFrame, referencePath, contactForces
from matplotlib.ticker import FormatStrFormatter


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
import matplotlib.patches as patch
import matplotlib.lines as line
import scipy.spatial.distance as dist
import matplotlib.ticker as ticker

#from tf.transformations import quaternion_from_euler
from scipy.spatial.transform import Rotation as R
from matplotlib import cm
from scipy import interpolate
from scipy.interpolate import splprep, splev, CubicSpline
from scipy.interpolate import RegularGridInterpolator
from scipy.optimize import minimize
from scipy import ndimage
from functools import partial
from matplotlib.patches import Ellipse
from matplotlib.lines import Line2D
from matplotlib.transforms import Bbox
from matplotlib.colors import TwoSlopeNorm
from threading import Thread
from tkinter import *
from urdf_parser_py.urdf import URDF
from pygame_widgets.slider import Slider
from pygame_widgets.textbox import TextBox
from nilearn.plotting import show
from nilearn.plotting.cm import _cmap_d as nilearn_cmaps

#   HPIPM (for qp optimization)
from hpipm_python import *
from hpipm_python.common import *

from mpl_toolkits.mplot3d import Axes3D

from PIL import Image

#from simple_pid import PID

#   Acados
from acados_template import AcadosModel, AcadosOcpCost, AcadosOcpConstraints, AcadosOcpDims, AcadosOcp, AcadosOcpSolver, AcadosOcpOptions, AcadosSimSolver, AcadosSimOptions, AcadosSimDims, AcadosSim