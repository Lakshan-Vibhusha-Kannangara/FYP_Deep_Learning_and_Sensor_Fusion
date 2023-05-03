# -*- coding: utf-8 -*-
"""
Created on Thu Jan  5 17:56:41 2023

@author: User
"""

import numpy as np
import math
import time
import matplotlib.pyplot as plt
# import time
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
rospy.init_node('listener', anonymous=True)
#############################################
joint_states = [0, 0, 0, 0]
altitude = 0.0
latitude = 0.0


def callback(data):
    global imu_acceleration_x
    imu_acceleration_x = data.linear_acceleration.x
    # print the actual message in its raw format
    rospy.loginfo("imu: %s", data.linear_acceleration.x)
    # otherwise simply print a convenient message on the terminal


def callback2(data):
    global joint_states
    joint_states = data.position
    # print the actual message in its raw format
    # rospy.loginfo("joint states: %s", data.position)

    # otherwise simply print a convenient message on the terminal


def callback3(data):
    global latitude
    global altitude
    altitude = data.altitude
    latitude = data.latitude


class ExtendedKalmanFilter:
    def __init__(self, state_dim, measurement_dim):
        self.state_dim = state_dim
        self.measurement_dim = measurement_dim

        # Initialize state and covariance matrices
        self.state = np.zeros((state_dim, 1))
        self.covariance = np.eye(state_dim)

    def predict(self, F, Q):
        """
        Perform the prediction step of the EKF.
        This involves predicting the state and covariance of the system at the next time step.
        """
        self.state = np.dot(F, self.state)
        self.covariance = np.dot(F, self.covariance).dot(F.T) + Q

    def update(self, H, R, measurement):
        """
        Perform the update step of the EKF.
        This involves using the measurement to update the estimate of the state and covariance of the system.
        """
        innovation = measurement - np.dot(H, self.state)
        innovation_covariance = np.dot(H, self.covariance).dot(H.T) + R
        kalman_gain = np.dot(self.covariance, H.T).dot(
            np.linalg.inv(innovation_covariance))
        self.state = self.state + np.dot(kalman_gain, innovation)
        self.covariance = self.covariance - \
            np.dot(kalman_gain, H).dot(self.covariance)


# Initialize the EKF with the appropriate state and measurement dimensions
ekf = ExtendedKalmanFilter(state_dim=3, measurement_dim=3)

# Set the process and measurement noise covariance matrices
Q = np.eye(3) * 0.01
R = np.eye(3) * 0.1

# Set the state transition matrix
F = np.eye(3)

# Set the measurement matrix
H = np.eye(3)


###########################################


dt = 0.2
ts = 400
t = np.arange(0, ts+dt, dt)

w = 0.483
r = 0.13

factor = 0.025
x_scale = 20

kp = 1.7
kr = (1.7)**0.5*2

theta_res_dot = np.array([0, 0]).reshape(2, 1)
x_res_n = np.array([[0], [0], [0]])
x_res_o = np.array([0, 0, 0]).reshape(3, 1)
thetam_doubledot = np.array([0, 0]).reshape(2, 1)
thetam_dot = np.array([0, 0]).reshape(2, 1)
theta_res = np.array([0, 0]).reshape(2, 1)
v_pk = 0
psi_o = 0


x0 = 0
y0 = 0
psi0 = math.pi/4

eta = np.array([[], [], []])
x_gps = np.array([[], [], []])
x_kf = np.array([[], [], []])
x_kf_o = 0
x_kf_n = 0
x_d = 0  # derivation of kf

eta_dot = np.array([[], [], []])
# encoder
x_enco = np.array([[], []])

x = np.array([[], [], []])
theta_res_o = np.array([0, 0]).reshape(2, 1)

theta_res_ch = np.array([0, 0]).reshape(2, 1)
theta_res_dot_ch = np.array([0, 0]).reshape(2, 1)


# x_d=np.array([[],[],[]])
endt = 0.00
#start1 = 0.00

a1 = 0.0
a2 = 0.0
b1 = 0.0
b2 = 0.0
Re = 6378000
xgps1 = 0.0
ygps1 = 0.0
thetagps1 = 0.0

for i in range(len(t)):
    # rospy.Subscriber("imu", Imu, callback)
    time.sleep(0.2)
    rospy.Subscriber("jaguar_robot/joint_states", JointState, callback2)
    rospy.Subscriber("gps/fix", NavSatFix, callback3)

    # print([joint_states[1]*-1, joint_states[0]])

    eta = np.append(eta, np.array([t[i]/x_scale, math.sin(factor*t[i]), math.atan(
        x_scale*factor*math.cos(factor*t[i]))]).reshape(3, 1), axis=1)

    eta_dot = np.append(eta_dot, np.array([1/x_scale, factor*math.cos(factor*t[i]), -x_scale*factor**2*math.sin(
        t[i])/(1+(x_scale*factor*math.cos(factor*t[i]))**2)]).reshape(3, 1), axis=1)

    psi_n = (theta_res[0, 0]-theta_res[1, 0])*r/w

    j_psi = np.array([[(r/2)*math.cos(psi_n), (r/2)*math.cos(psi_n)],
                     [(r/2)*math.sin(psi_n), (r/2)*math.sin(psi_n)],
                     [r/w, -r/w]])

    j_psi_inv = 1/r*np.array([[math.cos(psi_n), math.sin(psi_n), w/2],
                             [math.cos(psi_n), math.sin(psi_n), -w/2]])
    x_res_dot = np.dot(j_psi, theta_res_dot)

    v_pk = (x_res_dot[0, 0]**2+x_res_dot[1, 0]**2)**0.5

    x_res_n = np.array([x_res_n[0, 0]+v_pk*math.cos((psi_n+psi_o)/2)*dt,
                       x_res_n[1, 0]+v_pk*math.sin((psi_n+psi_o)/2)*dt, psi_n]).reshape(3, 1)

    x_res_o = x_res_n
    x = np.append(x, x_res_n, axis=1)
# X from encoder readings
    # gpsreadings
    #x_gps_n = x_res_n+np.random.randn(3, 1)*0.02
    #a2 = altitude
    #b2 = latitude
    #xgps2 = Re*(a2-a1)*np.pi/180*np.cos(b1)
    #ygps2 = Re*(b2-b1)*np.pi/180
    #thetagps2 = math.atan((ygps2-ygps1)/(xgps2-xgps1))
    #x_gps_n = np.array([xgps2, ygps2, thetagps2]).reshape(3, 1)
    # print(x_gps_n)
    x_gps_n = eta[:, i].reshape(3, 1)+np.random.randn(3, 1)*0.02
    x_gps = np.append(x_gps, x_gps_n, axis=1)

    #a1 = a2
    #b1 = b2
    #xgps1 = xgps2
    #ygps1 = ygps2
    #thetagps1 = thetagps2

    ekf.predict(F, Q)

    # Use one of the location data to update the estimate
    ekf.update(H, R, x_res_n)
    ekf.update(H, R, x_gps_n)

    x_kf_n = ekf.state
    x_d = (x_kf_n-x_kf_o)/dt

    x_kf_o = x_kf_n
    x_kf = np.append(x_kf, x_kf_n, axis=1)

    # x_d = np.append(x_d,x_res_dot, axis=1)
    psi_o = psi_n

    x_doubledot = (eta[:, i].reshape(3, 1)-x_kf_n)*kp + \
        (eta_dot[:, i].reshape(3, 1)-x_d)*kr

    thetam_doubledot = np.dot(j_psi_inv, x_doubledot)
###################################################################
    thetam_dot = thetam_dot+thetam_doubledot*dt
    # give theta_ma input to the glazibo
    pub1 = rospy.Publisher(
        '/jaguar_robot/joint1_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher(
        '/jaguar_robot/joint2_position_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher(
        '/jaguar_robot/joint3_position_controller/command', Float64, queue_size=10)
    pub4 = rospy.Publisher(
        '/jaguar_robot/joint4_position_controller/command', Float64, queue_size=10)
    pub1.publish(thetam_dot[1]*-1)
    pub2.publish(thetam_dot[0]*1)
    pub3.publish(thetam_dot[1]*-1)
    pub4.publish(thetam_dot[0]*1)
    #start1 = time.time()

    #theta_res = theta_res+thetam_dot*dt
    #theta_res_dot = thetam_dot

    #x_enco = np.array([joint_states[0]*-1, joint_states[1]]).reshape(2, 1)
    theta_res = np.array(
        [joint_states[1]*1, joint_states[0]*-1]).reshape(2, 1)
    theta_res_dot = (theta_res-theta_res_o)/dt
    theta_res_o = theta_res

####################################################################

pub1 = rospy.Publisher(
    '/jaguar_robot/joint1_position_controller/command', Float64, queue_size=10)
pub2 = rospy.Publisher(
    '/jaguar_robot/joint2_position_controller/command', Float64, queue_size=10)
pub3 = rospy.Publisher(
    '/jaguar_robot/joint3_position_controller/command', Float64, queue_size=10)
pub4 = rospy.Publisher(
    '/jaguar_robot/joint4_position_controller/command', Float64, queue_size=10)
pub1.publish(0)
pub2.publish(0)
pub3.publish(0)
pub4.publish(0)

plt.plot(eta[0, :], eta[1, :], "c", label='ground truth')
plt.plot(x_kf[0, :], x_kf[1, :], "r", label='ground truth')
plt.scatter(x_gps[0, :], x_gps[1, :], s=3, color='g', label='gps')
plt.scatter(x[0, :], x[1, :], s=3, color='b', label='encoder')
#print("this is encoder reding from sim")
# print(theta_res)
#print("this is encoder reding from gazebo")
# print(x_enco)
# Add a legend

plt.legend()

# Show the plot
plt.show()


# gps data
# gps_x=x[0,:]+np.random.normal(0, 0.008,np.size(x[0,:]))
# gps_y= x[1,:]+np.random.normal(0, 0.008,np.size(x[1,:]))
# gps_theta=x[2,:]+ np.random.normal(0,0.008,np.size(x[2,:]))

# x_gps=np.array([gps_x,gps_y,gps_theta])
# #accelarometer data
# acc_x=x[0,:]+np.random.normal(0, 0.05,np.size(x[0,:]))
# acc_y= x[1,:]+np.random.normal(0, 0.05,np.size(x[1,:]))
# acc_theta=x[2,:]+ np.random.normal(0,0.05,np.size(x[2,:]))

# x_acc=np.array([acc_x,acc_y,acc_theta])
