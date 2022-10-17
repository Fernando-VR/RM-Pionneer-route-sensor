"""
Robot set-point control (regulation) using a Pioneer p3dx from CoppeliaSim

Author: Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
Mobile Robotics course, University of Guanajuato (2020)
"""
from tkinter import NS
import numpy as np
import math as m
import random as rm
import time, sys, sim
import matplotlib.pyplot as plt
import scipy.interpolate as spi

def angdiff(t1, t2):
    """ The code does the following:
    1. Define the function angdiff
    2. Calculate the difference between the two angles
    3. If the difference is greater than pi, subtract 2pi
    4. If the difference is less than -pi, add 2pi
    5. Return the difference 
    """
    # The angle magnitude comes from the dot product of two vectors
    angmag = m.acos(m.cos(t1)*m.cos(t2)+m.sin(t1)*m.sin(t2))
    # The direction of rotation comes from the sign of the cross product of two vectors
    angdir = m.cos(t1)*m.sin(t2)-m.sin(t1)*m.cos(t2)
    return m.copysign(angmag, angdir)

def v2u(v, omega, r, L):
    """ The code does the following:
    1. Convert the linear and angular velocities to the left and right wheel velocities
    2. Scale the wheel velocities to the range [0, 1]
    3. Return the scaled wheel velocities
    """
    ur = v/r + L*omega/(2*r)
    ul = v/r - L*omega/(2*r)
    return ur, ul

def remoteConection(port):
    """ The code does the following:
    1. Connects to the remote API server.
    2. Checks if the connection to the server was successful.
    3. If the connection was successful, the function will return the clientID. """
    # Connecting to the remote API server
    sim.simxFinish(-1) # just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',port,True,True,5000,5) # start a connection
    if clientID!=-1:
        print ('Connected to remote API server')
    else:
        print('Not connected to remote API server')
        sys.exit("No connection")
    return clientID

def getHandles(clientID):
    """  The code does the following:
    1. It gets the handles for the robot and the motors.
    2. If the connection was successful, the function will return the robot, left_motor, right_motor."""
    # Getting handles for the motors and robot
    err, motorL = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/leftMotor', sim.simx_opmode_blocking)
    err, motorR = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/rightMotor', sim.simx_opmode_blocking)
    err, robot = sim.simxGetObjectHandle(clientID, '/PioneerP3DX', sim.simx_opmode_blocking)
    return robot, motorL, motorR

def getUSensors(clientID, nSensor):
    """  The code does the following:
    1. Creates a list named usensor.
    2. Populates the list with nSensors ultrasonic sensors.
    3. For each sensor, it sets the mode to sim.simx_opmode_streaming, which means that the sensor will send data to the client every time the simulation steps forward.
    4. Returns the list of ultrasonic sensors. """
    usensor = []
    for i in range(nSensor):
        err, s = sim.simxGetObjectHandle(clientID, f'/PioneerP3DX/ultrasonicSensor[{str(i)}]', sim.simx_opmode_blocking)
        usensor.append(s)
    # Sensor initialization
    for i in range(nSensor):
        err, state, point, detectedObj, detectedSurfNormVec = sim.simxReadProximitySensor(clientID, usensor[i], sim.simx_opmode_streaming)
    return usensor

def randomTrajectory(Min, Max):
    xarr = [0]
    yarr = [0]
    for i in range(0,10):
        nr = rm.randint(Min, Max)
        xarr.append(nr)
        nr = rm.randint(Min, Max)
        yarr.append(nr)
    xarr = np.array(xarr)
    yarr = np.array(yarr)
    return xarr, yarr

def plotTime():
    """ The code does the following:
    1. The plotTime() function plots the x and y position versus time.
    2. The time array is created in the main function and is then passed to the plotTime() function.
    3. The xpos and ypos arrays are also created in the main function and are passed to the plotTime() function.
    4. The plotTime() function has two subplots, one for x versus t and the other for y versus t.
    5. The x and y labels are set for each subplot.
    6. The title of each subplot is set.
    7. plt.show() shows the plot. """
    plt.figure(1)
    plt.plot(tarr, xpos)
    plt.xlabel('t')
    plt.ylabel('x')
    plt.title('X - T')
    plt.figure(2)
    plt.plot(tarr, ypos)
    plt.xlabel('t')
    plt.ylabel('y')
    plt.title('Y - T')
    plt.show()

def plotXY():
    """ The code does the following:
    1. Define the function plotXY
    2. Create a new figure with number 3
    3. Plot the values of xpos and ypos
    4. Label the x-axis as x, and the y-axis as y
    5. Set the title of the plot as Desplazamiento
    6. Plot a red dot for each point in the xarr and yarr arrays
    7. Label each red dot with the corresponding x and y coordinates
    8. Show the plot """
    plt.figure(3)
    plt.plot(xpos, ypos)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Desplazamiento')
    for x, y in zip(xarr, yarr):
        plt.plot(x, y, 'ro')
        plt.text(x+0.2, y+0.4, '%d, %d' % (int(x), int(y)))
    plt.show()

def plotPath():
    """ The code does the following:
    1. The code first creates a 2D grid of 100 x 100 cells
    2. The code then creates a random list of 10 points within the grid
    3. The code then calculates the distance between each point in the random list
    4. The code then calculates the shortest path that connects all the points
    5. The code then plots the path on a graph
    6. The robot takes a step in the direction of the goal
    7. The robot takes another step in a random direction
    8. The robot takes another step in the direction of the goal
    9. The robot takes another step in a random direction
    10. Repeat steps 7-9 10 times """
    plt.figure(4)
    plt.plot(xnewArr, ynewArr, 'b')
    for x, y in zip(xarr, yarr):
        plt.plot(x, y, 'ro')
        plt.text(x+0.2, y+0.4, '%d, %d' % (int(x), int(y)))
    plt.title('Path')
    plt.figure(5)
    plt.plot(xpos, ypos, 'm')
    plt.plot(xnewArr, ynewArr, 'b')
    for x, y in zip(xarr, yarr):
        plt.plot(x, y, 'ro')
        plt.text(x+0.2, y+0.4, '%d, %d' % (int(x), int(y)))
    plt.title('Path')
    plt.show()

def pathObstacles():
    """ The code does the following:
    1. The first while is used to verify which sensor is being activated
    2. If sensor 1 and sensor 2 are activated, the robot will move forward
    3. If only sensor 0 is activated, the robot will move to the left
    4. If only sensor 1 is activated, the robot will move to the left
    5. If only sensor 2 is activated, the robot will move to the right
    6. If only sensor 3 is activated, the robot will move to the right
    7. If none of the sensors are activated, the robot will move using the path planning algorithm """
    # Initialize time control
    ts = time.time()
    td = time.time() - ts
    while td < Tt:
        td = time.time() - ts
        sensor = {}
        for i in range(nSensor):
            err, state, point, detectedObj, detectedSurfNormVec = sim.simxReadProximitySensor(clientID, usensor[i], sim.simx_opmode_buffer)
            sensor[i] = state
        if sensor[1] and sensor[2]:
            uRight, uLeft = v2u(0.05, -np.pi/9, r, L) # omega = angulo / tiempo
            err = sim.simxSetJointTargetVelocity(clientID, motorL, uLeft, sim.simx_opmode_oneshot)
            err = sim.simxSetJointTargetVelocity(clientID, motorR, uRight, sim.simx_opmode_oneshot)
            time.sleep(0.5)
            td = time.time() - ts
        if sensor[0]:
            # con -pi/8 funciona para cilindros
            uRight, uLeft = v2u(0.05, -np.pi/10, r, L) # omega = angulo / tiempo
            err = sim.simxSetJointTargetVelocity(clientID, motorL, uLeft, sim.simx_opmode_oneshot)
            err = sim.simxSetJointTargetVelocity(clientID, motorR, uRight, sim.simx_opmode_oneshot)
            time.sleep(0.5)
            td = time.time() - ts
        elif sensor[1]:
            uRight, uLeft = v2u(0.05, -np.pi/9, r, L) # omega = angulo / tiempo
            err = sim.simxSetJointTargetVelocity(clientID, motorL, uLeft, sim.simx_opmode_oneshot)
            err = sim.simxSetJointTargetVelocity(clientID, motorR, uRight, sim.simx_opmode_oneshot)
            time.sleep(0.5)
            td = time.time() - ts
        elif sensor[2]:
            uRight, uLeft = v2u(0.05, np.pi/9, r, L) # omega = angulo / tiempo
            err = sim.simxSetJointTargetVelocity(clientID, motorL, uLeft, sim.simx_opmode_oneshot)
            err = sim.simxSetJointTargetVelocity(clientID, motorR, uRight, sim.simx_opmode_oneshot)
            time.sleep(0.5)
            td = time.time() - ts
        elif sensor[3]:
            uRight, uLeft = v2u(0.05, np.pi/10, r, L) # omega = angulo / tiempo
            err = sim.simxSetJointTargetVelocity(clientID, motorL, uLeft, sim.simx_opmode_oneshot)
            err = sim.simxSetJointTargetVelocity(clientID, motorR, uRight, sim.simx_opmode_oneshot)
            time.sleep(0.5)
            td = time.time() - ts
        else:

            ret, carpos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_blocking)
            ret, carrot = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)

            td = time.time() - ts

            xnew = pcix(td)
            ynew = pciy(td)
            
            xnewArr.append(xnew)
            ynewArr.append(ynew)

            errp = m.sqrt((xnew-carpos[0])**2 + (ynew-carpos[1])**2)
            angd = m.atan2(ynew-carpos[1], xnew-carpos[0])
            errh = angdiff(carrot[2], angd)

            v = Kv*errp
            omega = Kh*errh

            if v > 1:
                v = 0.5
            if omega > np.pi/2:
                omega = np.pi/4
            ur, ul = v2u(v, omega, r, L)
            errf = sim.simxSetJointTargetVelocity(clientID, motorL, ul, sim.simx_opmode_oneshot)
            errf = sim.simxSetJointTargetVelocity(clientID, motorR, ur, sim.simx_opmode_oneshot)
                
        xpos.append(carpos[0])
        ypos.append(carpos[1])
        tarr.append(td)
    for i in range(5):
        errf = sim.simxSetJointTargetVelocity(clientID, motorL, 0, sim.simx_opmode_streaming)
        errf = sim.simxSetJointTargetVelocity(clientID, motorR, 0, sim.simx_opmode_streaming)

def diff_mean():
    """ The code does the following:
    1. We use the np.diff function to get the difference between each time point in the array. 
    2. Then we use the np.mean function to get the mean of the differences. 
    3. Finally, we print the result. """
    diff = np.diff(tarr)
    diff = np.mean(diff)
    print(f'TIME MEAN:{diff}')

########
# MAIN #
########
clientID = remoteConection(1999)
robot, motorL, motorR = getHandles(clientID)

# Assigning handles to the ultrasonic sensors
nSensor = 4
usensor = getUSensors(clientID, nSensor)

# Maximum random number in the generation of the trajectory
# (Modify if required)
Min = -7
Max = 7

# Initialize random path.
xarr, yarr = randomTrajectory(Min, Max)
print(f'X-Y:\n{xarr}\n{yarr}\n')

# Controller gains (linear and heading)
Kv = 0.5
Kh = 0.9

# Robot measurements
hd = 0
r = 0.5*1.9502e-01
L = 2.0*1.6550e-01
errp = 10

# Initialize control path.
Tt = 300
tnew = np.linspace(0, Tt, 100)
tarr = np.linspace(0, Tt, xarr.shape[0])

# Interpolador
pcix = spi.PchipInterpolator(tarr, xarr)
pciy = spi.PchipInterpolator(tarr, yarr)

# Initialize arrays for plotting
xpos = []
ypos = []
tarr = []
xnewArr = []
ynewArr = []

# Main loop
pathObstacles()

# Mean time between each point
diff_mean()

#%%################
#    Plotting     #
###################
# Graph time
plotTime()
# Graph displacement
plotXY()
# Graph path
plotPath()

# #%%##############################
# # Graficar posicion y velocidad #
# #################################
# plt.figure(2)
# xdot = spi.splev(tnew, pcix, der=1)
# ydot = spi.splev(tnew, pciy, der=1)
# plt.plot(tnew, xnew, 'b', label='x')
# plt.plot(tnew, ynew, 'r', label='y')
# plt.plot(tnew, xdot, 'c', label='xdot')
# plt.plot(tnew, ydot, 'm', label='ydot')
# plt.plot(tarr, xarr, '.')
# plt.plot(tarr, yarr, '.')
# plt.legend()
# plt.title('Position and velocity over time')
# plt.show()

# Stop the simulation
sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)