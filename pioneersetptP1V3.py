"""
Robot set-point control (regulation) using a Pioneer p3dx from CoppeliaSim

Author: Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
Mobile Robotics course, University of Guanajuato (2020)
"""

import numpy as np
import time
import math as m
import sim # access all the sim elements
import matplotlib.pyplot as plt
import scipy.interpolate as spi
import random as rm

def angdiff(t1, t2):
    """
    Compute the angle difference, t2-t1, restricting the result to the [-pi,pi] range
    """
    # The angle magnitude comes from the dot product of two vectors
    angmag = m.acos(m.cos(t1)*m.cos(t2)+m.sin(t1)*m.sin(t2))
    # The direction of rotation comes from the sign of the cross product of two vectors
    angdir = m.cos(t1)*m.sin(t2)-m.sin(t1)*m.cos(t2)
    return m.copysign(angmag, angdir)

def v2u(v, omega, r, L):
    ur = v/r + L*omega/(2*r)
    ul = v/r - L*omega/(2*r)
    return ur, ul


########
# MAIN #
########

sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',1999,True,True,5000,5) # start a connection
if clientID!=-1:
	print ('Connected to remote API server')
else:
	print('Not connected to remote API server')
	sys.exit("No connection")

# Getting handles for the motors and robot
err, motorL = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/leftMotor', sim.simx_opmode_blocking)
err, motorR = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/rightMotor', sim.simx_opmode_blocking)
err, robot = sim.simxGetObjectHandle(clientID, '/PioneerP3DX', sim.simx_opmode_blocking)

# Assigning handles to the ultrasonic sensors
usensor = []
nSensor = 4

for i in range(0,nSensor):
    err, s = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/ultrasonicSensor['+str(i)+']', sim.simx_opmode_blocking)
    usensor.append(s)

# Sensor initialization
for i in range(nSensor):
    err, state, point, detectedObj, detectedSurfNormVec = sim.simxReadProximitySensor(clientID, usensor[i], sim.simx_opmode_streaming)
time.sleep(1)

xarr = []
yarr = []

# Maximo numero aleatorio en la generacion de la trayectoria
# ( Modificar si es requerido )

Min = -7
Max = 7

xarr.append(0)
yarr.append(0)

# Inizializar trayectoria aleatoria.
for i in range(0,10):
    nr = rm.randint(Min, Max)
    xarr.append(nr)
    nr = rm.randint(Min, Max)
    yarr.append(nr)

# xarr = [0, 5]
# yarr = [0, 0]
xarr = np.array(xarr)
yarr = np.array(yarr)
print(f'X-Y:\n{xarr}\n{yarr}\n')

# Controller gains (linear and heading)
Kv = 0.5
Kh = 0.9

# Medidas del robot
hd = 0
r = 0.5*1.9502e-01
L = 2.0*1.6550e-01
errp = 10


xpos = []
ypos = []

Tt = 300
tnew = np.linspace(0, Tt, 100)
tarr = np.linspace(0, Tt, xarr.shape[0])
# Interpolador
pcix = spi.PchipInterpolator(tarr, xarr)
pciy = spi.PchipInterpolator(tarr, yarr)

ts = time.time()
td = time.time() - ts

tarr = []
xnewArr = []
ynewArr = []

while td < Tt:
    
    td = time.time() - ts
    # tarr.append(td)

    sensor = {}
    for i in range(nSensor):
        err, state, point, detectedObj, detectedSurfNormVec = sim.simxReadProximitySensor(clientID, usensor[i], sim.simx_opmode_buffer)
        sensor[i] = state

    if sensor[1] == True and sensor[2] == True:
        uRight, uLeft = v2u(0.05, -np.pi/9, r, L) # omega = angulo / tiempo
        err = sim.simxSetJointTargetVelocity(clientID, motorL, uLeft, sim.simx_opmode_oneshot)
        err = sim.simxSetJointTargetVelocity(clientID, motorR, uRight, sim.simx_opmode_oneshot)
        # time.sleep(0.35)
        time.sleep(0.5)
        td = time.time() - ts
    if sensor[0]:
        # con -pi/8 funciona para cilindros
        uRight, uLeft = v2u(0.05, -np.pi/10, r, L) # omega = angulo / tiempo
        err = sim.simxSetJointTargetVelocity(clientID, motorL, uLeft, sim.simx_opmode_oneshot)
        err = sim.simxSetJointTargetVelocity(clientID, motorR, uRight, sim.simx_opmode_oneshot)
        # time.sleep(0.3)
        time.sleep(0.5)
        td = time.time() - ts
    elif sensor[1]:
        uRight, uLeft = v2u(0.05, -np.pi/9, r, L) # omega = angulo / tiempo
        err = sim.simxSetJointTargetVelocity(clientID, motorL, uLeft, sim.simx_opmode_oneshot)
        err = sim.simxSetJointTargetVelocity(clientID, motorR, uRight, sim.simx_opmode_oneshot)
        # time.sleep(0.35)
        time.sleep(0.5)
        td = time.time() - ts
    elif sensor[2]:
        uRight, uLeft = v2u(0.05, np.pi/9, r, L) # omega = angulo / tiempo
        err = sim.simxSetJointTargetVelocity(clientID, motorL, uLeft, sim.simx_opmode_oneshot)
        err = sim.simxSetJointTargetVelocity(clientID, motorR, uRight, sim.simx_opmode_oneshot)
        # time.sleep(0.35)
        time.sleep(0.5)
        td = time.time() - ts
    elif sensor[3]:
        uRight, uLeft = v2u(0.05, np.pi/10, r, L) # omega = angulo / tiempo
        err = sim.simxSetJointTargetVelocity(clientID, motorL, uLeft, sim.simx_opmode_oneshot)
        err = sim.simxSetJointTargetVelocity(clientID, motorR, uRight, sim.simx_opmode_oneshot)
        # time.sleep(0.3)
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
        #print('Distance to goal: {}   Heading error: {}'.format(errp, errh))

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
    # time.sleep(0.1)

for i in range(5):
    errf = sim.simxSetJointTargetVelocity(clientID, motorL, 0, sim.simx_opmode_streaming)
    errf = sim.simxSetJointTargetVelocity(clientID, motorR, 0, sim.simx_opmode_streaming)
    #time.sleep(0.1)

diff = np.diff(tarr)
diff = np.mean(diff)
print(f'TIME MEAN:{diff}')

# Graficar tiempo
plt.figure(1)
plt.plot(tarr, xpos)
plt.xlabel('t')
plt.ylabel('x')
plt.title('X - T')

# Graficar tiempo
plt.figure(2)
plt.plot(tarr, ypos)
plt.xlabel('t')
plt.ylabel('y')
plt.title('Y - T')


# Graficar desplazamiento
plt.figure(3)
plt.plot(xpos, ypos)
plt.xlabel('x')
plt.ylabel('y')
plt.title('Desplazamiento')
for x, y in zip(xarr, yarr):
    plt.plot(x, y, 'ro')
    plt.text(x+0.2, y+0.4, '%d, %d' % (int(x), int(y)))


#%%################
# Graficar Camino #
###################

plt.figure(4)
# xnew = pcix(td)
# ynew = pciy(td)
plt.plot(xnewArr, ynewArr, 'b')
# plt.plot(xarr, yarr, '.')
for x, y in zip(xarr, yarr):
    plt.plot(x, y, 'ro')
    plt.text(x+0.2, y+0.4, '%d, %d' % (int(x), int(y)))
plt.title('Path')

plt.figure(5)
plt.plot(xpos, ypos, 'm')
plt.plot(xnewArr, ynewArr, 'b')
# plt.plot(xarr, yarr, '.')
for x, y in zip(xarr, yarr):
    plt.plot(x, y, 'ro')
    plt.text(x+0.2, y+0.4, '%d, %d' % (int(x), int(y)))
plt.title('Path')
plt.show()

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

sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
