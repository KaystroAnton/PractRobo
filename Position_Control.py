import pybullet as p
import numpy as np
import time

# used inverse kinematic and position control
dt = 1 / 240  # pybullet simulation step
th0 = 0.5
jIdx = [0, 1, 3]
maxTime = 2
logTime = np.arange(0.0, maxTime, dt)
sz = len(logTime)
logX = np.zeros(sz)
logY = np.zeros(sz)
logZ = np.zeros(sz)
idx = 0
u = 0
aim = [0.4, 0.3, 1]
L = 0.5

physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -10)
boxId = p.loadURDF("./Position_Control.urdf", useFixedBase=True)

# turn off internal damping
numJoints = p.getNumJoints(boxId)
for i in range(numJoints):
    print(f"{i} {p.getJointInfo(boxId, i)[1]} {p.getJointInfo(boxId, i)[12]}")

# go to the starting position
p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=jIdx, targetPositions=[0, 0, 0],
                            controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

# initial cartesian position
linkState = p.getLinkState(boxId, linkIndex=4)
xSim2 = linkState[0][0]
ySim2 = linkState[0][1]
zSim2 = linkState[0][2]
logX[idx] = xSim2
logY[idx] = ySim2
logZ[idx] = zSim2

# turn off the motor for the free motion
p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=jIdx, targetVelocities=[0, 0, 0],
                            controlMode=p.VELOCITY_CONTROL, forces=[0, 0, 0])

for t in logTime[1:]:
    # get new joint positions
    newJointPos = p.calculateInverseKinematics(bodyIndex=boxId, endEffectorLinkIndex=4, targetPosition=aim)
    # setJointMotorControl for 3 joints
    p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=jIdx, targetPositions=newJointPos,
                                controlMode=p.POSITION_CONTROL)

    p.stepSimulation()

    idx += 1
    linkState = p.getLinkState(boxId, linkIndex=4)
    xSim2 = linkState[0][0]
    ySim2 = linkState[0][1]
    zSim2 = linkState[0][2]
    logX[idx] = xSim2
    logY[idx] = ySim2
    logZ[idx] = zSim2

    time.sleep(dt)

import matplotlib.pyplot as plt

plt.subplot(3, 1, 1)
plt.grid(True)
plt.plot(logTime, logX, label="simX")
plt.plot([logTime[0], logTime[-1]], [aim[0], aim[0]], label='refX')
plt.legend()

plt.subplot(3, 1, 2)
plt.grid(True)
plt.plot(logTime, logY, label="simY")
plt.plot([logTime[0], logTime[-1]], [aim[1], aim[1]], label='refY')
plt.legend()

plt.subplot(3, 1, 3)
plt.grid(True)
plt.plot(logTime, logZ, label="simZ")
plt.plot([logTime[0], logTime[-1]], [aim[2], aim[2]], label='refZ')
plt.legend()

plt.show()

p.disconnect()