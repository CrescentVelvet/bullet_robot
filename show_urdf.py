#!/usr/bin/env python
import pybullet as p
import sys
import pybullet_data
import numpy as np
import time

# 运行 python show_urdf.py "dancer_urdf_model/model/dancer_urdf_model.URDF" 以查看urdf模型质心

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("should input a urdf name, a format like */*/*.urdf")
        sys.exit(0)
    urdf_file_name = sys.argv[1]
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,0)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0.2]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF(urdf_file_name,cubeStartPos, cubeStartOrientation)
num_joint = p.getNumJoints(boxId)
link_pos = np.zeros((num_joint + 1,3))
obj_list = np.zeros(num_joint + 1)
# 初始化模型关节设置
for j in range(num_joint):
    p.resetJointState(boxId, j, targetValue=0.0, targetVelocity=0.0)
# 查看模型各部件的质心
for i in range (1):
    p.stepSimulation()
    state_base = p.getBasePositionAndOrientation(boxId)
    link_pos[0,:] = state_base[0][:3]
    obj_list[0] = p.loadURDF(r'dancer_urdf_model/model/sphere_1cm.urdf', link_pos[0,:], useMaximalCoordinates=True)
    for m in np.arange(num_joint): 
        link_pos[m+1,:] = p.getLinkState(boxId,m)[0][:3]
        obj_list[m+1] = p.loadURDF(r'dancer_urdf_model/model/sphere_1cm.urdf', link_pos[m+1,:], useMaximalCoordinates=True)
    for im in np.arange(-1,num_joint):
       p.changeVisualShape(boxId, im, rgbaColor=[0.5, 0.5, 0.5, 0.2])   
    time.sleep(1000000)
p.disconnect()