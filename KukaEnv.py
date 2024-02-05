import pybullet as p
import pybullet_data as pbd
import os
import numpy as np
import imageio
TIME_STEP = 1.0/240.0


class KukaEnv():

    def __init__(self, gifsave=False):
        p.connect(p.GUI)
        print(gifsave)
        p.setAdditionalSearchPath(pbd.getDataPath())
        p.setGravity(0,0,-9.81)
        p.loadURDF("plane.URDF", [0,0,0], [0,0,0,1])

        self.kukaId = p.loadURDF("kuka_iiwa/model.urdf", [0,0,0], [0,0,0,1])
        self.joint_info()
        self.joint_limits()
        self.reset_state()
        self.trajectories()

        target = p.getDebugVisualizerCamera()[11]
        p.resetDebugVisualizerCamera( cameraDistance=1.2,cameraYaw=60, cameraPitch=-15,cameraTargetPosition=[0.0,0.0,0.5])
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # Disable the GUI, this can be important
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 1)
        self.counter = 0
        self.kp = 2
        self.Kd = 5
        self.gifsave=gifsave
        self.temp =[]

    def joint_info(self):

        '''
        For information like the number of joints and their limits
        '''

        self.numjoints = p.getNumJoints(self.kukaId)
        self.jointIds = []
        for i in range(self.numjoints):
            self.jointIds.append(p.getJointInfo(self.kukaId, i)[0])
        
        self.pos_limit = []
        self.vel_limit = []
        self.torque_limit = []
        for i in self.jointIds:
            info = p.getJointInfo(self.kukaId, i)
            self.pos_limit.append([info[8], info[9]])
            self.vel_limit.append(info[11])
            self.torque_limit.append(info[10])
    
    def joint_states(self):

        '''
        To return the joint states
        '''

        joint_pos = []
        joint_vel = []
        joint_torques = []
        jointstates = p.getJointStates(self.kukaId, self.jointIds)
        for i in jointstates:
            joint_pos.append(i[0])
            joint_vel.append(i[1])
            joint_torques.append(i[-1])
        
        return joint_pos, joint_vel, joint_torques

    def joint_limits(self):

        '''
        To return the joint limits
        '''
        return self.pos_limit, self.vel_limit, self.torque_limit
    
    def reset_state(self):

        """
        To set the initial pose of the 
        """

        m = self.invkin([0.4, 0.3, 0.65])
        
        for i in self.jointIds:
            p.resetJointState(self.kukaId, i, m[i])
        pass

    def invkin(self, target):
        return p.calculateInverseKinematics(self.kukaId, 6, target)
    
    def trajectories(self):
        duration = 10
        self.steps = int(1/TIME_STEP)
        f = 0.5
        self.steps = int(1/(f*TIME_STEP))
        t = np.linspace(0,int(1/f),self.steps)
        x = [0.4 for i in t]
        y = [0.1 + 0.2*np.cos(2*np.pi*f*i) for i in t]
        z = [0.65+ 0.15*np.sin(2*np.pi*f*i) for i in t]
        self.ref_joint_angles = []
        targs = []
        for i in range(self.steps):
            targs.append([x[i], y[i], z[i]])
            self.ref_joint_angles.append(self.invkin([x[i], y[i], z[i]]))
        self.visualize(targs)

    def controller(self):
        m=0
        for joint_index, ref_angle in enumerate(self.ref_joint_angles[self.counter]):
            p.setJointMotorControl2(self.kukaId, joint_index, controlMode=p.POSITION_CONTROL, targetPosition=ref_angle)#, positionGain = self.kp, velocityGain=self.Kd)
    

    def visualize(self, targ):

        for i in range(len(targ)-1):
            p.addUserDebugLine(targ[i+1], targ[i], [0,0,1])
        p.addUserDebugLine(targ[-1], targ[0],[0,0,1])


    def step(self, trajectory=True):
        if trajectory==True:
            self.counter = self.counter%int(self.steps)
            self.controller()
            self.gifsaver(self.gifsave)
            self.counter+=1
        p.stepSimulation()

    def gifsaver(self, save=False):
        if save==True:
            if (self.counter<self.steps-1):
                if (self.counter%4==0):
                    self.temp.append(np.array(p.getCameraImage(width=640,height=480,lightDirection=[1, 1, 1],renderer=p.ER_BULLET_HARDWARE_OPENGL,flags=p.ER_NO_SEGMENTATION_MASK )[2]))

            else:
                imageio.mimsave('kuka1.gif',self.temp, format='GIF',fps=50)
                self.gifsave=0
    
    def getState():
        pass
 