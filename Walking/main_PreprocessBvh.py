# +-------------------------------------------------------------------------
# | main_PreprocessBvh.py
# |
# | Author: Yoonsang Lee
# +-------------------------------------------------------------------------
# | COPYRIGHT:
# |    Copyright Yoonsang Lee 2013
# |    See the included COPYRIGHT.txt file for further details.
# |    
# |    This file is part of the DataDrivenBipedController.
# |    DataDrivenBipedController is free software: you can redistribute it and/or modify
# |    it under the terms of the MIT License.
# |
# |    You should have received a copy of the MIT License
# |    along with DataDrivenBipedController.  If not, see <mit-license.org>.
# +-------------------------------------------------------------------------

import psyco; psyco.full()
from fltk import *
import os.path, glob, cPickle
import numpy as np

import sys
if '../PyCommon/modules' not in sys.path:
    sys.path.append('../PyCommon/modules')
import Math.mmMath as mm
import Resource.ysMotionLoader as yf
import Motion.mmAnalyticIK as aik
import Motion.ysSkeletonEdit as yse
import Motion.ysTrajectoryEdit as yte
import GUI.ysSimpleViewer as ysv
import Renderer.ysRenderer as yr
import Motion.ysMotionAnalysis as yma
import Motion.ysBipedAnalysis as yba
import Motion.ysMotionExtend as ymt
import Simulator.ysPhysConfig as ypc
import Renderer.csVpRenderer as cvr
import Simulator.csVpWorld as cvw
import Simulator.csVpModel as cvm
import ArticulatedBody.ysControl as yct

def normalizeSkeleton(motion, config):
    motion.scale(config['scale'], False)

    if config['type']=='woody2':
        yse.removeJoint(motion, 'Head', False)
        yse.removeJoint(motion, 'RightShoulder', False)
        yse.removeJoint(motion, 'LeftShoulder1', False)
        yse.removeJoint(motion, 'RightToes_Effector', False)
        yse.removeJoint(motion, 'LeftToes_Effector', False)
        yse.removeJoint(motion, 'RightHand_Effector', False)
        yse.removeJoint(motion, 'LeftHand_Effector', False)
        yse.offsetJointLocal(motion, 'RightArm', (.03,-.05,0), False)
        yse.offsetJointLocal(motion, 'LeftArm', (-.03,-.05,0), False)
    elif config['type']=='woody2_new':
        yse.removeJoint(motion, 'HEad', False)
        yse.removeJoint(motion, 'RightShoulder', False)
        yse.removeJoint(motion, 'LeftShoulder1', False)
        yse.removeJoint(motion, 'RightToes_Effector', False)
        yse.removeJoint(motion, 'LeftToes_Effector', False)
        yse.removeJoint(motion, 'RightHand_Effector', False)
        yse.removeJoint(motion, 'LeftHand_Effector', False)
        yse.offsetJointLocal(motion, 'RightArm', (.03,-.05,0), False)
        yse.offsetJointLocal(motion, 'LeftArm', (-.03,-.05,0), False)
    elif config['type']=='woody2amc':
        yse.removeJoint(motion, 'RightUpLegDummy', False)
        yse.removeJoint(motion, 'SpineDummy', False)
        yse.removeJoint(motion, 'HEadDummy', False)
        yse.removeJoint(motion, 'LeftShoulder1Dummy', False)
        yse.removeJoint(motion, 'RightShoulderDummy', False)
        yse.removeJoint(motion, 'LeftUpLegDummy', False)
        yse.removeJoint(motion, 'Head', False)
        yse.removeJoint(motion, 'RightShoulder', False)
        yse.removeJoint(motion, 'LeftShoulder1', False)
        yse.removeJoint(motion, 'RightToes_Effector', False)
        yse.removeJoint(motion, 'LeftToes_Effector', False)
        yse.removeJoint(motion, 'RightHand_Effector', False)
        yse.removeJoint(motion, 'LeftHand_Effector', False)
        yse.offsetJointLocal(motion, 'RightArm', (.03,-.05,0), False)
        yse.offsetJointLocal(motion, 'LeftArm', (-.03,-.05,0), False)

    yse.rotateJointLocal(motion, 'LeftFoot', config['footRot'], False)
    yse.rotateJointLocal(motion, 'RightFoot', config['footRot'], False)
    if 'leftFootR' in config:
        yse.rotateJointLocal(motion, 'LeftFoot', config['leftFootR'], False)
#    motion.translateByOffset((0, config['yOffset'], 0))
    
    if 'rootRot' in config:
        motion.rotateByOffset(config['rootRot'])
    
    yse.updateGlobalT(motion)
    
def adjustHeight(motion, halfFootHeight):
    lFoot = motion[0].skeleton.getJointIndex('LeftFoot')
    rFoot = motion[0].skeleton.getJointIndex('RightFoot')
    groundHeight = min([motion[0].getJointPositionGlobal(lFoot)[1], \
                        motion[0].getJointPositionGlobal(rFoot)[1]])
    motion.translateByOffset((0, halfFootHeight-groundHeight, 0))
    
def additionalEdit(motion, path):
    if 'wd2_left_turn.bvh' in path or 'wd2_right_turn.bvh' in path:
        lFoot = motion[0].skeleton.getJointIndex('LeftFoot')
        yte.setPositionTarget(motion, lFoot, motion[0].getJointPositionGlobal(lFoot)+(-.1,0,-.1)\
                              , [0,58], 150)
        
    if 'extended' in path:
        hRef = .15; vRef = .2
        lc = yma.getElementContactStates(motion, 'LeftFoot', hRef, vRef)
        interval = yba.getWalkingCycle(motion, lc)
        motion[:] = ymt.repeatCycle(motion, interval, 50, 10)


def preprocess():
    tasks = []
    
#    outputDir = './ppmotion/'
#    
#    dir = '../Data/woody2/Motion/Physics2/'
#    config = {'repeat':True, 'footRot': mm.rotX(-.4), 'yOffset':0., 'halfFootHeight': 0.0444444444444, 'scale':.01, 'type':'woody2'}
#    paths = []
#    paths.append(dir+'wd2_WalkSameSame01.bvh')
#    paths.append(dir+'wd2_WalkForwardSlow01.bvh')
#    paths.append(dir+'wd2_WalkForwardNormal00.bvh')
#    paths.append(dir+'wd2_WalkHandWav00.bvh')
#    paths.append(dir+'wd2_WalkForwardFast00.bvh')
#    paths.append(dir+'wd2_WalkForwardVFast00.bvh')
#    paths.append(dir+'wd2_WalkLean00.bvh')
#    paths.append(dir+'wd2_WalkAzuma01.bvh')
#    paths.append(dir+'wd2_WalkSoldier00.bvh')
##    paths.append(dir+'wd2_WalkSukiko00.bvh')
#    paths.append(dir+'wd2_WalkBackward00.bvh')
##    paths.append(dir+'wd2_WalkTongTong00.bvh')
##    tasks.append({'config':config, 'paths':paths})
##    
#    dir = '../Data/woody2/Motion/Balancing/'
#    config = {'footRot': mm.exp(mm.v3(1,-.5,0), -.6), 'yOffset': .0, 'halfFootHeight': 0.0444444444444, 'scale':.01, 'type':'woody2'}
#    paths = []
#    paths.append(dir+'wd2_2foot_walk_turn.bvh')
#    paths.append(dir+'wd2_2foot_walk_turn2.bvh')
#    paths.append(dir+'wd2_slow_2foot_hop.bvh')
#    paths.append(dir+'wd2_short_broad_jump.bvh')
#    paths.append(dir+'wd2_long_broad_jump.bvh')
#    paths.append(dir+'wd2_ffast_cancan_run.bvh')
#    paths.append(dir+'wd2_fast_cancan_run.bvh')
#    paths.append(dir+'wd2_fast_2foot_hop.bvh')
#    paths.append(dir+'wd2_1foot_contact_run.bvh')
#    paths.append(dir+'wd2_1foot_contact_run2.bvh')
##    tasks.append({'config':config, 'paths':paths})
#
##    dir = '../Data/woody2/Motion/Samsung/'
##    config = {'footRot': mm.rotX(-.46), 'yOffset': .0, 'halfFootHeight': 0.0444444444444, 'scale':.01*2.54, 'type':'woody2amc'}
##    paths = []
##    paths.append(dir+'wd2_left_turn.bvh')
##    paths.append(dir+'wd2_right_turn.bvh')
##    paths.append(dir+'wd2_pose_inner1.bvh')
##    paths.append(dir+'wd2_pose_inner2.bvh')
##    tasks.append({'config':config, 'paths':paths})
#
#    dir = '../Data/woody2/Motion/Picking/'
#    config = {'footRot': mm.rotX(-.5), 'yOffset': .0, 'halfFootHeight': 0.0444444444444, 'scale':.01, 'type':'woody2'}
#    paths = []
#    paths.append(dir+'wd2_pick_walk_1.bvh')
##    tasks.append({'config':config, 'paths':paths})
#    
#    dir = '../Data/woody2/Motion/VideoMotion/'
#    config = {'footRot': mm.rotX(-.48), 'yOffset': .0, 'halfFootHeight': 0.0444444444444, 'scale':.01*2.54, 'type':'woody2amc'}
##    paths = glob.glob(dir+'*.bvh')
#    paths = []
#    paths.append(dir+'wd2_cross_walk_90d_fast_27.bvh')
##    tasks.append({'config':config, 'paths':paths})
#    
#    dir = '../Data/woody2/Motion/Walking/'
#    config = {'footRot': mm.rotX(-.40), 'yOffset': .0, 'halfFootHeight': 0.0444444444444, 'scale':.01*2.54, 'type':'woody2amc'}
#    paths = glob.glob(dir+'*.bvh')
##    tasks.append({'config':config, 'paths':paths})
#
##    dir = '../Data/woody2/Motion/samsung_boxing/round/'
##    config = {'footRot': mm.rotX(-.65), 'yOffset': .0, 'halfFootHeight': 0.0444444444444, 'scale':.01*2.54, 'type':'woody2amc'}
##    paths = glob.glob(dir+'*.bvh')
##    tasks.append({'config':config, 'paths':paths})
#
##    dir = '../Data/woody2/Motion/samsung_boxing/boxman/'
##    config = {'footRot': mm.rotX(-.5), 'yOffset': .0, 'halfFootHeight': 0.0444444444444, 'scale':.01*2.54, 'type':'woody2amc'}
##    paths = glob.glob(dir+'*.bvh')
##    tasks.append({'config':config, 'paths':paths})
#
#    dir = '../Data/woody2/Motion/motion_edit/'
#    config = {'footRot': mm.rotX(-.5), 'yOffset': .0, 'halfFootHeight': 0.0444444444444, 'scale':.01*2.54, 'type':'woody2amc'}
#    paths = glob.glob(dir+'*.bvh')
##    tasks.append({'config':config, 'paths':paths})
#
##    outputDir = './icmotion_test/'
##    dir = './rawmotion/'
##    config = {'footRot': mm.rotX(-.5), 'yOffset': .0, 'halfFootHeight': 0.0444444444444, 'scale':1., 'type':'woody2_new'}
##    paths = glob.glob(dir+'*.temp')
##    tasks.append({'config':config, 'paths':paths})
#
#    outputDir = './icmotion_last/'
#    dir = './lastmotion/add/'
##    config = {'rootRot':mm.rotZ(.05), 'footRot': mm.rotX(-.5), 'leftFootR':mm.rotZ(-.1), \
#    config = {'rootRot':mm.rotZ(.0), 'footRot': np.dot(mm.rotX(-.5), mm.rotZ(.04)), 'leftFootR':mm.rotZ(-.2), \
#              'halfFootHeight': 0.0444444444444, 'scale':1., 'type':'woody2_new'}
#    paths = glob.glob(dir+'*.temp')
#    tasks.append({'config':config, 'paths':paths})

#    outputDir = './ppmotion_slope/'
#    dir = './rawmotion_slope_extracted/'
#    config = {'rootRot':mm.rotZ(.0), 'footRot': np.dot(mm.rotX(-.55), mm.rotZ(.04)), 'leftFootR':mm.rotZ(-.2), \
#              'halfFootHeight': 0.0444444444444, 'scale':1., 'type':'woody2_new'}
#    paths = glob.glob(dir+'*.bvh')
#    tasks.append({'config':config, 'paths':paths})

    VISUALIZE = False
    
    for task in tasks:
        config = task['config']
        paths = task['paths']
        for path in paths:
            motion = yf.readBvhFile(path)
            normalizeSkeleton(motion, config)
            adjustHeight(motion, config['halfFootHeight'])
            additionalEdit(motion, path)
            
            outputPath = outputDir + os.path.basename(path)
            yf.writeBvhFile(outputPath, motion)
            print outputPath, 'done'
            
            if 'repeat' in config and config['repeat']:
                hRef = .1; vRef = .3
                lc = yma.getElementContactStates(motion, 'LeftFoot', hRef, vRef)
                interval = yba.getWalkingCycle2(motion, lc)
                
                transitionLength = 20 if 'wd2_WalkAzuma01.bvh' in path else 10
                motion = ymt.repeatCycle(motion, interval, 50, transitionLength)

                outputName = os.path.splitext(os.path.basename(path))[0]+'_REPEATED.bvh'
                outputPath = outputDir + outputName
                yf.writeBvhFile(outputPath, motion)
                print outputPath, 'done'
                
            if VISUALIZE:
                viewer = ysv.SimpleViewer()
                viewer.record(False)
                viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,100,255), yr.LINK_LINE))
                viewer.doc.addObject('motion', motion)
                
                viewer.startTimer(1/30.)
                viewer.show()
                Fl.run()
    
    print 'FINISHED'
        
def simulation_test():
    Kt = 20.;       Dt = 2*(Kt**.5)
    Ks = 2000.;    Ds = 2*(Ks**.5)
    mu = 1.

    dir = './icmotion_last/'
    filename = 'stop_left_normal.temp'
#    filename = 'left_left_normal.temp'
#    filename = 'right_left_fast.temp'
    
    motion_ori = yf.readBvhFile(dir+filename)
    frameTime = 1/motion_ori.fps
    
    motion_ori[0:0] = [motion_ori[0]]*20

    mcfgfile = open(dir + 'mcfg', 'r')
    mcfg = cPickle.load(mcfgfile)
    mcfgfile.close()
    
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.
    wcfg.useDefaultContactModel = False
#    wcfg.lockingVel = c_locking_vel
    stepsPerFrame = 30
    wcfg.timeStep = (frameTime)/stepsPerFrame
    
    vpWorld = cvw.VpWorld(wcfg)
    motionModel = cvm.VpMotionModel(vpWorld, motion_ori[0], mcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion_ori[0], mcfg)
    vpWorld.initialize()
    print controlModel
    
    controlModel.initializeHybridDynamics()

    bodyIDsToCheck = range(vpWorld.getBodyNum())
    mus = [mu]*len(bodyIDsToCheck)
    
    viewer = ysv.SimpleViewer()
#    viewer.record(False)
#    viewer.doc.addRenderer('motion_ori', yr.JointMotionRenderer(motion_ori, (0,100,255), yr.LINK_BONE))
    viewer.doc.addObject('motion_ori', motion_ori)
    viewer.doc.addRenderer('motionModel', cvr.VpModelRenderer(motionModel, (0,150,255), yr.POLYGON_LINE))
    viewer.doc.addRenderer('controlModel', cvr.VpModelRenderer(controlModel, (200,200,200), yr.POLYGON_LINE))
    
    def simulateCallback(frame):
        th_r = motion_ori.getDOFPositions(frame)
        th = controlModel.getDOFPositions()
        dth_r = motion_ori.getDOFVelocities(frame)
        dth = controlModel.getDOFVelocities()
        ddth_r = motion_ori.getDOFAccelerations(frame)
        ddth_des = yct.getDesiredDOFAccelerations(th_r, th, dth_r, dth, ddth_r, Kt, Dt)
        
        for i in range(stepsPerFrame):
            bodyIDs, contactPositions, contactPositionLocals, contactForces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
            vpWorld.applyPenaltyForce(bodyIDs, contactPositionLocals, contactForces)
            
            controlModel.setDOFAccelerations(ddth_des)
            controlModel.solveHybridDynamics()
            
            vpWorld.step()
            
        motionModel.update(motion_ori[frame])
                
    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(frameTime / 1.4)
    viewer.show()
    
    Fl.run()
                
if __name__=='__main__':
    preprocess()
#    simulation_test()