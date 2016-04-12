# +-------------------------------------------------------------------------
# | main_PreprocessSeg.py
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
import os.path, glob, cPickle,pprint
import numpy as np

import sys
if '../PyCommon/modules' not in sys.path:
    sys.path.append('../PyCommon/modules')
import Math.mmMath as mm
import Resource.ysMotionLoader as yf
import GUI.ysSimpleViewer as ysv
import Renderer.ysRenderer as yr
import Motion.ysMotionAnalysis as yma
import Motion.ysBipedAnalysis as yba
import Simulator.ysPhysConfig as ypc
import Simulator.csVpWorld as cvw
import Simulator.csVpModel as cvm
import ArticulatedBody.ysReferencePoints as yrp


if __name__=='__main__':
#    dir = './icmotion_test/'
#    paths = glob.glob(dir+'*.temp')

#    dir = './ppmotion/'
#    paths = glob.glob(dir+'*.bvh')
#    paths = glob.glob(dir+'wd2_WalkSameSame01.bvh')
#    paths = glob.glob(dir+'wd2_u-turn_1.bvh')
#    paths = glob.glob(dir+'wd2_cross_walk*.bvh')
#    paths = glob.glob(dir+'*_REPEATED.bvh')
#    paths = glob.glob(dir+'wd2_pick_walk_1.bvh')
#    paths = glob.glob(dir+'wd2_WalkSameSame01_REPEATED_FOOT.bvh')
#    paths = glob.glob(dir+'wd2_WalkForwardSlow01_REPEATED_FOOT.bvh')
#    paths = glob.glob(dir+'wd2_WalkSoldier00_REPEATED_FOOT.bvh')

#    hRef = .1; vRef = .4
##    hRef = .1; vRef = .2

    dir = './ppmotion_long/'
    paths = glob.glob(dir+'wd2_WalkBackward00_REPEATED.bvh')
    hRef = 10.; vRef = .4


#    dir = './rawmotion_slope/'
#    paths = glob.glob(dir+'*.bvh')
##    paths = [dir+'woddy2_walk_normal_to_slope.bvh'] 
#    hRef = 10000.; vRef = .4*100
#    
#    dir = './ppmotion_slope/'
#    paths = glob.glob(dir+'*.bvh')
#    hRef = 10000.; vRef = .2
    
    jumpThreshold = 15; jumpBias = 1.
    stopThreshold = 15; stopBias = 0.

    for path in paths:
        motion_ori = yf.readBvhFile(path)
        
        # informations
        skeleton = motion_ori[0].skeleton

        lFoot = skeleton.getJointIndex('LeftFoot'); rFoot = skeleton.getJointIndex('RightFoot')
        lHip = skeleton.getJointIndex('LeftUpLeg'); rHip = skeleton.getJointIndex('RightUpLeg')
        lKnee = skeleton.getJointIndex('LeftLeg');  rKnee = skeleton.getJointIndex('RightLeg')
        lFoot = skeleton.getJointIndex('LeftFoot'); rFoot = skeleton.getJointIndex('RightFoot')
        
#        mcfgfile = open(dir + 'mcfg', 'r')
#        mcfg = cPickle.load(mcfgfile)
#        mcfgfile.close()
#        wcfg = ypc.WorldConfig()
#        vpWorld = cvw.VpWorld(wcfg)
#        motionModel = cvm.VpMotionModel(vpWorld, motion_ori[0], mcfg)
#        
##        bodyMasses = getBodyMasses()
#        bodyMasses = motionModel.getBodyMasses()
#        uppers = [skeleton.getJointIndex(name) for name in ['Hips', 'Spine', 'Spine1', 'LeftArm', 'LeftForeArm', 'RightArm', 'RightForeArm']]
#        upperMass = sum([bodyMasses[i] for i in uppers])
        
        lc = yma.getElementContactStates(motion_ori, 'LeftFoot', hRef, vRef)
        rc = yma.getElementContactStates(motion_ori, 'RightFoot', hRef, vRef)
#        intervals, states = yba.getBipedGaitIntervals(lc, rc, jumpThreshold, jumpBias, stopThreshold, stopBias)
        intervals, states = yba.getBipedGaitIntervals2(lc, rc, jumpThreshold, jumpBias, stopThreshold, stopBias)
        
        seginfos = [{} for i in range(len(intervals))]
        for i in range(len(intervals)):
            start = intervals[i][0]; end = intervals[i][1]
             
            seginfos[i]['interval'] = intervals[i]
            seginfos[i]['state'] = states[i]
#            print yba.GaitState.text[states[i]], intervals[i]
            
            stanceHips = []; swingHips = []; stanceFoots = []; swingFoots = []; swingKnees = []
            if states[i]==yba.GaitState.LSWING:   stanceHips = [rHip]; stanceFoots = [rFoot]; swingHips = [lHip]; swingFoots = [lFoot]; swingKnees = [lKnee]
            elif states[i]==yba.GaitState.RSWING: stanceHips = [lHip]; stanceFoots = [lFoot]; swingHips = [rHip]; swingFoots = [rFoot]; swingKnees = [rKnee]
            elif states[i]==yba.GaitState.STOP:   stanceHips = [rHip, lHip]; stanceFoots = [rFoot, lFoot]
            elif states[i]==yba.GaitState.JUMP:   swingHips = [rHip, lHip]; swingFoots = [rFoot, lFoot]
            seginfos[i]['stanceHips'] = stanceHips
            seginfos[i]['swingHips'] = swingHips
            seginfos[i]['stanceFoots'] = stanceFoots
            seginfos[i]['swingFoots'] = swingFoots
            seginfos[i]['swingKnees'] = swingKnees
            
#            if start<end:
#                seginfos[i]['ground_height'] = min([posture_seg.getJointPositionGlobal(foot)[1] for foot in [lFoot, rFoot] for posture_seg in motion_ori[start+1:end+1]])
#            
#                seginfos[i]['max_stf_push_frame'] = None
#                if len(swingFoots)>0:
#                    pushes = []
#                    for frame in range(start, (start+end)/2 + 1):
#                        dCM_tar = yrp.getCM(motion_ori.getJointVelocitiesGlobal(frame), bodyMasses, None, uppers)
#                        direction = mm.normalize2(mm.projectionOnPlane(dCM_tar, (1,0,0), (0,0,1)))
#                        directionAxis = np.cross((0,1,0), direction)
#                        pushes.append(mm.componentOnVector(mm.logSO3(motion_ori[frame].getJointOrientationFromParentGlobal(swingFoots[0])), directionAxis))
#                    seginfos[i]['max_stf_push_frame'] = pushes.index(max(pushes)) 

        # write .seg
        inputName = os.path.basename(path)
        root = os.path.splitext(inputName)[0]
        outputName = root+'.seg'
        outputFile = open(dir+outputName, 'w')
        cPickle.dump(seginfos, outputFile)
        outputFile.close() 

        print outputName, 'done'
        pprint.pprint(seginfos)
        
    print 'FINISHED'            
