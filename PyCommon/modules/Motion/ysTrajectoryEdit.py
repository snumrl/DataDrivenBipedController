# +-------------------------------------------------------------------------
# | ysTrajectoryEdit.py
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

import numpy as np

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mm
import Math.ysFunctionGraph as yfg
import Motion.mmAnalyticIK as aik
import Motion.ysMotionBlend as ymb

# keep position of element at startFrame during motion[startFrame] ~ motion[endFrame]
def fixPosition(motion, element_name_or_index, startFrame, endFrame, update=True):
    if isinstance(element_name_or_index, int): index = element_name_or_index
    else: index = motion[0].skeleton.getElementIndex(element_name_or_index)
    
    fixPos = motion.getPosition(index, startFrame)
    oriEndRootPos = motion[endFrame].rootPos.copy()
    
    # keep during motion[startFrame] ~ motion[endFrame]
    for i in range(startFrame+1, endFrame+1):
        motion[i].setPosition(index, fixPos)
        
    # align motion[endFrame+1:]
    offset = motion[endFrame].rootPos - oriEndRootPos
    for i in range(endFrame+1, len(motion)):
        motion[i].rootPos += offset

    if update:
        children = motion[0].skeleton.getChildIndexes(index)
        for i in range(startFrame+1, len(motion)):
            for child in children:
                motion[i].updateGlobalT(child)
    
def setPositionTarget(motion, joint_name_or_index, targetPosition, applyInterval=None, transitionLength=10, transitionFunc=yfg.identity):
    if isinstance(joint_name_or_index, int): jointIndex = joint_name_or_index
    else: jointIndex = motion[0].skeleton.getJointIndex(joint_name_or_index)
    
    if applyInterval==None:
        applyInterval = [0, len(motion)-1]
        
    m1 = motion[applyInterval[0]:applyInterval[1]+1].copy()
    for posture in m1:
        aik.ik_analytic(posture, jointIndex, targetPosition)
    
    motion[:] = ymb.stitchedReplace(motion, m1, applyInterval[0], transitionLength, transitionFunc)

def setPositionTarget2(motion, joint_name_or_index, targetPosition, applyInterval=None, preserveJoint=None, transitionLength=10, transitionFunc=yfg.identity):
    if isinstance(joint_name_or_index, int): jointIndex = joint_name_or_index
    else: jointIndex = motion[0].skeleton.getJointIndex(joint_name_or_index)
    
    if applyInterval==None:
        applyInterval = [0, len(motion)-1]
        
    m1 = motion[applyInterval[0]:applyInterval[1]+1].copy()
    for posture in m1:
        R_ori = posture.getJointOrientationGlobal(preserveJoint)
        aik.ik_analytic(posture, jointIndex, targetPosition)
        posture.setJointOrientationGlobal(preserveJoint, R_ori)
    
    motion[:] = ymb.stitchedReplace(motion, m1, applyInterval[0], transitionLength, transitionFunc)

def updateGlobalT(motion):
    for posture in motion:
        posture.updateGlobalT()
            
if __name__=='__main__':
    import psyco; psyco.full()
    from fltk import *
    import numpy
    import operator as op
    import Motion.ysMotion as ym
    import Resource.ysMotionLoader as yf
    import Motion.ysMotionConverter as ymc
    import Motion.ysMotionExtend as ymt
    import Renderer.ysRenderer as yr
    import GUI.ysSimpleViewer as ysv
    import Motion.ysMotionAnalysis as yma
    import Motion.ysBipedAnalysis as yba
    import Math.ysFunctionGraph as yfg
    import Motion.ysMotionBlend as ymb
    from pylab import *
    import copy
            
    def test_stitch_fix_foot():
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        motion = yf.readBvhFile(bvhFilePath, .01)
        print motion[0].skeleton
        print

        hRef = .1; vRef = .3
        lfoot = motion[0].skeleton.getElementIndex('LeftFoot')
        rfoot = motion[0].skeleton.getElementIndex('RightFoot')
        rtoe = motion[0].skeleton.getElementIndex('RightToes_Effector')
        lc = yma.getElementContactStates(motion, lfoot, hRef, vRef)
        rc = yma.getElementContactStates(motion, rfoot, hRef, vRef)
        
        intervals, states = yba.getBipedGaitIntervals(lc, rc, 10, .1)
        segments = yma.splitMotionIntoSegments(motion, intervals)
        print 'wd2_WalkSameSame00'
        print intervals
        print [yba.GaitState.text[state] for state in states]
        print

        shorten = 5
        tlen = 10
        transitionFunc = yfg.identity
        
        motion_stitch = copy.copy(segments[0])
        
        # just add
        motion_stitch.extend(segments[1][1:])
        
        # add shorter segment
        motion_stitch.extend(segments[2][1:-shorten])
        
        # stitch
        motion_stitch_foot = copy.deepcopy(motion_stitch)
        
        print 'transition interval: %d~%d'%(len(motion_stitch), len(motion_stitch)+tlen)
        
        motion_stitch.extend(ymb.getStitchedNextMotion(segments[3], motion_stitch[-1], tlen, transitionFunc))
        
        startFrame = len(motion_stitch_foot)-1
        motion_stitch_foot.extend(ymb.getStitchedNextMotion(segments[3], motion_stitch_foot[-1], tlen, transitionFunc))
        fixPosition(motion_stitch_foot, rtoe, startFrame, startFrame+4)
        
        # just add remain parts
        for i in range(4, len(segments)):
            motion_stitch.extend(ymb.getStitchedNextMotion(segments[i], motion_stitch[-1], tlen, transitionFunc))
            motion_stitch_foot.extend(ymb.getStitchedNextMotion(segments[i], motion_stitch_foot[-1], tlen, transitionFunc))
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_LINE))
        viewer.doc.addObject('motion', motion)
        viewer.doc.addRenderer('motion_stitch', yr.JointMotionRenderer(motion_stitch, (0,255,0), yr.LINK_LINE))
        viewer.doc.addObject('motion_stitch', motion_stitch)
        viewer.doc.addRenderer('motion_stitch_foot', yr.JointMotionRenderer(motion_stitch_foot, (255,255,0), yr.LINK_LINE))
        viewer.doc.addObject('motion_stitch_foot', motion_stitch_foot)
        
        
        viewer.startTimer(1/30.)
        viewer.show()
        Fl.run()
        
    def test_setPositionTarget():
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        motion = yf.readBvhFile(bvhFilePath, .01)
        print motion[0].skeleton
        
        lFoot = motion[0].skeleton.getJointIndex('LeftFoot')
        oripos = motion[0].getJointPositionGlobal(lFoot)
        newpos = oripos + mm.v3(0,1,0)

        motion_edit1 = copy.deepcopy(motion)
        setPositionTarget(motion_edit1, lFoot, newpos)
        
        motion_edit2 = copy.deepcopy(motion)
        setPositionTarget(motion_edit2, lFoot, newpos, [10,20], 5, yfg.identity)
        print len(motion), len(motion_edit2)
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_LINE))
        viewer.doc.addObject('motion', motion)
        viewer.doc.addRenderer('motion_edit1', yr.JointMotionRenderer(motion_edit1, (0,255,0), yr.LINK_LINE))
        viewer.doc.addObject('motion_edit1', motion_edit1)
        viewer.doc.addRenderer('motion_edit2', yr.JointMotionRenderer(motion_edit2, (255,255,0), yr.LINK_LINE))
        viewer.doc.addObject('motion_edit2', motion_edit2)
        
        
        viewer.startTimer(1/30.)
        viewer.show()
        Fl.run()

    pass
#    test_stitch_fix_foot()
    test_setPositionTarget()