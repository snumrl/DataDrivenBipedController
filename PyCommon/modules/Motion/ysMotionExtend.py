# +-------------------------------------------------------------------------
# | ysMotionExtend.py
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

import copy
import numpy as np

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mm
import Math.ysFunctionGraph as yfg
import Motion.ysMotion as ym
import Motion.ysMotionBlend as ymb
import Motion.ysMotionAnalysis as yma
import Motion.ysBipedAnalysis as yba
import Motion.mmAnalyticIK as aik


def repeatCycle(motion, cycleInterval, repeatNum, transitionLength, transitionFunc=yfg.identity):
    newMotion = motion[:cycleInterval[0]]
    
    repeatedPart = copy.deepcopy(yma.sliceMotion(motion, cycleInterval))
    startRootPos = copy.deepcopy(repeatedPart[0].rootPos)
    repeatedPart.translateByOffset(-startRootPos)
        
    d = repeatedPart[0]-repeatedPart[-1]
#    d.disableTranslation()
    d.rootPos[0]=0.; d.rootPos[2]=0. 
    
    repeatedPart = repeatedPart[:-transitionLength] + ymb.stitchSegment(repeatedPart[-transitionLength:], transitionFunc, d)
    
    for i in range(repeatNum):
        newRepeatedPart = copy.deepcopy(repeatedPart)
        newRepeatedPart.translateByOffset(newMotion[-1].rootPos)
        newMotion += newRepeatedPart[1:]
    
    return newMotion

def extendByIntegration(motion, extendLength, preserveJoints=[], finiteDiff=1):
    lastFrame = len(motion)-1
    p = motion.getJointPositionGlobal(0, lastFrame)
    v = motion.getJointVelocityGlobal(0, lastFrame-finiteDiff, lastFrame)
    a = motion.getJointAccelerationGlobal(0, lastFrame-finiteDiff, lastFrame)
    ap = motion.getJointOrientationsLocal(lastFrame)
    av = motion.getJointAngVelocitiesLocal(lastFrame-finiteDiff, lastFrame)
    aa = motion.getJointAngAccelerationsLocal(lastFrame-finiteDiff, lastFrame)
    t = 1/motion.fps

    # integration
    extended = ym.JointMotion([motion[0].getTPose() for i in range(extendLength)])
    for i in range(extendLength):
        p += v * t
        v += a * t
        ap = map(lambda R0, dR: np.dot(R0, mm.exp(t*dR)), ap, av)
        av = map(lambda V0, dV: V0 + t*dV, av, aa)
        extended[i].rootPos = p.copy()
        extended.setJointOrientationsLocal(i, ap)

    # preserve joint orientations
    preserveJointOrientations = [motion[-1].getJointOrientationGlobal(footJoint) for footJoint in preserveJoints]
    for extendedPosture in extended:
        for i in range(len(preserveJoints)):
            extendedPosture.setJointOrientationGlobal(preserveJoints[i], preserveJointOrientations[i])
        
    return extended

def extendByIntegration_constant(motion, extendLength, preserveJoints=[], finiteDiff=1):
    lastFrame = len(motion)-1
    p = motion.getJointPositionGlobal(0, lastFrame)
    v = motion.getJointVelocityGlobal(0, lastFrame-finiteDiff, lastFrame)
    a = motion.getJointAccelerationGlobal(0, lastFrame-finiteDiff, lastFrame)
    ap = motion.getJointOrientationsLocal(lastFrame)
    av = motion.getJointAngVelocitiesLocal(lastFrame-finiteDiff, lastFrame)
    aa = motion.getJointAngAccelerationsLocal(lastFrame-finiteDiff, lastFrame)
    t = 1/motion.fps

    # integration
    extended = ym.JointMotion([motion[0].getTPose() for i in range(extendLength)])
    for i in range(extendLength):
        p += v * t
#        v += a * t
        ap = map(lambda R0, dR: np.dot(R0, mm.exp(t*dR)), ap, av)
#        av = map(lambda V0, dV: V0 + t*dV, av, aa)
        extended[i].rootPos = p.copy()
        extended.setJointOrientationsLocal(i, ap)

    # preserve joint orientations
    preserveJointOrientations = [motion[-1].getJointOrientationGlobal(footJoint) for footJoint in preserveJoints]
    for extendedPosture in extended:
        for i in range(len(preserveJoints)):
            extendedPosture.setJointOrientationGlobal(preserveJoints[i], preserveJointOrientations[i])
        
    return extended

def extendByIntegrationAttenuation(motion, extendLength, preserveJoints=[], finiteDiff=1, k=1.):
    lastFrame = len(motion)-1
    p = motion.getJointPositionGlobal(0, lastFrame)
    v = motion.getJointVelocityGlobal(0, lastFrame-finiteDiff, lastFrame)
    a = motion.getJointAccelerationGlobal(0, lastFrame-finiteDiff, lastFrame)
    ap = motion.getJointOrientationsLocal(lastFrame)
    av = motion.getJointAngVelocitiesLocal(lastFrame-finiteDiff, lastFrame)
    aa = motion.getJointAngAccelerationsLocal(lastFrame-finiteDiff, lastFrame)
    t = 1/motion.fps

    # integration
    extended = ym.JointMotion([motion[0].getTPose() for i in range(extendLength)])
    for i in range(extendLength):
        p += v * t * k
        v += a * t * k
        ap = map(lambda R0, dR: np.dot(R0, mm.exp(t*k*dR)), ap, av)
        av = map(lambda V0, dV: V0 + t*k*dV, av, aa)
        extended[i].rootPos = p.copy()
        extended.setJointOrientationsLocal(i, ap)

    # preserve joint orientations
    preserveJointOrientations = [motion[-1].getJointOrientationGlobal(footJoint) for footJoint in preserveJoints]
    for extendedPosture in extended:
        for i in range(len(preserveJoints)):
            extendedPosture.setJointOrientationGlobal(preserveJoints[i], preserveJointOrientations[i])
        
    return extended

def extendByIntegrationIK(motion, extendLength, effectorJoint, preserveJoints=[], finiteDiff=1):
    lastFrame = len(motion)-1
    p = motion.getJointPositionGlobal(0, lastFrame)
    v = motion.getJointVelocityGlobal(0, lastFrame-finiteDiff, lastFrame)
    a = motion.getJointAccelerationGlobal(0, lastFrame-finiteDiff, lastFrame)
    ap = motion.getJointOrientationsLocal(lastFrame)
    av = motion.getJointAngVelocitiesLocal(lastFrame-finiteDiff, lastFrame)
    aa = motion.getJointAngAccelerationsLocal(lastFrame-finiteDiff, lastFrame)
    p_effector = motion.getJointPositionGlobal(effectorJoint, lastFrame)
    v_effector = motion.getJointVelocityGlobal(effectorJoint, lastFrame-finiteDiff, lastFrame)
    a_effector = motion.getJointAccelerationGlobal(effectorJoint, lastFrame-finiteDiff, lastFrame)
    t = 1/motion.fps

    # integration
    extended = ym.JointMotion([motion[0].getTPose() for i in range(extendLength)])
    for i in range(extendLength):
        p += v * t
        v += a * t
        ap = map(lambda R0, dR: np.dot(R0, mm.exp(t*dR)), ap, av)
        av = map(lambda V0, dV: V0 + t*dV, av, aa)
        extended[i].rootPos = p.copy()
        extended.setJointOrientationsLocal(i, ap)

    # preserve joint orientations
    preserveJointOrientations = [motion[-1].getJointOrientationGlobal(footJoint) for footJoint in preserveJoints]
    for extendedPosture in extended:
        for i in range(len(preserveJoints)):
            extendedPosture.setJointOrientationGlobal(preserveJoints[i], preserveJointOrientations[i])

    # integration with IK
    for i in range(extendLength):
        p_effector += v_effector * t
        v_effector += a_effector * t
        aik.ik_analytic(extended[i], effectorJoint, p_effector)
        
    return extended

def extendByIntegration_root(motion, extendLength, finiteDiff=1):
    lastFrame = len(motion)-1
    p = motion.getJointPositionGlobal(0, lastFrame)
    v = motion.getJointVelocityGlobal(0, lastFrame-finiteDiff, lastFrame)
    a = motion.getJointAccelerationGlobal(0, lastFrame-finiteDiff, lastFrame)
    t = 1/motion.fps

    # integration
    extended = ym.JointMotion([motion[-1].copy() for i in range(extendLength)])
    for i in range(extendLength):
        p += v * t
        v += a * t
        extended[i].rootPos = p.copy()
        extended[i].updateGlobalT()

    return extended

if __name__=='__main__':
    import psyco; psyco.full()
    from fltk import *
    import Motion.ysMotion as ym
    import Resource.ysMotionLoader as yf
    import Renderer.ysRenderer as yr
    import GUI.ysSimpleViewer as ysv


    def test_repeatCycle():
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        jointMotion = yf.readBvhFile(bvhFilePath, .01)

#        bvhFilePath = '../samples/wd2_n_kick.bvh'
#        jointMotion = yf.readBvhFile(bvhFilePath, .01*2.53)
        
        lFoot = jointMotion[0].skeleton.getElementIndex('LeftFoot')
        rFoot = jointMotion[0].skeleton.getElementIndex('RightFoot')
        
        hRef = .1; vRef = .3
        lc = yma.getElementContactStates(jointMotion, lFoot, hRef, vRef)
        
        interval = yba.getWalkingCycle(jointMotion, lc)
#        interval = [60, 102]
        extendedMotion = repeatCycle(jointMotion, interval, 10, 10)

        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('jointMotion', yr.JointMotionRenderer(jointMotion, (0,0,255), yr.LINK_LINE))
        viewer.doc.addObject('jointMotion', jointMotion)
        viewer.doc.addRenderer('extendedMotion', yr.JointMotionRenderer(extendedMotion, (0,255,0), yr.LINK_LINE))
        viewer.doc.addObject('extendedMotion', extendedMotion)
        
        viewer.startTimer(1/30.)
        viewer.show()
        Fl.run()

    def test_extendByIntegration():
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        motion_ref = yf.readBvhFile(bvhFilePath, .01)
        linkStyle = yr.LINK_LINE

#        bvhFilePath = '../samples/chain_2_rotate_expt_root.bvh'
#        motion_ref = yf.readBvhFile(bvhFilePath, 1.)
#        linkStyle = yr.LINK_WIREBOX
        
        cut = 96
        motion = motion_ref[:cut]
        skeleton = motion[0].skeleton
        print skeleton
        
        lFoot = skeleton.getJointIndex('LeftFoot')
        rFoot = skeleton.getJointIndex('RightFoot')
        lKnee = skeleton.getJointIndex('LeftLeg')
        rKnee = skeleton.getJointIndex('RightLeg')
        lLeg = skeleton.getJointIndex('LeftUpLeg')
        rLeg = skeleton.getJointIndex('RightUpLeg')
        
        motion_extended_by_5_frames = copy.deepcopy(motion)
        motion_extended_by_5_frames.extend(extendByIntegration(motion_extended_by_5_frames, 5))

        motion_extended_by_one_frame = copy.deepcopy(motion)
        for i in range(5):
            motion_extended_by_one_frame.extend(extendByIntegration(motion_extended_by_one_frame, 1))
        
        motion_extended_preserve_foot = copy.deepcopy(motion)
        for i in range(5):
            motion_extended_preserve_foot.extend(extendByIntegration(motion_extended_preserve_foot, 1, (lFoot, rFoot)))

        motion_extended_preserve_leg = copy.deepcopy(motion)
        for i in range(5):
            motion_extended_preserve_leg.extend(extendByIntegration(motion_extended_preserve_leg, 1, (lFoot, rFoot, lKnee, rKnee, lLeg, rLeg)))

        motion_extended_preserve_knee = copy.deepcopy(motion)
        for i in range(5):
            motion_extended_preserve_knee.extend(extendByIntegration(motion_extended_preserve_knee, 1, (lFoot, rFoot)))

        motion_extended_ik = copy.deepcopy(motion)
        for i in range(5):
            motion_extended_ik.extend(extendByIntegrationIK(motion_extended_ik, 1, rFoot, (lFoot, rFoot, lKnee, rKnee, lLeg, rLeg)))

        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('motion_ref', yr.JointMotionRenderer(motion_ref, (100,100,255), linkStyle))
        viewer.doc.addObject('motion_ref', motion_ref)
        viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (255,0,0), linkStyle))
        viewer.doc.addObject('motion', motion)
#        viewer.doc.addRenderer('motion_extended_by_5_frames', yr.JointMotionRenderer(motion_extended_by_5_frames, (0,255,0), linkStyle))
#        viewer.doc.addObject('motion_extended_by_5_frames', motion_extended_by_5_frames)
#        viewer.doc.addRenderer('motion_extended_by_one_frame', yr.JointMotionRenderer(motion_extended_by_one_frame, (255,255,0), linkStyle))
#        viewer.doc.addObject('motion_extended_by_one_frame', motion_extended_by_one_frame)
#        viewer.doc.addRenderer('motion_extended_preserve_foot', yr.JointMotionRenderer(motion_extended_preserve_foot, (255,0,255), linkStyle))
#        viewer.doc.addObject('motion_extended_preserve_foot', motion_extended_preserve_foot)
#        viewer.doc.addRenderer('motion_extended_preserve_leg', yr.JointMotionRenderer(motion_extended_preserve_leg, (0,255,255), linkStyle))
#        viewer.doc.addObject('motion_extended_preserve_leg', motion_extended_preserve_leg)
        viewer.doc.addRenderer('motion_extended_preserve_knee', yr.JointMotionRenderer(motion_extended_preserve_knee, (0,255,255), linkStyle))
        viewer.doc.addObject('motion_extended_preserve_knee', motion_extended_preserve_knee)
        viewer.doc.addRenderer('motion_extended_ik', yr.JointMotionRenderer(motion_extended_ik, (0,255,0), linkStyle))
        viewer.doc.addObject('motion_extended_ik', motion_extended_ik)

        viewer.startTimer(1/30.)
        viewer.show()
        Fl.run()
        
    pass
#    test_repeatCycle()
    test_extendByIntegration()