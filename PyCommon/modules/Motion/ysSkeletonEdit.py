# +-------------------------------------------------------------------------
# | ysSkeletonEdit.py
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
import math

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mm

def removeJoint(motion, joint_name_or_index, update=True):
    if isinstance(joint_name_or_index, int):
        removeIndex = joint_name_or_index
    else:
        removeIndex = motion[0].skeleton.getElementIndex(joint_name_or_index)

    skeleton = motion[0].skeleton
    children = skeleton.getChildIndexes(removeIndex)
    
    for posture in motion:
        for child in children:
            posture.setLocalR(child, np.dot(posture.getLocalR(removeIndex), posture.getLocalR(child)))

    skeleton.removeElement(removeIndex)
    for posture in motion:
        del posture.localRs[removeIndex:removeIndex+1]
        del posture.globalTs[removeIndex:removeIndex+1]
        if update:
            for child in children:
                posture.updateGlobalT(child)
            
def offsetJointLocal(motion, joint_name_or_index, offset, update=True):
    if isinstance(joint_name_or_index, int):
        index = joint_name_or_index
    else:
        index = motion[0].skeleton.getElementIndex(joint_name_or_index)
    
    skeleton = motion[0].skeleton
    skeleton.getElement(index).offset += offset
    
    if update:
        for posture in motion:
            posture.updateGlobalT(index)
        
def rotateJointLocal(motion, joint_name_or_index, R, update=True):
    if isinstance(joint_name_or_index, int):
        index = joint_name_or_index
    else:
        index = motion[0].skeleton.getElementIndex(joint_name_or_index)
        
    for posture in motion:
        posture.mulLocalR(index, R)
        if update:
            posture.updateGlobalT(index)
            
def updateGlobalT(motion):
    for posture in motion:
        posture.updateGlobalT()


if __name__ == '__main__':
    import psyco; psyco.full()
    from fltk import *
    import copy
    import Resource.ysMotionLoader as yf
    import GUI.ysSimpleViewer as ysv
    import Renderer.ysRenderer as yr
    import Motion.ysMotion as ym
    
    def test_removeJoint():
        # skeleton 1
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        motion = yf.readBvhFile(bvhFilePath, .01)
        print motion[0].skeleton
        
        editedMotion = copy.deepcopy(motion)
        removeJoint(editedMotion, 'Head')
        removeJoint(editedMotion, 'RightShoulder')
        removeJoint(editedMotion, 'LeftShoulder1')
        removeJoint(editedMotion, 'RightToes_Effector')
        removeJoint(editedMotion, 'LeftToes_Effector')
        removeJoint(editedMotion, 'RightHand_Effector')
        removeJoint(editedMotion, 'LeftHand_Effector')
        offsetJointLocal(editedMotion, 'RightArm', (.03,-.05,0))
        offsetJointLocal(editedMotion, 'LeftArm', (-.03,-.05,0))
        print editedMotion[0].skeleton
        
        editedTPose = editedMotion[0].getTPose()
        editedTPose.translateByTarget((0,0,0))
        
        # skeleton 2
        bvhFilePath = '../samples/wd2_left_turn.bvh'
        motion = yf.readBvhFile(bvhFilePath, .01 * 2.53999905501)
        print motion[0].skeleton
        
        editedMotion2 = copy.deepcopy(motion)
        removeJoint(editedMotion2, 'RightUpLegDummy')
        removeJoint(editedMotion2, 'SpineDummy')
        removeJoint(editedMotion2, 'HEadDummy')
        removeJoint(editedMotion2, 'LeftShoulder1Dummy')
        removeJoint(editedMotion2, 'RightShoulderDummy')
        removeJoint(editedMotion2, 'LeftUpLegDummy')
        removeJoint(editedMotion2, 'Head')
        removeJoint(editedMotion2, 'RightShoulder')
        removeJoint(editedMotion2, 'LeftShoulder1')
        removeJoint(editedMotion2, 'RightToes_Effector')
        removeJoint(editedMotion2, 'LeftToes_Effector')
        removeJoint(editedMotion2, 'RightHand_Effector')
        removeJoint(editedMotion2, 'LeftHand_Effector')
        offsetJointLocal(editedMotion2, 'RightArm', (.03,-.05,0))
        offsetJointLocal(editedMotion2, 'LeftArm', (-.03,-.05,0))
        print editedMotion2[0].skeleton
        
        editedTPose2 = editedMotion2[0].getTPose()
        editedTPose2.translateByTarget((0,0,0))
                
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_LINE))
        viewer.doc.addObject('motion', motion)
        viewer.doc.addRenderer('editedMotion', yr.JointMotionRenderer(editedMotion, (0,255,0), yr.LINK_LINE))
        viewer.doc.addObject('editedMotion', editedMotion)
        viewer.doc.addRenderer('editedTPose', yr.JointMotionRenderer(ym.JointMotion([editedTPose]), (255,0,0), yr.LINK_LINE))
        viewer.doc.addObject('editedTPose', editedTPose)
        viewer.doc.addRenderer('editedTPose2', yr.JointMotionRenderer(ym.JointMotion([editedTPose2]), (255,255,0), yr.LINK_LINE))
        viewer.doc.addObject('editedTPose2', editedTPose2)

        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()
        
    def test_rotateJoint_amc():
        #=======================================================================
        # bvh
        #=======================================================================
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        bvh_motion = yf.readBvhFile(bvhFilePath, .01)
        
        removeJoint(bvh_motion, 'Head')
        removeJoint(bvh_motion, 'RightShoulder')
        removeJoint(bvh_motion, 'LeftShoulder1')
        removeJoint(bvh_motion, 'RightToes_Effector')
        removeJoint(bvh_motion, 'LeftToes_Effector')
        removeJoint(bvh_motion, 'RightHand_Effector')
        removeJoint(bvh_motion, 'LeftHand_Effector')
        offsetJointLocal(bvh_motion, 'RightArm', (.03,-.05,0))
        offsetJointLocal(bvh_motion, 'LeftArm', (-.03,-.05,0))
        
        bvh_tpose = bvh_motion[0].getTPose()
        bvh_tpose.translateByTarget((0,0,0))
        print bvh_tpose.skeleton
        
        #=======================================================================
        # from amc
        #=======================================================================
        bvhFilePath = '../samples/wd2_left_turn.bvh'
        amc_motion = yf.readBvhFile(bvhFilePath, .01 * 2.53999905501)
        
        removeJoint(amc_motion, 'RightUpLegDummy')
        removeJoint(amc_motion, 'SpineDummy')
        removeJoint(amc_motion, 'HEadDummy')
        removeJoint(amc_motion, 'LeftShoulder1Dummy')
        removeJoint(amc_motion, 'RightShoulderDummy')
        removeJoint(amc_motion, 'LeftUpLegDummy')
        removeJoint(amc_motion, 'Head')
        removeJoint(amc_motion, 'RightShoulder')
        removeJoint(amc_motion, 'LeftShoulder1')
        removeJoint(amc_motion, 'RightToes_Effector')
        removeJoint(amc_motion, 'LeftToes_Effector')
        removeJoint(amc_motion, 'RightHand_Effector')
        removeJoint(amc_motion, 'LeftHand_Effector')
        offsetJointLocal(amc_motion, 'RightArm', (.03,-.05,0))
        offsetJointLocal(amc_motion, 'LeftArm', (-.03,-.05,0))
        
        amc_tpose = amc_motion[0].getTPose()
        amc_tpose.translateByTarget((0,0,0))
        print amc_tpose.skeleton
        
        # edited amc
        amc_motion2 = copy.deepcopy(amc_motion)
        
        amc_tpose2 = amc_motion2[0].getTPose() 
        amc_tpose2.translateByTarget((0,0,0))
        
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('bvh_tpose', yr.JointMotionRenderer(ym.JointMotion([bvh_tpose]), (255,0,0), yr.LINK_LINE))
        viewer.doc.addObject('bvh_tpose', bvh_tpose)
        viewer.doc.addRenderer('amc_tpose', yr.JointMotionRenderer(ym.JointMotion([amc_tpose]), (255,255,0), yr.LINK_LINE))
        viewer.doc.addObject('amc_tpose', amc_tpose)
        viewer.doc.addRenderer('amc_tpose2', yr.JointMotionRenderer(ym.JointMotion([amc_tpose2]), (0,255,255), yr.LINK_LINE))
        viewer.doc.addObject('amc_tpose2', amc_tpose2)
        
        viewer.doc.addRenderer('amc_motion', yr.JointMotionRenderer(amc_motion, (0,100,255), yr.LINK_LINE))
        viewer.doc.addObject('amc_motion', amc_motion)
        viewer.doc.addRenderer('amc_motion2', yr.JointMotionRenderer(amc_motion2, (0,255,0), yr.LINK_LINE))
        viewer.doc.addObject('amc_motion2', amc_motion2)

        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()
        
    def test_offsetJointLocal():
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        motion = yf.readBvhFile(bvhFilePath, .01)
        print motion[0].skeleton
        
        shortLegs = copy.deepcopy(motion)
        offsetJointLocal(shortLegs, 'LeftLeg', (0, .2, 0))
        offsetJointLocal(shortLegs, 'RightLeg', (0, .2, 0))
        offsetJointLocal(shortLegs, 'LeftFoot', (0, .2, 0))
        offsetJointLocal(shortLegs, 'RightFoot', (0, .2, 0))
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_LINE))
        viewer.doc.addObject('motion', motion)
        viewer.doc.addRenderer('shortLegs', yr.JointMotionRenderer(shortLegs, (0,255,0), yr.LINK_LINE))
        viewer.doc.addObject('shortLegs', shortLegs)

        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()
                
    pass
#    test_removeJoint()
#    test_rotateJoint_amc()
    test_offsetJointLocal()