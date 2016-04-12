# +-------------------------------------------------------------------------
# | mmAnalyticIK.py
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

''' analytic ik module by mmkim'''

import numpy as np

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mm

def ik_analytic(posture, joint_name_or_index, new_position):
    if isinstance(joint_name_or_index, int):
        joint = joint_name_or_index
    else:
        joint = posture.skeleton.getJointIndex(joint_name_or_index)
    
#    joint_parent = posture.body.joint_parent[joint]
#    joint_parent_parent = posture.body.joint_parent[joint_parent]
    joint_parent = posture.skeleton.getParentJointIndex(joint)
    joint_parent_parent = posture.skeleton.getParentJointIndex(joint_parent)

#    B = posture.get_position(joint)
#    C = posture.get_position(joint_parent)
#    A = posture.get_position(joint_parent_parent)
    B = posture.getJointPositionGlobal(joint)
    C = posture.getJointPositionGlobal(joint_parent)
    A = posture.getJointPositionGlobal(joint_parent_parent)

    L = B - A
    N = B - C
    M = C - A

#    l = mathlib.length(L);
#    n = mathlib.length(N);
#    m = mathlib.length(M);
    l = mm.length(L);
    n = mm.length(N);
    m = mm.length(M);

#    a = mathlib.ACOS((l*l + n*n - m*m) / (2*l*n))
#    b = mathlib.ACOS((l*l + m*m - n*n) / (2*l*m))
    a = mm.ACOS((l*l + n*n - m*m) / (2*l*n))
    b = mm.ACOS((l*l + m*m - n*n) / (2*l*m))

    B_new = new_position;
    L_new = B_new - A;

#    l_ = mathlib.length(L_new)
    l_ = mm.length(L_new)

#    a_ = mathlib.ACOS((l_*l_ + n*n - m*m) / (2*l_*n))
#    b_ = mathlib.ACOS((l_*l_ + m*m - n*n) / (2*l_*m))
    a_ = mm.ACOS((l_*l_ + n*n - m*m) / (2*l_*n))
    b_ = mm.ACOS((l_*l_ + m*m - n*n) / (2*l_*m))

    # rotate joint in plane 
#    rotV = mathlib.normalize(numpy.cross(M, L))
    rotV = mm.normalize2(np.cross(M, L))
    rotb = b - b_;
    rota = a_ - a - rotb;
#    posture.rotate_global_orientation(joint_parent_parent, mathlib.exp(rotV, rotb))
#    posture.rotate_global_orientation(joint_parent, mathlib.exp(rotV * rota))
    posture.mulJointOrientationGlobal(joint_parent_parent, mm.exp(rotV, rotb))
    posture.mulJointOrientationGlobal(joint_parent, mm.exp(rotV * rota))

    # rotate plane
#    rotV2 = mathlib.normalize(numpy.cross(L, L_new))
#    l_new = mathlib.length(L_new)
#    l_diff = mathlib.length(L_new - L)
#    rot2 = mathlib.ACOS((l_new * l_new + l * l - l_diff * l_diff) / (2 * l_new * l))
#    posture.rotate_global_orientation(joint_parent_parent, mathlib.exp(rotV2, rot2))
    rotV2 = mm.normalize2(np.cross(L, L_new))
    l_new = mm.length(L_new)
    l_diff = mm.length(L_new - L)
    rot2 = mm.ACOS((l_new * l_new + l * l - l_diff * l_diff) / (2 * l_new * l))
    posture.mulJointOrientationGlobal(joint_parent_parent, mm.exp(rotV2, rot2))

    return posture

#def ik_hand(posture, joint, new_hand_position, thorax_joint = 'thorax'):
#    # rotate thorax
#    pos_thorax = posture.get_position(thorax_joint)
#    pos_humerus = posture.get_position(posture.body.joint_parent[posture.body.joint_parent[joint]])
#    thorax_to_humerus = pos_humerus - pos_thorax
#    new_thorax_to_humerus =  new_hand_position - pos_thorax
#
#    arm_length = mathlib.length(posture.body.joint_offset[joint]) + mathlib.length(posture.body.joint_offset[posture.body.joint_parent[joint]])
#    cur_length = mathlib.length(pos_humerus - new_hand_position)
#    rot_ratio = cur_length / (arm_length + 0.3) - 0.3
#    if rot_ratio > 1.0:
#        rot_ratio = 1.0
#    elif rot_ratio < 0.0:
#        rot_ratio = 0.0
#
#    rotv = mathlib.get_rot_vector(thorax_to_humerus, new_thorax_to_humerus)
#    posture.rotate_global_orientation(thorax_joint, mathlib.exp(rotv * rot_ratio))
#
#    # hand
#    ik_analytic(posture, joint, new_hand_position)
#
#    return posture

#def retarget_single(motion, diff_frame, diff_posture, width = 7):
#    for i in range(diff_frame-width, diff_frame+width+1):
#        x  = (i - diff_frame) / float(width+1)
#        ratio = mathlib.blend_weight(x)
#
#        p = motion[i]
#        for joint in p.orientations:
#            p.orientations[joint] = numpy.dot(p.orientations[joint], mathlib.exp(ratio * diff_posture[joint]))

if __name__=='__main__':
    import psyco; psyco.full()
    from fltk import *
    import copy
    import Resource.ysMotionLoader as yf
    import GUI.ysSimpleViewer as ysv
    import Renderer.ysRenderer as yr
    
    def test_ik_analytic():
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        jointMotion = yf.readBvhFile(bvhFilePath, .01)

        ik_target = [(0, .3, .3)]
        
        jointMotion2 = copy.deepcopy(jointMotion)
        for i in range(len(jointMotion2)):
            ik_analytic(jointMotion2[i], 'LeftFoot', ik_target[0])
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('jointMotion', yr.JointMotionRenderer(jointMotion, (0,150,255)))
        viewer.doc.addObject('jointMotion', jointMotion)
        viewer.doc.addRenderer('jointMotion2', yr.JointMotionRenderer(jointMotion2, (0,255,0)))
        viewer.doc.addObject('jointMotion2', jointMotion2)
        viewer.doc.addRenderer('ik_target', yr.PointsRenderer(ik_target, (255,0,0)))

        viewer.startTimer(1/30.)
        viewer.show()
        Fl.run()
        
        pass        

    test_ik_analytic()