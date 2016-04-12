# +-------------------------------------------------------------------------
# | ysMotionConverter.py
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

import numpy

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mmMath
import Motion.ysMotion as ym

#===============================================================================
# point2Joint
#===============================================================================
def mm2Joint_Motion(pointMotion, rootName):
    jointMotion = ym.Motion()
    jointSkeleton = mm2Joint_Skeleton(pointMotion[0], rootName)
    for pointPosture in pointMotion:
        jointMotion.append(mm2Joint_Posture(pointPosture, pointMotion[0], jointSkeleton))
    jointMotion.fps = pointMotion.fps
    return jointMotion

def mm2Joint_Skeleton(pointTPosture, rootName):
    visited = set()
    root = ym.Joint(rootName, None)
    _addChildJointOf(root, pointTPosture, visited)
    jointSkeleton = ym.JointSkeleton(root)
    return jointSkeleton

def mm2Joint_Posture(pointPosture, pointTPose, jointSkeleton):
    jointPosture = ym.JointPosture(jointSkeleton)
    jointPosture.rootPos = pointPosture.pointMap[jointSkeleton.root.name]

    extendedPoints = {}
    for jointName, joint in jointSkeleton.joints.items():
        if joint.parent==None:
            extendedPoints['parent_'+jointName] = pointTPose.pointMap[jointName] + numpy.array([1.,0.,0.])
            extendedPoints[jointName] = pointTPose.pointMap[jointName]
        elif len(joint.parent.children)>1:
            extendedPoints[jointName] = pointTPose.pointMap[joint.parent.name]
        else:
            extendedPoints[jointName] = pointTPose.pointMap[jointName]
            
    initRot = {}
    for jointName, joint in jointSkeleton.joints.items():
        if len(joint.children)!=1:
            curRot = mmMath.I_SO3()
        else:
            p = extendedPoints[_getValidParentName(joint)] - extendedPoints[joint.name]
            c = extendedPoints[joint.children[0].name] - extendedPoints[joint.name]
            curRot = mmMath.getSO3FromVectors(p, c)
        initRot[joint.name] = curRot
    
    extendedPoints2 = {}
    for jointName, joint in jointSkeleton.joints.items():
        if joint.parent==None:
            extendedPoints2['parent_'+jointName] = pointPosture.pointMap[jointName] + numpy.array([1.,0.,0.])
            extendedPoints2[jointName] = pointPosture.pointMap[jointName]
        elif len(joint.parent.children)>1:
            extendedPoints2[jointName] = pointPosture.pointMap[joint.parent.name]
        else:
            extendedPoints2[jointName] = pointPosture.pointMap[jointName]
            
    _calcJointLocalR(jointSkeleton.root, extendedPoints2, initRot, jointPosture)
    
    return jointPosture

def _calcJointLocalR(joint, extendedPoints, initRot, jointPosture):
    if len(joint.children)!=1:
        curRot = mmMath.I_SO3()
    else:
        p = extendedPoints[_getValidParentName(joint)] - extendedPoints[joint.name]
        c = extendedPoints[joint.children[0].name] - extendedPoints[joint.name]
        curRot = mmMath.getSO3FromVectors(p, c)
        
    jointPosture.setLocalR(joint.name, numpy.dot(curRot, initRot[joint.name].transpose()))
        
    for childJoint in joint.children:
        _calcJointLocalR(childJoint, extendedPoints, initRot, jointPosture)

def _getValidParentName(joint):
    if joint.parent==None:
        return 'parent_'+joint.name
    elif len(joint.parent.children)>1:
        return _getValidParentName(joint.parent)
    else:
        return joint.parent.name

def _addChildJointOf(joint, pointTPosture, visited):
    if joint.name in visited:
        return
    else:
        visited.add(joint.name)
        
    childNames = []
    for link in pointTPosture.skeleton.links:
        if link[0] == joint.name or link[1] == joint.name:
            if link[0] == joint.name:
                childName = link[1]
            elif link[1] == joint.name:
                childName = link[0]
            
            if childName in visited:
                continue
            
            childNames.append(childName)
            
    for childName in childNames:
        if len(childNames) > 1:
            interJoint = joint.addChild(joint.name+'_'+childName)
            interJoint.offset = numpy.array([0.,0.,0.])
            
            childJoint = interJoint.addChild(childName)
            childJoint.offset = pointTPosture.pointMap[childName] - pointTPosture.pointMap[joint.name] 
        else:
            childJoint = joint.addChild(childName)
            childJoint.offset = pointTPosture.pointMap[childName] - pointTPosture.pointMap[joint.name] 
        
        _addChildJointOf(childJoint, pointTPosture, visited)
         
        
#===============================================================================
# joint2Point            
#===============================================================================
def joint2mm_Motion(jointMotion):
    pointMotion = ym.Motion()
    pointSkeleton = joint2mm_Skeleton(jointMotion[0].skeleton)
    for jointPosture in jointMotion:
        pointMotion.append(joint2mm_Posture(jointPosture, pointSkeleton))
    pointMotion.fps = jointMotion.fps
    return pointMotion

def joint2mm_Skeleton(jointSkeleton):
    pointSkeleton = ym.PointSkeleton()
    for name, joint in jointSkeleton.joints.items():
        for childJoint in joint.children:
            if joint.parent!=None and len(joint.parent.children)>1:
#                print 'a', joint.name, childJoint.name, '-', joint.parent.name, childJoint.name
                pointSkeleton.addLink(joint.parent.name, childJoint.name)
            else:
                if len(joint.children)>1:
                    pass
#                    print 'b', joint.name, childJoint.name, '-', joint.name, childJoint.children[0].name
#                    pointSkeleton.addLink(joint.name, childJoint.children[0].name)
                else:
#                    print 'c', joint.name, childJoint.name, '-', joint.name, childJoint.name
                    pointSkeleton.addLink(joint.name, childJoint.name)
    return pointSkeleton

def joint2mm_Posture(jointPosture, pointSkeleton):
    pointPosture = ym.PointPosture(pointSkeleton)
    for joint in jointPosture.skeleton.joints.values():
        if not (joint.parent!=None and len(joint.parent.children)>1):
            pointPosture.addPoint(joint.name, jointPosture.getGlobalPos(joint.name))
    return pointPosture



if __name__ == '__main__':
    import copy, numpy, math
    from fltk import *

    import Resource.ysMotionLoader as yf
    import GUI.ysViewer as yv
    import Renderer.ysRenderer as yr
    
    def test_point2Joint_Motion():
        mmFilePath = '../samples/physics2_WalkSameSame01.mm'
        pointMotion = yf.readMMFile(mmFilePath)
        frameTime = 1./30.

        pointMotion[0].pointMap['lFoot'][1] -= 0.1
        
#        pointMotion = pointMotion[0:1]
        
        jointMotion = point2Joint_Motion(pointMotion, 'root')
        pointMotion2 = joint2Point_Motion(jointMotion)
        
        print 'pointMotion.skeleton', pointMotion[0].skeleton.links
        print pointMotion[0].pointMap.keys()
        print 'jointMotion.skeleton', jointMotion[0].skeleton.joints.keys()
        print 'pointMotion2.skeleton', pointMotion2[0].skeleton.links
        print pointMotion2[0].pointMap.keys()
    
        motionSystem = ym.MotionSystem()
        motionSystem.addMotion(pointMotion)
        motionSystem.addMotion(jointMotion)
        
        renderers = []
#        renderers.append(yr.PointMotionRenderer(pointMotion))
        renderers.append(yr.PointMotionRenderer(pointMotion2))
        renderers.append(yr.JointMotionRenderer(jointMotion, (0,255,0), yr.LINK_LINE))
    
        viewer = yv.Viewer2(100, 100, 800, 650, None, motionSystem, renderers)
        viewer.startTimer(frameTime)
        viewer.show()
        Fl.run()
        
    def test_joint2Point_Motion():
        bvhFilePath = '../samples/physics2_WalkSameSame00.bvh'
        jointMotion, frameTime = yf.readBvhFileAsJointMotion(bvhFilePath, .1)
        
        pointMotion = joint2Point_Motion(jointMotion)
        
        print 'jointMotion.skeleton', jointMotion[0].skeleton
        print 'pointMotion.skeleton', pointMotion[0].skeleton
        print 'jointMotion.skeleton - pointMotion.skeleton', set(jointMotion[0].skeleton.joints.keys()) - pointMotion[0].skeleton.pointSet 
        print 'pointMotion.skeleton - jointMotion.skeleton', pointMotion[0].skeleton.pointSet - set(jointMotion[0].skeleton.joints.keys())
    
        motionSystem = ym.MotionSystem()
        motionSystem.addMotion(pointMotion)
        motionSystem.addMotion(jointMotion)
        
        renderers = []
        renderers.append(yr.PointMotionRenderer(pointMotion))
        renderers.append(yr.JointMotionRenderer(jointMotion, (0,255,0), yr.LINK_LINE))
    
        viewer = yv.Viewer2(100, 100, 800, 650, None, motionSystem, renderers)
        viewer.startTimer(frameTime)
        viewer.show()
        Fl.run()
        
    test_point2Joint_Motion()
#    test_joint2Point_Motion()