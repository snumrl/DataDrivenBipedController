# +-------------------------------------------------------------------------
# | ysMotionLoader.py
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

import numpy, math

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mm
import Math.csMath as cm
import Motion.ysMotion as ym

ROOT_NAME = "root"
#===============================================================================
# .mm file
#===============================================================================
def readMMFile(mmFilePath):
    names = {}
    names[0] = 'root'
    names[2] = 'lKnee'
    names[4] = 'neck'
    names[6] = 'rKnee'
    names[8] = 'rFoot'
    names[10] = 'rToe'
    names[12] = 'head'
    names[14] = 'rArm'
    names[16] = 'lArm'
    names[18] = 'lHand'
    names[20] = 'rHand'
#    names[14] = 'lArm'
#    names[16] = 'rArm'
#    names[18] = 'rHand'
#    names[20] = 'lHand'
    names[22] = 'lFoot'
    names[24] = 'lToe'
    
    skeleton = ym.MMSkeleton()
    skeleton.addLink("head", "neck")
    skeleton.addLink("neck", "lArm")
    skeleton.addLink("lArm", "lHand")
    skeleton.addLink("neck", "rArm")
    skeleton.addLink("rArm", "rHand")
    skeleton.addLink("neck", "root")
    skeleton.addLink("root", "lKnee")
    skeleton.addLink("lKnee", "lFoot")
    skeleton.addLink("lFoot", "lToe")
    skeleton.addLink("root", "rKnee")
    skeleton.addLink("rKnee", "rFoot")
    skeleton.addLink("rFoot", "rToe")
            
#    # lowest foot height finding code  
#    lowest = 100
#    for mmFilePath in paths:
#        pointMotion = yf.readMMFile(mmFilePath)
#        i = 0
#        for name, point in pointMotion[i].pointMap.items():
#            if name == 'rFoot' or name == 'lFoot' or name == 'rToe' or name == 'lToe':
#                if point[1] < lowest:
#                    print mmFilePath, i
#                    print name
#                    print point[1]
#                    lowest = point[1]
    lowest = .15
    
    f = open(mmFilePath)
    fileLines = f.readlines()
    pointMotion = ym.Motion()
    i = 0
    while i != len(fileLines):
        if fileLines[i].isspace()==False:
            splited = fileLines[i].split()
            pointPosture = ym.MMPosture(skeleton)
            for j in range(0, len(splited), 2):
                point = numpy.array([float(splited[j]), float(splited[j+1]), 0.])
                point[1] -= lowest
                pointPosture.addPoint(names[j], point)
            pointMotion.append(pointPosture)
        i += 1

    f.close()
    pointMotion.fps = 30.
    return pointMotion

#===============================================================================
# .trc file
#===============================================================================
def readTrcFile(trcFilePath, scale = 1.0):
    f = open(trcFilePath)
    fileLines = f.readlines()
    pointMotion = ym.Motion()
    i = 0
    while i != len(fileLines):
        splited = fileLines[i].split()
        boneNames = []
        if i == 2:
            dataRate = float(splited[0])
            numFrames = int(splited[2])
            numMarkers = int(splited[3])
#            print numFrames, numMarkers
        elif i == 3:
            markerNames = [name.split(':')[1] for name in splited[2:]]
            skeleton = ym.PointSkeleton()
            for name in markerNames:
                skeleton.addElement(None, name)
#            print markerNames
        elif i > 5:
            markerPositions = splited[2:]
#            print markerPositions
#            print 'i', i
            pointPosture = ym.PointPosture(skeleton)
            for m in range(numMarkers):
                point = numpy.array([float(markerPositions[m*3]),float(markerPositions[m*3+1]),float(markerPositions[m*3+2])])
                point = numpy.dot(mm.exp(numpy.array([1,0,0]), -math.pi/2.), point)*scale
#                pointPosture.addPoint(markerNames[m], point)
                pointPosture.setPosition(m, point)
#                print 'm', m
#                print markerNames[m], (markerPositions[m*3],markerPositions[m*3+1],markerPositions[m*3+2])
            pointMotion.append(pointPosture)
        i += 1
    f.close()
    pointMotion.fps = dataRate
    return pointMotion

pass
#===============================================================================
# .bvh file
#===============================================================================
def readBvhFile(bvhFilePath, scale=1.0, applyRootOffset=False):
    bvh = Bvh()
    bvh.parseBvhFile(bvhFilePath)
    jointMotion = bvh.toJointMotion(scale, applyRootOffset)
    return jointMotion

def readBvhFileAsBvh(bvhFilePath):
    bvh = Bvh()
    bvh.parseBvhFile(bvhFilePath)
    return bvh

def writeBvhFile(bvhFilePath, jointMotion):
    bvh = Bvh()
    bvh.fromJointMotion(jointMotion)
    bvh.writeBvhFile(bvhFilePath)

class Bvh:
    channelTypes6dof = ['XPOSITION', 'YPOSITION', 'ZPOSITION', 'ZROTATION', 'XROTATION', 'YROTATION']
    channelTypes3dof = ['ZROTATION', 'XROTATION', 'YROTATION']

    class Joint:
        def __init__(self, name):
            self.name = name
            self.offset = None
            self.channels = []
            self.children = []
        def __strHierarchy__(self, depth=0):
            s = ''
            tab1 = '  '*depth
            tab2 = '  '*(depth+1)
            s += '%sJOINT %s\n'%(tab1, self.name)
            s += '%s{\n'%tab1
            s += '%sOFFSET %s\n'%(tab2, self.offset)
            
            channelString = ''
            for channel in self.channels:
                channelString += channel.__str__()+' '
            s += '%sCHANNELS %s\n'%(tab2, channelString)
            
            for child in self.children:
                s += child.__strHierarchy__(depth+1)
            s += '%s}\n'%tab1
            return s

    class Channel:
        def __init__(self, channelType, channelIndex):
            self.channelType = channelType
            self.channelIndex = channelIndex
        def __str__(self):
            return self.channelType
        
    def __init__(self):
        self.joints = []
        self.frameNum = 0
        self.frameTime = 0
        self.motionList = []
        
        self.totalChannelCount = 0

    def __str__(self):
        s = 'HIERARCHY\n'
        s += self.joints[0].__strHierarchy__()
        s += 'MOTION\n'
        s += 'Frame: %d\n'%self.frameNum
        s += 'Frame Time: %f\n'%self.frameTime
#        for i in range(len(self.motionList)):
#            s += self.motionList[i].__str__() + '\n'
        return s

    #===========================================================================
    # read functions
    #===========================================================================
    def parseBvhFile(self, filepath_or_fileobject):
        if isinstance(filepath_or_fileobject, str): file = open(filepath_or_fileobject)
        else:                                       file = filepath_or_fileobject
    
        tokens= file.read().split()
        tokens.reverse()
    
        self.totalChannelCount = 0
        self.parseBvhHierachy(tokens)
        self.parseBvhMotion(tokens)
        
        if not isinstance(filepath_or_fileobject, str): file.close()

    def parseBvhHierachy(self, tokens):
        if tokens.pop().upper() != "HIERARCHY":
            print "HIERARCHY missing"
            return 
        if tokens.pop().upper() != "ROOT":
            print "ROOT missing"
            return
        self.parseBvhJoint(tokens.pop(), tokens)
        
    def parseBvhJoint(self, name, tokens):
        bvhJoint = Bvh.Joint(name)
        self.joints.append(bvhJoint)
        
        if tokens.pop()!="{":
            print "'{' missing"
            return None
        
        endDetected = False
        while not endDetected:
            t = tokens.pop().upper()
            if t == '}':
                endDetected = True
            elif t == 'OFFSET':
                x = float(tokens.pop())
                y = float(tokens.pop())
                z = float(tokens.pop())
                bvhJoint.offset = numpy.array([x,y,z],float)
            elif t == 'CHANNELS':
                channelCount = int(tokens.pop())
                for i in range(channelCount):
                    channelType = tokens.pop().upper()
                    bvhJoint.channels.append(Bvh.Channel(channelType, self.totalChannelCount))
                    self.totalChannelCount += 1
            elif t == 'JOINT':
                bvhJoint.children.append(self.parseBvhJoint(tokens.pop(), tokens))
            elif t == 'END':
                next = tokens.pop().upper()
                if next != 'SITE':
                    print 'END', next, 'is unknown keyword'
                bvhJoint.children.append(self.parseBvhJoint("%s_Effector"%name, tokens))
            else:
                print "invalid bvhJoint definition"
                return None
        return bvhJoint

    def parseBvhMotion(self, tokens):
        if tokens.pop().upper() != 'MOTION':
            print "MOTION missing"
            return None
        if tokens.pop().upper() != 'FRAMES:':
            print "FRAMES: missing"
            return None
        self.frameNum = int(tokens.pop())
        if tokens.pop().upper() != 'FRAME TIME:':
            if tokens.pop().upper() != 'TIME:':
                print "FRAME TIME: missing"
                return None
        self.frameTime = float(tokens.pop())
        
        self.motionList = [None]*self.frameNum
        for i in range(self.frameNum):
            self.motionList[i] = [None]*self.totalChannelCount
            
        for i in range(self.frameNum):
            for j in range(self.totalChannelCount):
                self.motionList[i][j] = float(tokens.pop())

    #===========================================================================
    # write functions
    #===========================================================================
    def writeBvhFile(self, filepath_or_fileobject):
        if isinstance(filepath_or_fileobject, str): file = open(filepath_or_fileobject, 'w')
        else:                                       file = filepath_or_fileobject
        
        self.writeBvhHierarchy(file)
        self.writeBvhMotion(file)

        if not isinstance(filepath_or_fileobject, str): file.close()
    
    def writeBvhHierarchy(self, file):
        file.write('HIERARCHY\n')
        self.writeBvhJoint(file, self.joints[0], 0)
    
    def writeBvhJoint(self, file, joint, depth):
        indent_joint = '  '*depth
        indent_offset = '  '*(depth+1)
        
        if len(joint.children) > 0: endsite = False
        else:                       endsite = True 
        
        # JOINT
        if not endsite:
            if depth==0:    joint_label = 'ROOT'
            else:           joint_label = 'JOINT'
            file.write('%s%s %s\n'%(indent_joint, joint_label, joint.name))
        else:
            file.write('%sEnd Site\n'%indent_joint)
        file.write('%s{\n'%indent_joint)
        
        # OFFSET
        file.write('%sOFFSET %s %s %s\n'%(indent_offset, joint.offset[0], joint.offset[1], joint.offset[2]))
            
        if not endsite:
            # CHANNELS
            channelString = ''
            for channel in joint.channels:
                channelString += channel.__str__()+' '
            file.write('%sCHANNELS %d %s\n'%(indent_offset, len(joint.channels), channelString))
            
            # children
            for child in joint.children:
                self.writeBvhJoint(file, child, depth+1)
        
        # end JOINT
        file.write('%s}\n'%indent_joint)
        
    def writeBvhMotion(self, file):
        file.write('MOTION\n')
        file.write('Frames: %d\n'%self.frameNum)
        file.write('Frame Time: %f\n'%self.frameTime)

        for i in range(self.frameNum):
            for j in range(self.totalChannelCount):
                file.write('%s '%self.motionList[i][j])
            file.write('\n')
                
    #===========================================================================
    # Bvh -> JointMotion
    #===========================================================================
    def toJointMotion(self, scale, applyRootOffset):
        skeleton = self.toJointSkeleton(scale, applyRootOffset)
        
        jointMotion = ym.JointMotion()
        for i in range(len(self.motionList)):
            jointPosture = ym.JointPosture(skeleton)
            self.addJointSO3FromBvhJoint(jointPosture, self.joints[0], self.motionList[i], scale)
            jointPosture.updateGlobalT()
            jointMotion.append(jointPosture)
        
        jointMotion.fps = 1./self.frameTime
        return jointMotion

    def toJointSkeleton(self, scale, applyRootOffset):
        # build joint hierarchy
        jointMap = {}
        root = self.addJointFromBvhJoint(jointMap, self.joints[0].name, self.joints[0], None, scale, applyRootOffset)
        
        # build joint array
        skeleton = ym.JointSkeleton(root)
        for bvhJoint in self.joints:
            skeleton.addElement(jointMap[bvhJoint.name], bvhJoint.name)
        skeleton.rootIndex = skeleton.getElementIndex(root.name)
        
        # initialize
        skeleton.initialize()
        
        return skeleton
    
    def addJointFromBvhJoint(self, jointMap, jointName, bvhJoint, parentJoint, scale, applyOffset):
        joint = ym.Joint(jointName, parentJoint)
        if applyOffset:
            joint.offset = bvhJoint.offset*scale
        jointMap[jointName] = joint
        
        for i in range(len(bvhJoint.children)):
            child = self.addJointFromBvhJoint(jointMap, bvhJoint.children[i].name, bvhJoint.children[i], joint, scale, True)
            joint.children.append(child)
            
        return joint
        
    def addJointSO3FromBvhJoint(self, jointPosture, bvhJoint, channelValues, scale = 1.0):
        localR = mm.I_SO3()
        for channel in bvhJoint.channels:
            if channel.channelType == 'XPOSITION':
                jointPosture.rootPos[0] = channelValues[channel.channelIndex]*scale
            elif channel.channelType == 'YPOSITION':
                jointPosture.rootPos[1] = channelValues[channel.channelIndex]*scale
            elif channel.channelType == 'ZPOSITION':
                jointPosture.rootPos[2] = channelValues[channel.channelIndex]*scale
            elif channel.channelType == 'XROTATION':
                localR = numpy.dot(localR, mm.rotX(mm.RAD * channelValues[channel.channelIndex]))
            elif channel.channelType == 'YROTATION':
                localR = numpy.dot(localR, mm.rotY(mm.RAD * channelValues[channel.channelIndex]))
            elif channel.channelType == 'ZROTATION':
                localR = numpy.dot(localR, mm.rotZ(mm.RAD * channelValues[channel.channelIndex]))
        jointPosture.setLocalR(jointPosture.skeleton.getElementIndex(bvhJoint.name), localR)
            
        for i in range(len(bvhJoint.children)):
            self.addJointSO3FromBvhJoint(jointPosture, bvhJoint.children[i], channelValues)

    #===========================================================================
    # JointMotion -> Bvh
    #===========================================================================
    def fromJointMotion(self, jointMotion):
        skeleton = jointMotion[0].skeleton
        self.fromJointSkeleton(skeleton)

        self.frameNum = len(jointMotion)
        self.motionList = [[None]*self.totalChannelCount for i in range(self.frameNum)]
        for i in range(self.frameNum):
            self._jointValue2channelValues(jointMotion[i], self.motionList[i], skeleton, self.joints[0])
        
        self.frameTime = 1./jointMotion.fps
        
    def fromJointSkeleton(self, jointSkeleton):
        # build bvh joint hierarchy
        bvhJointDict = {}
        self.totalChannelCount = 0
        bvhRoot = self._Joint2BvhJoint(jointSkeleton.getElement(0), bvhJointDict)
        
        # build bvh joint array
        self.joints = [None]*jointSkeleton.getElementNum()
        for i in range(jointSkeleton.getElementNum()):
            self.joints[i] = bvhJointDict[jointSkeleton.getElementName(i)]
            
    def _Joint2BvhJoint(self, joint, bvhJointDict):
        bvhJoint = Bvh.Joint(joint.name)    # name
        bvhJointDict[joint.name] = bvhJoint
        
        bvhJoint.offset = joint.offset  # offset

        # channels
        if joint.parent==None:
            channelTypes = Bvh.channelTypes6dof
        elif len(joint.children)==0:
            channelTypes = []
        else:
            channelTypes = Bvh.channelTypes3dof
        for channelType in channelTypes:
            bvhJoint.channels.append(Bvh.Channel(channelType, self.totalChannelCount))
            self.totalChannelCount += 1
             
        # children
        for child in joint.children:
            bvhJoint.children.append(self._Joint2BvhJoint(child, bvhJointDict))
            
        return bvhJoint
    
    # jointPosture : input
    # channelValues : output
    def _jointValue2channelValues(self, jointPosture, channelValues, jointSkeleton, bvhJoint):
        jointIndex = jointSkeleton.getElementIndex(bvhJoint.name)
        zrot, xrot, yrot = cm.R2zxy_r(jointPosture.getLocalR(jointIndex))

        for channel in bvhJoint.channels:
            if channel.channelType == 'XPOSITION':
                channelValues[channel.channelIndex] = jointPosture.rootPos[0]
            elif channel.channelType == 'YPOSITION':
                channelValues[channel.channelIndex] = jointPosture.rootPos[1]
            elif channel.channelType == 'ZPOSITION':
                channelValues[channel.channelIndex] = jointPosture.rootPos[2]
            elif channel.channelType == 'XROTATION':
                channelValues[channel.channelIndex] = xrot * mm.DEG
            elif channel.channelType == 'YROTATION':
                channelValues[channel.channelIndex] = yrot * mm.DEG
            elif channel.channelType == 'ZROTATION':
                channelValues[channel.channelIndex] = zrot * mm.DEG
        
        for child in bvhJoint.children:
            self._jointValue2channelValues(jointPosture, channelValues, jointSkeleton, child)

        
if __name__ == "__main__":
    import psyco; psyco.full()
    import time
    import cProfile, os
    from datetime import datetime
    from fltk import *
    import GUI.ysSimpleViewer as ysv
    import Renderer.ysRenderer as yr

    def test_readTrcFile():
        trcMotion = readTrcFile('../samples/Day7_Session2_Take01_-_walk.trc', .01)
        print trcMotion[0].skeleton
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('trcMotion', yr.PointMotionRenderer(trcMotion, (0,255,0)))
        viewer.doc.addObject('trcMotion', trcMotion)

        viewer.startTimer(1/trcMotion.fps)
        viewer.show()
        
        Fl.run()
        
    def test_parseBvhFile():
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        bvh = Bvh()
        bvh.parseBvhFile(bvhFilePath)
        print bvh
        
    def test_readBvhFile():
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        motion = readBvhFile(bvhFilePath, .01)
        motion2 = readBvhFile(bvhFilePath, .01, True)
        print motion[0].skeleton

        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,255,0)))
        viewer.doc.addObject('motion', motion)
        viewer.doc.addRenderer('motion2', yr.JointMotionRenderer(motion2, (255,0,0)))
        viewer.doc.addObject('motion2', motion2)

        viewer.startTimer(1/motion.fps)
        viewer.show()
        
        Fl.run()
        
    def test_writeBvhFile():
#        # bvh
#        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
#        bvh = readBvhFileAsBvh(bvhFilePath)
#        
#        tempFilePath = '../samples/bvh_wd2_WalkSameSame00.bvh.temp'        
#        bvh.writeBvhFile(tempFilePath)
        
        # motion
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        motion = readBvhFile(bvhFilePath, .01)
#        motion[0] = motion[0].getTPose()
        
        tempFilePath = '../samples/motion_temp_wd2_WalkSameSame00.bvh.temp'
        writeBvhFile(tempFilePath, motion)
        motion2 = readBvhFile(tempFilePath)
#        motion2[0] = motion2[0].getTPose()
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,255,0)))
        viewer.doc.addObject('motion', motion)
        viewer.doc.addRenderer('motion2', yr.JointMotionRenderer(motion2, (255,0,0)))
        viewer.doc.addObject('motion2', motion2)

        viewer.startTimer(1/motion.fps)
        viewer.show()
        
        Fl.run()
        
    def profile_readBvhFile():
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
    
        profileDataFile = '../samples/cProfile_%s.profile'%datetime.today().strftime('%y%m%d_%H%M%S')
        cProfile.runctx('motion = readBvhFile(bvhFilePath)', globals(), locals(), profileDataFile)
        os.system('python ../../Tools/pprofui.py %s'%profileDataFile)
    
               
    pass
#    test_readTrcFile()
#    test_parseBvhFile()
#    test_readBvhFile()
    test_writeBvhFile()
#    profile_readBvhFile()
