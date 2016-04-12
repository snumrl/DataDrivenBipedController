# +-------------------------------------------------------------------------
# | ysMotionAnalysis.py
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

import math

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mmMath
import Util.ysPythonEx as ype

class FootState:
    STANCE = 0
    TAKING = 1
    SWING = 3
    LANDING = 2
    text = ype.getReverseDict(locals())
    
class FootPeakType:
    STANCE_MID = 0
    TAKING_PULL = 1
    TAKING_PUSH = 2
    LANDING_PUSH = 3
    START_END = 4
    text = ype.getReverseDict(locals())

pass
#===============================================================================
# states generation 
#===============================================================================
def getElementContactStates(motion, element_name_or_index, hRef, vRef):
    if isinstance(element_name_or_index, int): index = element_name_or_index
    else: index = motion[0].skeleton.getElementIndex(element_name_or_index)
    
    contactStates = [None]*len(motion)
    for i in range(len(motion)):
        pos = motion.getPosition(index, i)
        vel = motion.getVelocity(index, i)
        if pos[1] < hRef and mmMath.length(vel) < vRef:
            contactStates[i] = True
        else:
            contactStates[i] = False
            
    return contactStates

def getMotionContactStates(motion, hRef, vRef):
    map = {}
    for index in range(motion[0].skeleton.getElementNum()):
        contactStates = getElementContactStates(motion, index, hRef, vRef)
        for frame in range(len(contactStates)):
            if contactStates[frame]:
                map[index] = contactStates
                break
    return map

def getTakingLandingFrames(contactStates):
    takingFrames = []
    landingFrames = []
    
    for i in range(1, len(contactStates)):
        if contactStates[i]!=contactStates[i-1]:
            if contactStates[i]==True:
                landingFrames.append(i)
            else:
                takingFrames.append(i)
            
    return takingFrames, landingFrames

pass
#===============================================================================
# foot states 
#===============================================================================
def getFootStates(heelContactStates, toeContactState):
    prevIntervalState = None
    footStates = [None]*len(heelContactStates)
    for i in range(len(heelContactStates)):
        if heelContactStates[i] and toeContactState[i]:
            footStates[i] = FootState.STANCE
        elif not heelContactStates[i] and not toeContactState[i]:
            footStates[i] = FootState.SWING
        elif not heelContactStates[i] and toeContactState[i]:
            footStates[i] = FootState.TAKING
        elif heelContactStates[i] and not toeContactState[i]:
            footStates[i] = FootState.LANDING
#        elif not heelContactStates[i] and toeContactState[i] or heelContactStates[i] and not toeContactState[i]:
#            if prevIntervalState == STANCE:
#                footStates[i] = TAKING
#            if prevIntervalState == SWING:
#                footStates[i] = LANDING
    return footStates        

pass
#===============================================================================
# states / intervals picking 
#===============================================================================
def getStateFromStates(floatFrame, frameStates):
    floor = int(math.floor(floatFrame))
    ceil = int(math.ceil(floatFrame))
    if frameStates[floor] == frameStates[ceil]:
        return frameStates[floor]
    else:
        if floatFrame - floor >= .5:
            return frameStates[ceil]  
        else:
            return frameStates[floor]

def getIntervalFromStates(floatFrame, frameStates):
    currentState = getStateFromStates(floatFrame, frameStates)
    interval = [0, len(frameStates)-1]
    for i in range(int(math.ceil(floatFrame)), len(frameStates)):
        if i <= len(frameStates)-1:
            if frameStates[i] != currentState:
                interval[1] = i-.5
                break
    for i in range(int(math.floor(floatFrame)), -1, -1):
        if i >= 0:
            if frameStates[i] != currentState:
                interval[0] = i+.5
                break
    return interval

def getStateIntervalFromStates(floatFrame, frameStates):
    currentState = getStateFromStates(floatFrame, frameStates)
    interval = [0, len(frameStates)-1]
    for i in range(int(math.ceil(floatFrame)), len(frameStates)):
        if i <= len(frameStates)-1:
            if frameStates[i] != currentState:
                interval[1] = i-.5
                break
    for i in range(int(math.floor(floatFrame)), -1, -1):
        if i >= 0:
            if frameStates[i] != currentState:
                interval[0] = i+.5
                break
    return currentState, interval

def getStateFromIntervals(floatFrame, frameIntervals, intervalStates):
    for i in range(len(frameIntervals)):
        if inInterval(floatFrame, frameIntervals[i]):
            return intervalStates[i]
    return None

def getIntervalFromIntervals(floatFrame, frameIntervals):
    for i in range(len(frameIntervals)):
        if inInterval(floatFrame, frameIntervals[i]):
            return frameIntervals[i] 
    return None

def getStateIntervalFromIntervals(floatFrame, frameIntervals, intervalStates):
    for i in range(len(frameIntervals)):
        if inInterval(floatFrame, frameIntervals[i]):
            return intervalStates[i], frameIntervals[i]
    return None, None

def getStateFromBorders(floatFrame, borderFrames, borderStates):
    return getStateFromIntervals(floatFrame, borders2intervals(borderFrames, True), borders2intervals(borderStates, True))

def getIntervalFromBorders(floatFrame, borderFrames):
    return getIntervalFromIntervals(floatFrame, borders2intervals(borderFrames, True))

def getStateIntervalFromBorders(floatFrame, borderFrames, borderStates):
    return getStateIntervalFromIntervals(floatFrame, borders2intervals(borderFrames, True), borders2intervals(borderStates, True))

def getIntervalsWithState(state, frameIntervals, intervalStates):
    intervals = []
    for i in range(len(frameIntervals)):
        if state == intervalStates[i]:
            intervals.append(frameIntervals[i])
    return intervals

pass
#===============================================================================
# interval manipulation
#===============================================================================
def inInterval(frame, interval):
    if frame>=interval[0] and frame<interval[1]:
        return True
    else:
        return False
    
def offsetInterval(offset, interval):
    return [interval[0]+offset, interval[1]+offset]

# [2.2, 5.8] -> [3, 5]
# [2, 4] -> [2, 4]
def intIntervalInner(interval):
    return [int(math.ceil(interval[0])), int(math.floor(interval[1]))]

# [2.2, 5.8] -> [2, 6]
# [2, 4] -> [2, 4]
def intIntervalOuter(interval):
    return [int(math.floor(interval[0])), int(math.ceil(interval[1]))]

# [2.2, 5.8] -> [3, 6]
# [2, 4] -> [2, 4]
def intIntervalUp(interval):
    return [int(math.ceil(interval[0])), int(math.ceil(interval[1]))]

# [2.2, 5.8] -> [2, 5]
# [2, 4] -> [2, 4]
def intIntervalDown(interval):
    return [int(math.floor(interval[0])), int(math.floor(interval[1]))]
    
pass
#===============================================================================
# states / intervals / borders converting
#===============================================================================
def states2intervals(frameStates, stateGroups=None):
    def inSameGroup(state0, state1, stateGroups):
        if stateGroups == None:
            return state0 == state1
        else:
            for group in stateGroups:
                if state0 in group and state1 in group:
                    return True
            return False
        
    def stateGroup(state, stateGroups):
        if stateGroups == None:
            return state
        else:
            for group in stateGroups:
                if state in group:
                    return group
            return None
    
    frameIntervals = []
    intervalStates = []
    
    for i in range(0, len(frameStates)):
        if i == 0:
            currentInterval = [0, None]
            
#        elif frameStates[i]!=frameStates[i-1]:
        elif not inSameGroup(frameStates[i], frameStates[i-1], stateGroups):
            currentInterval[1] = i-.5
            frameIntervals.append(currentInterval)
            
#            intervalStates.append(frameStates[i-1])
            intervalStates.append(stateGroup(frameStates[i-1], stateGroups))
            
            if i == len(frameStates)-1:
                currentInterval = [i-.5, i]
                frameIntervals.append(currentInterval)
                
#                intervalStates.append(frameStates[i])
                intervalStates.append(stateGroup(frameStates[i], stateGroups))
            else:
                currentInterval = [i-.5, None]
                
        else:
            if i == len(frameStates) - 1:
                currentInterval[1] = i
                frameIntervals.append(currentInterval)
                
#                intervalStates.append(frameStates[i])
                intervalStates.append(stateGroup(frameStates[i], stateGroups))
                
    return frameIntervals, intervalStates

def intervals2states(frameIntervals, intervalsStates):
    pass
 
# frames = [0, 3, 6, 10]
# return [[0, 3], [3, 6], [6, 10]] if overlap = True
#        [[0, 3], [4, 6], [7, 10]] if overlap = False
def borders2intervals(frames, overlap=True):
    intervals = []
    for i in range(len(frames)):
        if i < len(frames)-1:
            if overlap:
                intervals.append([frames[i], frames[i+1]])
            else:
                intervals.append([frames[i]+1 if i>0 else frames[i], frames[i+1]])
    return intervals

pass
#===============================================================================
# frame values manipulation 
#===============================================================================
def getMaxValueFrame(frameValues, interval=None):
    if interval==None:
        interval = [0, len(frameValues)-1]
    else:
        if interval[0] < 0:
            interval[0] = 0
        if interval[1] > len(frameValues)-1:
            interval[1] = len(frameValues)-1
    maxFrame = int(math.ceil(interval[0]))
    maxValue = frameValues[int(math.ceil(interval[0]))]
    for i in range(int(math.ceil(interval[0])), int(math.floor(interval[1]))+1):
        if frameValues[i] > maxValue:
            maxFrame = i
            maxValue = frameValues[i]
    return maxFrame, maxValue
    
def getMinValueFrame(frameValues, interval=None):
    if interval==None:
        interval = [0, len(frameValues)-1]
    else:
        if interval[0] < 0:
            interval[0] = 0
        if interval[1] > len(frameValues)-1:
            interval[1] = len(frameValues)-1
    minFrame = int(math.ceil(interval[0]))
    minValue = frameValues[int(math.ceil(interval[0]))]
    for i in range(int(math.ceil(interval[0])), int(math.floor(interval[1]))+1):
        if frameValues[i] < minValue:
            minFrame = i
            minValue = frameValues[i]
    return minFrame, minValue

pass
#===============================================================================
# posture distance
#===============================================================================
def distanceByRelPos(jointPosture0, jointPosture1):
    distance = 0.
    rootPos0 = jointPosture0.getPosition(jointPosture0.skeleton.rootIndex)
    rootPos1 = jointPosture1.getPosition(jointPosture1.skeleton.rootIndex)
    for i in range(jointPosture0.skeleton.getElementNum()):
        relPos0 = jointPosture0.getPosition(i) - rootPos0
        relPos1 = jointPosture1.getPosition(i) - rootPos1
        distance += mmMath.length(relPos0 - relPos1)
    return distance

def distanceByRelPos2(jointPosture0, jointPosture1):
    distance = 0.
    rootPos0 = jointPosture0.rootPos
    rootPos1 = jointPosture1.rootPos
    for i in range(jointPosture0.skeleton.getJointNum()):
        relPos0 = jointPosture0.getJointPositionGlobal(i) - rootPos0
        relPos1 = jointPosture1.getJointPositionGlobal(i) - rootPos1
        distance += mmMath.length(relPos0 - relPos1)
    return distance

#===============================================================================
# motion manipulation
#===============================================================================
def sliceMotion(motion, intInterval):
    return motion[intInterval[0]:intInterval[1]+1]

def splitMotionIntoSegments(motion, intervals):
    return [sliceMotion(motion, interval) for interval in intervals]

pass
#===============================================================================
# foot peak states  
#===============================================================================
    
# footAngles : + directions means pulling up toes
def getFootPeakFramesAndTypes(footAngles, footStates, gap):
    frameIntervals, intervalStates = states2intervals(footStates) 
    peakFrames = []
    peakTypes = []
    
    peakFrames.append(0)
    peakTypes.append(FootPeakType.START_END)
    for i in range(len(frameIntervals)):
        if intervalStates[i]==FootState.STANCE:
#             peakFrames.append((frameIntervals[i][0]+frameIntervals[i][1])/2.)
#             peakTypes.append(STANCE_MID)
             peakFrames.append(getMaxValueFrame(footAngles, [frameIntervals[i][1], frameIntervals[i][1]+gap])[0])
             peakTypes.append(FootPeakType.TAKING_PULL)
        elif intervalStates[i]==FootState.SWING:
             peakFrames.append(getMinValueFrame(footAngles, frameIntervals[i])[0])
             peakTypes.append(FootPeakType.TAKING_PUSH)
             peakFrames.append(getMinValueFrame(footAngles, [frameIntervals[i][1], frameIntervals[i][1]+gap])[0])
             peakTypes.append(FootPeakType.LANDING_PUSH)
             
    peakFrames.append(len(footStates)-1)
    peakTypes.append(FootPeakType.START_END)
    return peakFrames, peakTypes

def getFrameIntervalAndIndexIntervalFromLines(frame, frameLines, len_frame):
    frameInterval = [0, len_frame-1]
    indexInterval = [FootPeakType.START_END, FootPeakType.START_END]
    if frame < frameLines[0]:
        frameInterval[1] = frameLines[0]
        indexInterval[1] = 0
    elif frame >= frameLines[len(frameLines)-1]:
        frameInterval[0] = frameLines[len(frameLines)-1]
#        frameInterval[1] = frameLines[len(frameLines)-1]
        indexInterval[0] = len(frameLines)-1
#        indexInterval[1] = len(frameLines)-1
    else:
        for i in range(len(frameLines)):
            if frame >= frameLines[i]:
                frameInterval[0] = frameLines[i]
                frameInterval[1] = frameLines[i+1]
                indexInterval[0] = i
                indexInterval[1] = i+1
    return frameInterval, indexInterval

if __name__ == '__main__':
    import copy, numpy
    from fltk import *
    import Resource.ysMotionLoader as yf
    import Motion.ysMotionConverter as ymc
    import Util.ysMatplotEx as ymp
    import GUI.ysSimpleViewer as ysv
    import Renderer.ysRenderer as yr
    import Motion.ysMotion as ym

    def test_mapulate_funcs():
        str = '''contactStates = [0,0,0,1,1,1,0,0,0,2,2,2]
        print getStateFromStates(2.4, contactStates)
        print getIntervalFromStates(2.4, contactStates)
        print getStateIntervalFromStates(2.4, contactStates)
        
        intervals, states = states2intervals(contactStates)
        print getStateFromIntervals(2.4, intervals, states)
        print getIntervalFromIntervals(2.4, intervals)
        print getStateIntervalFromIntervals(2.4, intervals, states)
        
        print getIntervalsWithState(0, intervals, states)
        
        print states2intervals(contactStates)
        print states2intervals(contactStates,[[1],[0,2]])
        
        borders = [0, 3, 6, 10]
        print borders2intervals(borders, True)
        print borders2intervals(borders, False)
        floatBorders= [0., 2.5, 6.1, 10.]
        print borders2intervals(floatBorders, True)
        
        print intIntervalInner([2.2, 5.8])   
        print intIntervalInner([2,4])   
        
        print intIntervalOuter([2.2, 5.8])   
        print intIntervalOuter([2,4])   
        
        print intIntervalUp([2.2, 5.8])   
        print intIntervalUp([2,4])
           
        print intIntervalDown([2.2, 5.8])   
        print intIntervalDown([2,4])   

        
        peakFrames, peakTypes = ([0, 54, 58, 75, 95, 99, 116, 136, 140, 152, 193, 191, 192, 193, 193], [4, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 4])
        currentFrame = 100
        
        peakInterval, indexInterval = getFrameIntervalAndIndexIntervalFromLines(currentFrame, peakFrames, 193)
        print peakInterval, indexInterval, [FootPeakType.text[peakTypes[indexInterval[0]]], FootPeakType.text[peakTypes[indexInterval[1]]]]
        
        print getStateFromBorders(currentFrame, peakFrames, peakTypes)
        print getIntervalFromBorders(currentFrame, peakFrames)
        print getStateIntervalFromBorders(currentFrame, peakFrames, peakTypes)
        '''
        codes = str.split('\n')
        
        for code in codes:
            code = code.lstrip()
            if len(code)>0 and code[0] == '#':
                continue
            print code,
            if 'print' in code:
#                print ':',
                print ':'
            exec code
            if 'print' not in code:
                print
        
    def test_distancFuncs():
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        jointMotion = yf.readBvhFile(bvhFilePath, .01)

        rankNum = 5
        
        srcFrame = 100
        searchIntervals = [[0,80], [120,200]]
        print 'srcFrame', srcFrame
        print 'searchIntervals', searchIntervals


        def isIn(i, searchIntervals):
            for searchInterval in searchIntervals:
                if i >= searchInterval[0] and i <= searchInterval[1]:
                    return True
            return False

        distances = [None]*len(jointMotion)
        for i in range(len(jointMotion)):
            if isIn(i, searchIntervals):
                distances[i] = distanceByRelPos(jointMotion[srcFrame], jointMotion[i])
            else:
                distances[i] = sys.maxint
        sorted = copy.copy(distances)
        sorted.sort()
        
        print 'rank \t frame \t distance'
        rankedPostures = [None]*rankNum
        for i in range(1,rankNum):
            for j in range(len(jointMotion)):
                if sorted[i] == distances[j]:
                    rankedPostures[i] = jointMotion[j]
                    print '%d \t %d \t %f'%(i, j, distances[j])
                    
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        
        viewer.doc.addRenderer('jointMotion', yr.JointMotionRenderer(jointMotion, (0,0,255), yr.LINK_BONE))
        viewer.doc.addObject('jointMotion', jointMotion)
        viewer.doc.addRenderer('srcFrame', yr.JointMotionRenderer(jointMotion[srcFrame:srcFrame+1], (0,255,0), yr.LINK_BONE))
        for i in range(1, rankNum):
            viewer.doc.addRenderer('%d'%i, yr.JointMotionRenderer(ym.Motion([rankedPostures[i]]), numpy.array([255,255,255])*(1-(i-1)*.2), yr.LINK_BONE))

        viewer.startTimer(1/30.)
        viewer.show()
        Fl.run()

    
    pass        
    test_mapulate_funcs()
#    test_distancFuncs()
