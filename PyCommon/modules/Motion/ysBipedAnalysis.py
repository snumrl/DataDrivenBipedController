# +-------------------------------------------------------------------------
# | ysBipedAnalysis.py
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
import Util.ysPythonEx as ype
import Motion.ysMotionAnalysis as yma

class GaitState:
    STOP = 0
    LSWING = 1
    RSWING = 2
    JUMP = 3
    text = ype.getReverseDict(locals())

# return interval
def getWalkingCycle(jointMotion, one_contactStates, endZoneSize = 10):
    intervals, types = yma.states2intervals(one_contactStates)
    half1stIndex = (len(intervals)-1)/2 - 1
    half2ndIndex = half1stIndex + 1
#    print half1stIndex, half2ndIndex, intervals[half1stIndex], intervals[half2ndIndex]
    
#    startFrame = intervals[half1stIndex][0]
#    endFrame = intervals[half2ndIndex][-1]
    startFrame = int(math.ceil(intervals[half1stIndex][0]))
    endFrame = int(math.floor(intervals[half2ndIndex][-1]))
    
    minDistance = sys.maxint
    minFrame = 0
    for i in range(endFrame-endZoneSize, endFrame+endZoneSize):
        d = yma.distanceByRelPos(jointMotion[startFrame], jointMotion[i])
#        print i, d
        if d < minDistance:
#            print 'min', i, d
            minDistance = d
            minFrame = i
    endFrame = minFrame
                        
    return [startFrame, endFrame]

def getWalkingCycle2(jointMotion, one_contactStates, endZoneSize = 10):
    intervals, types = yma.states2intervals(one_contactStates)
    half1stIndex = (len(intervals)-1)/2 - 1
    half2ndIndex = half1stIndex + 1
#    print half1stIndex, half2ndIndex, intervals[half1stIndex], intervals[half2ndIndex]
    
#    startFrame = intervals[half1stIndex][0]
#    endFrame = intervals[half2ndIndex][-1]
    startFrame = int(math.ceil(intervals[half1stIndex][0]))
    endFrame = int(math.floor(intervals[half2ndIndex][-1]))
    
    minDistance = sys.maxint
    minFrame = 0
    for i in range(endFrame-endZoneSize, endFrame+endZoneSize):
        d = yma.distanceByRelPos2(jointMotion[startFrame], jointMotion[i])
#        print i, d
        if d < minDistance:
#            print 'min', i, d
            minDistance = d
            minFrame = i
    endFrame = minFrame
                        
    return [startFrame, endFrame]

# depreciate
def getWalkingSteps(left_contactStates, right_contactStates, includeFirstLastSteps=False, overlap=True):
    l_taking, l_landing = yma.getTakingLandingFrames(left_contactStates)
    r_taking, r_landing = yma.getTakingLandingFrames(right_contactStates)
    landing = l_landing + r_landing
    landing.sort()
    
    if includeFirstLastSteps:
#        landing.insert(0, min(l_taking[0], r_taking[0]))
        landing.insert(0, 0)
    else:
        del landing[-1]
        
    return yma.borders2intervals(landing, overlap)

def getBipedGaitStates(lFootContactStates, rFootContactStates, jumpThreshold = 0, jumpBias = .5, stopThreshold = 0, stopBias = .5):
    gaitStates = [None]*len(lFootContactStates)
    
    for i in range(len(lFootContactStates)):
        if lFootContactStates[i] and not rFootContactStates[i]:
            gaitStates[i] = GaitState.RSWING
        elif not lFootContactStates[i] and rFootContactStates[i]:
            gaitStates[i] = GaitState.LSWING
        elif lFootContactStates[i] and rFootContactStates[i]:
            gaitStates[i] = GaitState.STOP
        elif not lFootContactStates[i] and not rFootContactStates[i]:
            gaitStates[i] = GaitState.JUMP
            
    # check thresholds
#    intervals, types = getBipedGaitIntervalsTypes(gaitStates)
#    stopIntervals = getSpecifiedTypeIntervals(GaitState.STOP, intervals, types)
#    jumpIntervals = getSpecifiedTypeIntervals(GaitState.JUMP, intervals, types)

    intervals, types = yma.states2intervals(gaitStates)
    intervals = [yma.intIntervalInner(interval) for interval in intervals]
    stopIntervals = yma.getIntervalsWithState(GaitState.STOP, intervals, types)
    jumpIntervals = yma.getIntervalsWithState(GaitState.JUMP, intervals, types)
    
    total = [stopIntervals, jumpIntervals]
    thresholds = [stopThreshold, jumpThreshold]
    biases = [stopBias, jumpBias]

    for b in range(len(total)):
        intervals = total[b]
        threshold = thresholds[b]
        bias = biases[b]
        
        for interval in intervals:
            if interval[0] == 0 or interval[1] == len(gaitStates)-1:
                continue
            prevState = gaitStates[interval[0]-1] 
            nextState = gaitStates[interval[1]+1] 
                
            if interval[1] - interval[0] < threshold:
                mid = (interval[1] + interval[0])*bias
                for i in range(interval[0], interval[1]+1):
                    if i < mid:
                        gaitStates[i] = prevState
                    else:
                        gaitStates[i] = nextState
                          
    return gaitStates

def getBipedGaitIntervals(lFootContactStates, rFootContactStates, jumpThreshold = 0, jumpBias = .5, stopThreshold = 0, stopBias = .5):
    states = getBipedGaitStates(lFootContactStates, rFootContactStates, jumpThreshold, jumpBias, stopThreshold, stopBias)
    intervals, states = yma.states2intervals(states)
    return [yma.intIntervalUp(interval) for interval in intervals], states 

def getBipedGaitStates2(lFootContactStates, rFootContactStates, jumpThreshold = 0, jumpBias = .5, stopThreshold = 0, stopBias = .5):
    gaitStates = [None]*len(lFootContactStates)
    
    for i in range(len(lFootContactStates)):
        if lFootContactStates[i] and not rFootContactStates[i]:
            gaitStates[i] = GaitState.RSWING
        elif not lFootContactStates[i] and rFootContactStates[i]:
            gaitStates[i] = GaitState.LSWING
        elif lFootContactStates[i] and rFootContactStates[i]:
            gaitStates[i] = GaitState.STOP
        elif not lFootContactStates[i] and not rFootContactStates[i]:
            gaitStates[i] = GaitState.JUMP
            
    # check thresholds
#    intervals, types = getBipedGaitIntervalsTypes(gaitStates)
#    stopIntervals = getSpecifiedTypeIntervals(GaitState.STOP, intervals, types)
#    jumpIntervals = getSpecifiedTypeIntervals(GaitState.JUMP, intervals, types)

    intervals, types = yma.states2intervals(gaitStates)
    intervals = [yma.intIntervalInner(interval) for interval in intervals]
    stopIntervals = yma.getIntervalsWithState(GaitState.STOP, intervals, types)
    jumpIntervals = yma.getIntervalsWithState(GaitState.JUMP, intervals, types)
    
    total = [stopIntervals, jumpIntervals]
    thresholds = [stopThreshold, jumpThreshold]
    biases = [stopBias, jumpBias]

    for b in range(len(total)):
        intervals = total[b]
        threshold = thresholds[b]
        bias = biases[b]
        
        for interval in intervals:
#            if interval[0] == 0 or interval[1] == len(gaitStates)-1:
#                continue
#            prevState = gaitStates[interval[0]-1] 
#            nextState = gaitStates[interval[1]+1] 

            # temp - to be fixed
            if interval[0] == 0:
                prevState = gaitStates[interval[1]+1]
                nextState = gaitStates[interval[1]+1]
            elif interval[1] == len(gaitStates)-1:
                prevState = gaitStates[interval[0]-1]
                nextState = gaitStates[interval[0]-1]
            else:
                prevState = gaitStates[interval[0]-1] 
                nextState = gaitStates[interval[1]+1]
                
            if interval[1] - interval[0] < threshold:
                mid = (interval[1] + interval[0])*bias
                for i in range(interval[0], interval[1]+1):
                    if i < mid:
                        gaitStates[i] = prevState
                    else:
                        gaitStates[i] = nextState
                          
    return gaitStates

def getBipedGaitIntervals2(lFootContactStates, rFootContactStates, jumpThreshold = 0, jumpBias = .5, stopThreshold = 0, stopBias = .5):
    states = getBipedGaitStates2(lFootContactStates, rFootContactStates, jumpThreshold, jumpBias, stopThreshold, stopBias)
    intervals, states = yma.states2intervals(states)
    return [yma.intIntervalUp(interval) for interval in intervals], states 


if __name__ == '__main__':
    import psyco; psyco.full()
    import copy, numpy
    from fltk import *
    import Resource.ysMotionLoader as yf
    import Motion.ysMotionConverter as ymc
    import Util.ysMatplotEx as ymp
    import GUI.ysSimpleViewer as ysv
    import Renderer.ysRenderer as yr
    import Motion.ysMotion as ym
    
    def test_getWalkingCycle():
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        jointMotion = yf.readBvhFile(bvhFilePath, .01)
        
        lFoot = jointMotion[0].skeleton.getElementIndex('LeftFoot')
        rFoot = jointMotion[0].skeleton.getElementIndex('RightFoot')
        
        hRef = .1; vRef = .3
        lc = yma.getElementContactStates(jointMotion, lFoot, hRef, vRef)
#        rc = getElementContactStates(jointMotion, rFoot, hRef, vRef)
        
        interval = getWalkingCycle(jointMotion, lc)
        cycleMotion = jointMotion[interval[0]:interval[-1]+1]
        startMotion = cycleMotion[:1]
        endMotion = cycleMotion[-1:]
    
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('jointMotion', yr.JointMotionRenderer(jointMotion, (0,0,255), yr.LINK_LINE))
        viewer.doc.addObject('jointMotion', jointMotion)
        viewer.doc.addRenderer('startMotion', yr.JointMotionRenderer(startMotion, (0,255,0), yr.LINK_LINE))
        viewer.doc.addObject('startMotion', startMotion)
        viewer.doc.addRenderer('endMotion', yr.JointMotionRenderer(endMotion, (0,255,0), yr.LINK_LINE))
        viewer.doc.addObject('endMotion', endMotion)

        viewer.startTimer(1/30.)
        viewer.show()
        Fl.run()
        
    def test_getWalkingSteps():
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        motion = yf.readBvhFile(bvhFilePath, .01)
        motion = motion[:-10]

        lFoot = motion[0].skeleton.getElementIndex('LeftFoot')
        rFoot = motion[0].skeleton.getElementIndex('RightFoot')
        
        hRef = .1; vRef = .3
        lc = yma.getElementContactStates(motion, lFoot, hRef, vRef)
        rc = yma.getElementContactStates(motion, rFoot, hRef, vRef)
        
        t, l_landing = yma.getTakingLandingFrames(lc)
        t, r_landing = yma.getTakingLandingFrames(rc)
        landing = l_landing + r_landing
        landing.sort()
        print 'landingFrames', landing
        
#        steps = getWalkingSteps(lc, rc, True)
        steps = getWalkingSteps(lc, rc, True, False)
#        steps = getWalkingSteps(lc, rc, False)
        print 'steps', steps
        
        print
        stepMotions = yma.splitMotionIntoSegments(motion, steps)
        for i in range(len(steps)):
            print 'stepMotions[%d]: motion[%d]~motion[%d], len %d'%(i, steps[i][0], steps[i][1], len(stepMotions[i]))
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,100,255), yr.LINK_LINE, 3.))
        viewer.doc.addObject('motion', motion)
        for i in range(len(steps)):
            viewer.doc.addRenderer('stepMotions[%d]'%i, yr.JointMotionRenderer(stepMotions[i], (0,255,0), yr.LINK_LINE, 3.))
            viewer.doc.addObject('stepMotions[%d]', stepMotions[i])
        
        viewer.startTimer(1/30.)
        viewer.show()
        Fl.run()        

    def test_getBipedGaitStates():
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        motion = yf.readBvhFile(bvhFilePath, .01)
        
        hRef = .1; vRef = .3
        lc = yma.getElementContactStates(motion, motion[0].skeleton.getElementIndex('LeftFoot'), hRef, vRef)
        rc = yma.getElementContactStates(motion, motion[0].skeleton.getElementIndex('RightFoot'), hRef, vRef)

        rawStateList = getBipedGaitStates(lc, rc)
        cookedStateList = getBipedGaitStates(lc, rc, 10, 1.)
        
        intervals, types = yma.states2intervals(cookedStateList)
        for i in range(len(intervals)):
            print intervals[i], GaitState.text[types[i]]

        print
        print [yma.intIntervalUp(int) for int in intervals]
        print getWalkingSteps(lc, rc, True)
        print getBipedGaitIntervals(lc, rc, 10, 1.)
        
        plot = ymp.SmartPlot()
        plot.setXdata('frame', range(len(motion)))
        plot.addYdata('rawState', rawStateList)
        plot.addYdata('cookedState', cookedStateList)
        plot.showModeless()
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_LINE))
        viewer.doc.addObject('motion', motion)
        viewer.startTimer(1/30.)
        viewer.show()
        Fl.run()

    pass        
#    test_getWalkingCycle()
#    test_getWalkingSteps()
    test_getBipedGaitStates()