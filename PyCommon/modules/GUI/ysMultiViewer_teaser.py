# +-------------------------------------------------------------------------
# | ysMultiViewer_teaser.py
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

from fltk import *
import copy

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Motion.ysMotion as ym
import GUI.ysBaseUI as ybu
#import GUI.ysViewer3 as yv3
import GUI.seViewer3_ys_teaser as yv3
import GUI.tree as tree
from GUI.ysViewer3 import *

Fl.scheme('default')

# EVENTS
EV_addRenderer           = 0
EV_setRendererVisible    = 1
EV_addObject             = 2
EV_selectObjectElement   = 3
EV_selectObject          = 4

CAMERA_DISTANCE = 2.8

class MultiSetting(ybu.BaseSettings):
    def __init__(self):
        ybu.BaseSettings.__init__(self)
        self.camera = yv3.Camera().__dict__
        self.ortho = False
        self.viewMode = yv3.VIEW_PERSPECTIVE
        self.prevRotX = 0
        self.prevRotY = 0
    def setToApp(self, window):
        ybu.BaseSettings.setToApp(self, window)
        window.glWindow1.camera.__dict__ = copy.deepcopy(self.camera)
        window.glWindow1.projectionOrtho = self.ortho
        window.glWindow1.viewMode = self.viewMode
        window.glWindow1.prevRotX = self.prevRotX
        window.glWindow1.prevRotY = self.prevRotY
        window.glWindow2.camera.__dict__ = copy.deepcopy(self.camera)
        window.glWindow2.projectionOrtho = self.ortho
        window.glWindow2.viewMode = self.viewMode
        window.glWindow2.prevRotX = self.prevRotX
        window.glWindow2.prevRotY = self.prevRotY
    def getFromApp(self, window):
        ybu.BaseSettings.getFromApp(self, window)
        self.camera = window.glWindow1.camera.__dict__
        self.ortho = window.glWindow1.projectionOrtho
        self.viewMode = window.glWindow1.viewMode
        self.prevRotX = window.glWindow1.prevRotX
        self.prevRotY = window.glWindow1.prevRotY

class GlWindow2(yv3.GlWindow):
#    def viewFromFront(self):
#        yv3.GlWindow.viewFromFront(self)
#        self.syncFriend()
#    def viewFromRight(self):
#        yv3.GlWindow.viewFromRight(self)
#        self.syncFriend()
#    def viewFromTop(self):
#        yv3.GlWindow.viewFromTop(self)
#        self.syncFriend()
#    def viewPerspective(self):
#        yv3.GlWindow.viewPerspective(self)
#        self.syncFriend()
    def __init__(self, x, y, w, h, parent = None):
        yv3.GlWindow.__init__(self, x, y, w, h, parent)
        self.syncPos = True
        
    def handle(self, e):
        ret = yv3.GlWindow.handle(self, e)
        if e == FL_DRAG or e == FL_MOUSEWHEEL:# or e == FL_KEYUP: 
            self.syncFriend()
        return ret
    def syncFriend(self):
        if self.syncPos:
            self.friend.camera.__dict__ = copy.deepcopy(self.camera.__dict__)
        else:
            self.friend.camera.rotateY = self.camera.rotateY
            self.friend.camera.rotateX = self.camera.rotateX
            self.friend.camera.distance = self.camera.distance
            self.friend.camera.center[1] = self.camera.center[1]  
            
#            centerX = self.friend.camera.center[0]
#            centerZ = self.friend.camera.center[2]
#            self.friend.camera.__dict__ = self.camera.__dict__
#            self.friend.camera.center[0] = 0
#            self.friend.camera.center[2] = 0
            
        self.friend.projectionOrtho = self.projectionOrtho
        self.friend.viewMode = self.viewMode
        self.friend.prevRotX = self.prevRotX
        self.friend.prevRotY = self.prevRotY
        self.friend.redraw()

#class MultiViewer(Fl_Window):
class MultiViewer(ybu.BaseWnd):
#    def __init__(self, x, y, w, h):
#        title = 'MotionViewer'
#        Fl_Window.__init__(self, x, y, w, h, title)
    def __init__(self, w=None, h=None, singleView=False, wheelWork=False, reflection=False, shadow=True, title='MultiViewer'):
        ybu.BaseWnd.__init__(self, None, title, MultiSetting())
        
        if len(self.settingsFile)>0:
            self.settings.load(self.settingsFile)
            self.position(self.settings.x, self.settings.y)
            if w!=None and h!=None:
                self.size(w, h)
            else:
                self.size(self.settings.w, self.settings.h)
            
        gap = 4
        self.begin()
        
        self.glContainer = Fl_Window(0, 0, self.w(), self.h()-55)
        self.glContainer.begin()
        if not singleView:
            self.glWindow1 = GlWindow2(0, 0, self.w()/2-gap/2, self.h()-55, self.glContainer)
            self.glWindow2 = GlWindow2(self.w()/2+gap/2, 0, self.w()/2, self.h()-55, self.glContainer)
        else:
            self.glWindow1 = GlWindow2(0, 0, 0, 0, self.glContainer)
            self.glWindow1.hide()
            self.glWindow2 = GlWindow2(0, 0, self.w(), self.h()-55, self.glContainer)
        self.glContainer.end()
        self.glContainer.color(FL_BLACK)
        
#        self.glWindow1 = GlWindow2(0, 0, self.w()/2-gap/2, self.h()-55, self)
#        self.glWindow2 = GlWindow2(self.w()/2+gap/2, 0, self.w()/2, self.h()-55, self)
        self.panel = ControlPanel(0, self.h()-55, self.w(), 55, self)
        self.end()
        
        if len(self.settingsFile)>0:
            self.settings.setToApp(self)
        if len(self.settingsFile)>0:
            self.settings.load(self.settingsFile)
            self.position(self.settings.x, self.settings.y)
            if w!=None and h!=None:
                self.size(w, h)
            else:
                self.size(self.settings.w, self.settings.h)
            
        self.resizable(self.glWindow2)
        
        self.glWindow1.friend = self.glWindow2
        self.glWindow2.friend = self.glWindow1
        self.glWindow1.theme = self.glWindow2.theme = yv3.THEME_GREY
        
        if not wheelWork:
            self.glWindow1.camera.distance = CAMERA_DISTANCE
            self.glWindow2.camera.distance = CAMERA_DISTANCE
        
        self.glWindow1.wheelWork = wheelWork
        self.glWindow1.reflection = reflection
        self.glWindow1.shadow = shadow
        self.glWindow2.wheelWork = wheelWork
        self.glWindow2.reflection = reflection
        self.glWindow2.shadow = shadow
        
        self.initialize()

    def initialize(self):
        self.playing = False
        self.recording = True
        self.frame = -1
        self.maxFrame = 0
        self.maxRecordedFrame = 0
        self.loaded = False
        
        self.motionSystem = None
        self.glWindow1.renderers = []
        self.glWindow2.renderers = []
        
        self.sceneStates1 = []
        self.sceneStates2 = []
        self.initSceneState1 = None
        self.initSceneState2 = None
        
        self.panel.updateAll()     
        
        self.initialUpdate = True

    def setSimulateCallback(self, callback):
        self.simulateCallback = callback
    def setPostFrameCallback_Always(self, callback):           
        self.postFrameCallback_Always = callback
    def setCameraTarget1(self, targetPos):
        self.glWindow1.camera.center[0] = targetPos[0]
        self.glWindow1.camera.center[2] = targetPos[2]
        self.glWindow1.syncPos = False
    def setCameraTarget2(self, targetPos):
        self.glWindow2.camera.center[0] = targetPos[0]
        self.glWindow2.camera.center[2] = targetPos[2]
        self.glWindow2.syncPos = False
    def setRenderers1(self, renderers):
        self.glWindow1.renderers = renderers
    def setRenderers2(self, renderers):
        self.glWindow2.renderers = renderers
    
    def setMotionSystem(self, motionSystem):
        self.motionSystem = motionSystem
        self.loaded = True
        self.setMaxFrame(motionSystem.getMaxFrame())
        self.panel.updateControl(self.loaded)

    def getMaxFrame(self):
        return self.maxFrame

    def setMaxFrame(self, maxFrame):
        self.maxFrame = maxFrame
        self.panel.updateMaxFrame(maxFrame)
#        self.recordedData = [None]*(self.maxFrame+1)
        self.sceneStates1 = [None]*(self.maxFrame+1)
        self.sceneStates2 = [None]*(self.maxFrame+1)
    
    def setCurrentFrame(self, frame):
        self.frame = frame
        self.panel.updateFrame(frame)
    def getCurrentFrame(self):
        return self.frame
             
    def onTimer(self):
#        if self.initialUpdate:
#            self.saveInitStates()
#            self.loadInitStates()
#            self.initialUpdate = False
            
        if self.playing:
            self.frame += 1
            if self.frame > self.maxFrame:
#                self.frame = 0
                self.frame = self.maxFrame
                self.playing = False
            self.onFrame(self.frame)
                
        if self.timeInterval:
            Fl.repeat_timeout(self.timeInterval, self.onTimer)
            
    def preFrameCallback_Always(self, frame):
        pass
    def preFrameCallback(self, frame):
        pass
    def simulateCallback(self, frame):
        pass
    def postFrameCallback(self, frame):
        pass
    def postFrameCallback_Always(self, frame):
        pass
    
    # onFrame -1
    def loadInitStates(self):
        self.glWindow1.setState(self.initSceneState1)
        self.glWindow2.setState(self.initSceneState2)
        self.panel.updateFrame(self.frame)
        self.glWindow1.redraw()
        self.glWindow2.redraw()
    def saveInitStates(self):
        self.initSceneState1 = self.glWindow1.getState()
        self.initSceneState2 = self.glWindow2.getState()

    def onFrame(self, frame):
        if self.motionSystem:
            self.motionSystem.updateFrame(frame)
        
        self.preFrameCallback_Always(frame)
                
#        print '[FRAMELOG]onFrame', frame
        if self.recording:
            if self.sceneStates1[frame]==None:
                if frame == 0 or self.sceneStates1[self.frame-1]!=None:
                    self.preFrameCallback(frame)
                    self.simulateCallback(frame)
                    self.postFrameCallback(frame)
                    
                    self.saveFrameStates(frame)
                    self.glWindow1.setState(self.sceneStates1[frame])
                    self.glWindow2.setState(self.sceneStates2[frame])
                    
                    self.maxRecordedFrame = frame
                    self.panel.updateRecordedFrame(self.maxRecordedFrame)
                else:
                    self.glWindow1.setState(None)
                    self.glWindow2.setState(None)
            else:
                self.loadFrameStates(frame)
        else:
            self.preFrameCallback(frame)
            self.simulateCallback(frame)
            self.postFrameCallback(frame)
            self.glWindow1.setState(None)
            self.glWindow2.setState(None)
            
        self.postFrameCallback_Always(frame)
            
        self.panel.updateFrame(self.frame)
        self.glWindow1.redraw()
        self.glWindow2.redraw()
        
    def saveFrameStates(self, frame):
        self.sceneStates1[frame]= self.glWindow1.getState()
        self.sceneStates2[frame]= self.glWindow2.getState()
    def loadFrameStates(self, frame):
        self.glWindow1.setState(self.sceneStates1[frame])
        self.glWindow2.setState(self.sceneStates2[frame])
    def deleteFrameStates(self, frame):
#        print '[FRAMELOG]deletelist', frame
        self.glWindow1.deleteState(self.sceneStates1[frame])
        self.glWindow2.deleteState(self.sceneStates2[frame])
        self.sceneStates1[frame] = None
        self.sceneStates2[frame] = None
        
    def startTimer(self, timeInterval):
        Fl.add_timeout(0.0, self.onTimer)
        self.timeInterval = timeInterval
        
    def endTimer(self):
        self.timeInterval = None
        Fl.remove_timeout(self.onTimer)
        
    def setTimeInterval(self, timeInterval):
        self.timeInterval = timeInterval

    def show(self):
        Fl_Window.show(self)
#        ybu.BaseWnd.show(self)
        self.glWindow1.show()
        self.glWindow2.show()
        self.panel.show()

    def isPlaying(self):
        return self.playing
    def play(self):
        self.playing = True
    def pause(self):
        self.playing = False
    def record(self, recordingOn):
        self.recording = recordingOn
        if recordingOn==False:
            self.resetRecFrom(0)
        self.panel.updateControl(self.loaded)
    def resetRecFrom(self, startFrame):
        for frame in range(startFrame+1, len(self.sceneStates1)):
            if self.sceneStates1[frame]:
                self.deleteFrameStates(frame)
        self.maxRecordedFrame = startFrame
        self.panel.updateRecordedFrame(self.maxRecordedFrame)
    def goToFrame(self, frame):
        self.frame = frame
        if frame==-1:
            self.loadInitStates()
        else:
            self.onFrame(frame)



#class MultiViewer(ybu.BaseWnd):
#    def __init__(self, rect=None, title='MultiViewer'):
#        ybu.BaseWnd.__init__(self, rect, title, MultiSetting())
#        self.doc = SimpleDoc()
#        self.begin()
#        self.motionViewWnd1 = MotionViewWnd(0, 0, self.w()/2, self.h(), self.doc)
#        self.motionViewWnd2 = MotionViewWnd(self.w(), 0, self.w()/2, self.h(), self.doc)
#        self.end()
##        self.resizable(self.motionViewWnd1)
#        self.size_range(600, 400)
#    def startTimer(self, timeInterval):
#        self.motionViewWnd1.startTimer(timeInterval)
#    def endTimer(self):
#        self.motionViewWnd1.endTimer()
#    def setTimeInterval(self, timeInterval):
#        self.motionViewWnd1.setTimeInterval(timeInterval)
#    def show(self):
#        ybu.BaseWnd.show(self)
#        self.motionViewWnd1.show()
#    def setPreFrameCallback(self, callback):
#        self.motionViewWnd1.preFrameCallback = callback
#    def setPreFrameCallback_Always(self, callback):
#        self.motionViewWnd1.preFrameCallback_Always = callback
#    def setSimulateCallback(self, callback):
#        self.motionViewWnd1.simulateCallback = callback
#    def setPostFrameCallback(self, callback):
#        self.motionViewWnd1.postFrameCallback = callback
#    def setPostFrameCallback_Always(self, callback):
#        self.motionViewWnd1.postFrameCallback_Always = callback
#    def setExtraDrawCallback(self, callback):
#        self.motionViewWnd1.glWindow.extraDrawCallback = callback
##    def setRecSimulObjs(self, objs):
##        self.motionViewWnd1.setRecSimulObjs(objs)
#    def getMaxFrame(self):
#        return self.motionViewWnd1.getMaxFrame()
#    def setMaxFrame(self, maxFrame):
#        self.motionViewWnd1.setMaxFrame(maxFrame)
#    def record(self, bRec):
#        self.motionViewWnd1.record(bRec)
#    def play(self):
#        self.motionViewWnd1.play()
#    def setCurrentFrame(self, frame):
#        self.motionViewWnd1.setCurrentFrame(frame)
#    def getCurrentFrame(self):
#        return self.motionViewWnd1.getCurrentFrame()
#    def setCameraTarget(self, targetPos):
#        self.motionViewWnd1.glWindow.camera.center[0] = targetPos[0]
#        self.motionViewWnd1.glWindow.camera.center[2] = targetPos[2]
#    def initialize(self):
#        self.doc.initialize()
#        self.motionViewWnd1.initialize()
#        
#class SimpleDoc(ybu.Subject):
#    def __init__(self):
#        ybu.Subject.__init__(self)
#        
#        self.rendererNames = []
#        self.rendererMap = {}
#        self.renderersVisible = {}
#
#        self.motionNames = []
#        self.motionMap = {}
#        self.motionSystem = ym.MotionSystem()
#        
#        self.objectNames = []
#        self.objectMap = {}
#        self.selectedObject = None
#    def initialize(self):
#        self.removeAllRenderers()
#        self.removeAllObjects()
#    def removeAllRenderers(self):
#        del self.rendererNames[:]
#        self.rendererMap.clear()
#        self.renderersVisible.clear()
#        self.notify(EV_addRenderer)
#    def removeAllObjects(self):
#        del self.objectNames[:]
#        self.objectMap.clear()
#        self.motionSystem.removeAllMotions()
#    def addRenderer(self, name, renderer, visible=True):
#        self.rendererNames.append(name)
#        self.rendererMap[name] = renderer
#        self.renderersVisible[name] = visible
#        self.notify(EV_addRenderer)
#    def setRendererVisible(self, name, visible):
#        self.renderersVisible[name] = visible
#        self.notify(EV_setRendererVisible)
#    def getVisibleRenderers(self):
#        ls = []
#        for name in self.rendererNames:
#            if self.renderersVisible[name]:
#                ls.append(self.rendererMap[name])
#        return ls
#    def addObject(self, name, object):
#        self.objectNames.append(name)
#        self.objectMap[name] = object
#        if isinstance(object, ym.Motion):
#            self.motionSystem.addMotion(object)
#        self.notify(EV_addObject)
#    def selectObjectElement(self, element):
#        for renderer in self.rendererMap.values():
#            renderer.selectedElement = element 
#        self.notify(EV_selectObjectElement)
#    def selectObject(self, objectName):
#        self.selectedObject = self.objectMap[objectName]
#        self.notify(EV_selectObject)
#    
#class MotionViewWnd(yv3.MotionViewer, ybu.Observer):
#    def __init__(self, x, y, w, h, doc):
#        yv3.MotionViewer.__init__(self, x, y, w, h)
#        self.doc = doc
#        self.doc.attach(self)
#    def update(self, ev, doc):
#        if ev==EV_addRenderer or ev==EV_setRendererVisible:
#            self.setRenderers(doc.getVisibleRenderers())
#        elif ev==EV_addObject:
#            self.setMotionSystem(doc.motionSystem)
#            self.setStateObjects(doc.objectMap.values())
#        self.glWindow.redraw()

if __name__=='__main__':
    import psyco; psyco.full()
    import time
    import Resource.ysMotionLoader as yf
    import Renderer.ysRenderer as yr
    import Resource.ysOgreDataLoader as yol
    
#    def test_MultiViewer():
#        pointMotion = yf.readTrcFile('../samples/Day7_Session2_Take01_-_walk.trc', .01)
#        jointMotion = yf.readBvhFile('../samples/wd2_WalkSameSame00.bvh', .01)
#    
#        print 'pointSkeleton'
#        print pointMotion[0].skeleton
#        print 'jointSkeleton'
#        print jointMotion[0].skeleton
#
#        viewer =  ()
#        viewer.record(False)
#        viewer.doc.addRenderer('pointMotion', yr.PointMotionRenderer(pointMotion, (0,255,0)))
#        viewer.doc.addObject('pointMotion', pointMotion)
#        viewer.doc.addRenderer('jointMotion', yr.JointMotionRenderer(jointMotion, (0,255,0)))
#        viewer.doc.addObject('jointMotion', jointMotion)
#        
#        viewer.startTimer(1/pointMotion.fps)
#        viewer.show()
#        
#        Fl.run()
    
    def test_MultiViewer():
        import Motion.ysMotion as ym
        import Resource.ysMotionLoader as yf
        import Renderer.ysRenderer as yr
        
        mmFilePath = '../samples/physics2_WalkSameSame01.mm'
        pointMotion = yf.readMMFile(mmFilePath)
        frameTime = 1./30.
        
        motionSystem = ym.MotionSystem()
        motionSystem.addMotion(pointMotion)
    
        renderers = []
        renderers.append(yr.PointMotionRenderer(pointMotion))
        
#        viewer = MultiViewer()
#        viewer = MultiViewer(800, 655)
#        viewer = MultiViewer(800, 655, True)
        viewer = MultiViewer(1600, 1255)
#        viewer = Viewer(100, 100, 800, 650, motionSystem, renderers)
#        viewer.startTimer(frameTime)
        viewer.show()
        Fl.run()
            
    pass
    test_MultiViewer()
