# +-------------------------------------------------------------------------
# | ysBaseUI.py
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

import cPickle
from fltk import *
Fl.scheme('plastic')


class Subject:
    def __init__(self):
        self.observers = []
    def attach(self, observer):
        self.observers.append(observer)
    def detach(self, observer):
        self.observers.remove(observer)
    def notify(self, event=None):
        for observer in self.observers:
            observer.update(event, self)
            
class Observer:
    def update(self, event, subject):
        raise NotImplementedError("Must subclass me")
    
class BaseSettings:
    def __init__(self):
        self.x = 100; self.y = 100; self.w = 800; self.h = 650
    def load(self, fileName):
        try:
            self.__dict__.update(cPickle.load(open(fileName, 'r')).__dict__)
        except:
            pass
    def save(self, fileName):
        cPickle.dump(self, open(fileName, 'w'))
    def setToApp(self, window):
        window.position(self.x, self.y)
        window.size(self.w, self.h)
    def getFromApp(self, window):
        self.x = window.x(); self.y = window.y(); self.w = window.w(); self.h = window.h()

class BaseWnd(Fl_Window):
    def __init__(self, rect=None, title='BaseWnd', settings=BaseSettings()):
        Fl_Window.__init__(self, 100, 100, 600, 400, title)
        
        self.settingsFile = ''
        if rect == None:
            self.settingsFile = title+'.settings'
        else:    
            self.position(rect[0], rect[1])
            self.size(rect[2], rect[3])

#        self.settingsFile = title+'.settings'
#        if rect[0]!=None and rect[1]!=None:
#            self.position(rect[0], rect[1])
#        if rect[2]!=None and rect[3]!=None:
#            self.size(rect[2], rect[3])
            
        self.settings = settings
        self.callback(self.onClose)
    def show(self):
        if len(self.settingsFile)>0:
            self.settings.load(self.settingsFile)
            self.settings.setToApp(self)
        Fl_Window.show(self)
    def onClose(self, data):
        if len(self.settingsFile)>0:
            self.settings.getFromApp(self)
            self.settings.save(self.settingsFile)
        self.default_callback(self, data)
