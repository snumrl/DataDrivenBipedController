# +-------------------------------------------------------------------------
# | ysPhysConfig.py
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

import sys, numpy, math, copy

class WorldConfig:
    def __init__(self):
        self.timeStep = 0.001
        
        self.gravity = (0, -9.8, 0)
        self.planeHeight = 0.0
        self.useDefaultContactModel = True
        self.lockingVel = 0.05
        
        ##########################################33
        # for ODE
        # below values are ODE default values
        self.ContactSurfaceLayer = 0.0
        self.ContactMaxCorrectingVel = -1000000
        self.ERP = 0.2
        self.CFM = 1E-5
        ##########################################33

class Node:
    def __init__(self, name):
        self.name = name
        self.mass = None
        self.offset = (0,0,0)
        self.length = None
        self.width = None
        self.geom = 'MyBox'
        
        self.jointAxes = []
        self.jointLoStop = -1000000
        self.jointHiStop = 1000000
        
        self.density = 1000 # 1000 kg/m^3 = 1 g/cm^3 : density of liquid water at 4'C
        self.boneRatio = 1
        self.Kp = 100
        self.Kd = 5
        
        ##########################################
        # for ODE
        # below values are ODE default values
        self.contactMode = None
        self.contactMu = 1
        self.contactBounce = 0.1
        self.contactSoftERP = 0.0
        self.contactSoftCFM = 0.0
        ##########################################
    def __str__(self):
        return self.name + ' length : %f\n'%self.length    
class ModelConfig:
#    class Node:
#        def __init__(self, name):
#            self.name = name
#            self.mass = None
#            self.offset = (0,0,0)
#            self.length = None
#            self.width = None
#            self.geom = 'MyBox'
#            
#            self.jointAxes = []
#            self.jointLoStop = -ode.Infinity
#            self.jointHiStop = ode.Infinity
#            
#            self.density = 1000 # 1000 kg/m^3 = 1 g/cm^3 : density of liquid water at 4'C
#            self.boneRatio = 1
#            self.Kp = 100
#            self.Kd = 5
#            
#            ##########################################
#            # for ODE
#            # below values are ODE default values
#            self.contactMode = ode.ContactBounce
#            self.contactMu = ode.Infinity
#            self.contactBounce = 0.1
#            self.contactSoftERP = 0.0
#            self.contactSoftCFM = 0.0
#            ##########################################
            
    def __init__(self):
        self.nodes = {}
#        tempNode = ModelConfig.Node('')
        tempNode = Node('')
        self.defaultDensity = tempNode.density
        self.defaultBoneRatio = tempNode.boneRatio
        self.defaultKp = tempNode.Kp
        self.defaultKd = tempNode.Kd
        self.defaultJointAxes = tempNode.jointAxes
        self.defaultJointLoStop = tempNode.jointLoStop
        self.defaultJointHiStop = tempNode.jointHiStop
        self.defaultContactMode = tempNode.contactMode
        self.defaultContactMu = tempNode.contactMu
        self.defaultContactBounce = tempNode.contactBounce
        self.defaultContactSoftERP = tempNode.contactSoftERP
        self.defaultContactSoftCFM = tempNode.contactSoftCFM
    def addNode(self, name):
#        node = ModelConfig.Node(name)
        node = Node(name)
        node.density = self.defaultDensity
        node.boneRatio = self.defaultBoneRatio
        node.Kp = self.defaultKp
        node.Kd = self.defaultKd
        node.jointAxes = self.defaultJointAxes
        node.jointLoStop = self.defaultJointLoStop
        node.jointHiStop = self.defaultJointHiStop
        node.contactMode = self.defaultContactMode
        node.contactMu = self.defaultContactMu
        node.contactBounce = self.defaultContactBounce
        node.contactSoftERP = self.defaultContactSoftERP
        node.contactSoftCFM = self.defaultContactSoftCFM
        self.nodes[name] = node
        return node
    def getNode(self, name):
        return self.nodes[name]
    def hasNode(self, name):
        return name in self.nodes
    def setNodeCopy(self, name, node):
        nodeCopy = copy.deepcopy(node)
        nodeCopy.name = name 
        self.nodes[name] = nodeCopy
    def delNode(self, name):
        del self.nodes[name]
    def adjustTotalMass(self, totalMass):
        oldTotalMass = 0.
        for node in self.nodes.values():
            oldTotalMass += node.mass
        for node in self.nodes.values():
            node.mass *= (totalMass / oldTotalMass)
