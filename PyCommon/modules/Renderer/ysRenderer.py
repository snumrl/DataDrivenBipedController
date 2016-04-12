# +-------------------------------------------------------------------------
# | ysRenderer.py
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

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from OpenGL.GLE import *
import numpy, random

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mm
import Util.ysGlHelper as ygh
import Motion.ysMotion as ym

# RendererContext
NORMAL_FLAT = 0
NORMAL_SMOOTH = 1

# RendererContext, OdeModelRenderer, BoxesRenderer
POLYGON_LINE = 0
POLYGON_FILL = 1

# JointMotionRenderer
LINK_LINE = 0
LINK_BONE = 1
LINK_SOLIDBOX = 2
LINK_WIREBOX = 3

# RenderContext. PointsRenderer
POINT_POINT = 0
POINT_CROSS = 1
POINT_CUBE = 2 

#SELECTION_COLOR = (10,10,.7)
SELECTION_COLOR = (2550,2550,178)

RENDER_OBJECT = 0
RENDER_SHADOW = 1
RENDER_REFLECTION = 2

class Renderer:
    def __init__(self, target, color):
        self.rc = RenderContext()
        self.totalColor = color
        self.selectedElement = None
        self.shadowColor = (150,150,150)
    def render(self, renderType):
        print "Renderer.render() : Must subclass me"
        raise NotImplementedError
    
    
class SelectedGeomRenderer(Renderer):
    def __init__(self, color):
        Renderer.__init__(self, None, color = (255,0,0))
        self.geom = None
        self.rc.setPolygonStyle(POLYGON_LINE)
    def render(self):
        if self.geom:
            glColor3ubv(self.totalColor)
            self.rc.renderSelectedOdeGeom(self.geom, self.totalColor)

class OdeRenderer(Renderer):
    def __init__(self, target, color = (255,255,255)):
        Renderer.__init__(self, target, color)
        self.space = target
    def render(self):
        glColor3ubv(self.totalColor)
        for i in range(self.space.getNumGeoms()):
            geom = self.space.getGeom(i)
            
            if geom == self.selectedElement:
                glColor3ubv(SELECTION_COLOR)
                
            self.rc.renderOdeGeom(geom)
            
            if geom == self.selectedElement:
                glColor3ubv(self.totalColor)
            
class OdeModelRenderer(Renderer):
    def __init__(self, target, color = (255,255,255), polygonStyle = POLYGON_FILL):
        Renderer.__init__(self, target, color)
        self.model = target
        self.rc.setPolygonStyle(polygonStyle)
    def render(self):
        glColor3ubv(self.totalColor)
        for node in self.model.nodes.values():
            geom = node.geom
#            if node.name in self.partColors:
#                glColor3ubv(self.partColors[node.name])
#            else:
#                glColor3ubv(self.totalColor)
            if geom == self.selectedElement:
                glColor3ubv(SELECTION_COLOR)
                
            self.rc.renderOdeGeom(geom)

            if geom == self.selectedElement:
                glColor3ubv(self.totalColor)
            
class JointMotionRenderer(Renderer):
    def __init__(self, target, color = (0,255,255), linkStyle = LINK_LINE, lineWidth=1.):
        Renderer.__init__(self, target, color)
        self.motion = target
        self.renderFrames = None
        self.setLinkStyle(linkStyle)
        self.rc.setLineWidth(lineWidth)
    def setLinkStyle(self, linkStyle):
        self.linkStyle = linkStyle
        if self.linkStyle == LINK_WIREBOX:
            self.rc.setPolygonStyle(POLYGON_LINE)
        else:
            self.rc.setPolygonStyle(POLYGON_FILL)
    def render(self, renderType=RENDER_OBJECT):
        if len(self.motion) > 0:
            self.rc.beginDraw()
            if renderType==RENDER_SHADOW:
                glColor3ubv(self.shadowColor)
            else:
                glColor3ubv(self.totalColor)
            if self.renderFrames==None:
                posture = self.motion[self.motion.frame]
                self.renderJointPosture(posture)
            else:
                for renderFrame in self.renderFrames:
                    posture = self.motion[renderFrame]
                    self.renderJointPosture(posture)
    def renderJointPosture(self, posture):
        joint = posture.skeleton.root
        glPushMatrix()
        glTranslatef(posture.rootPos[0], posture.rootPos[1], posture.rootPos[2])
        self._renderJoint(joint, posture)
        glPopMatrix()
    def _renderJoint(self, joint, posture):
        glPushMatrix()
        glTranslatef(joint.offset[0],joint.offset[1],joint.offset[2])
#        glMultMatrixf(mm.R2T(posture.localRMap[joint.name]).transpose())
        glMultMatrixf(mm.R2T(posture.localRs[posture.skeleton.getElementIndex(joint.name)]).transpose())
                
#        if joint.name in self.partColors:
#            color = self.partColors[joint.name]
#        else:
#            color = self.totalColor
            
        if joint == self.selectedElement:
            glColor3ubv(SELECTION_COLOR)
            ygh.beginDraw()
            ygh.drawCoordinate()
            ygh.endDraw()
            
        # 1
#        ygh.drawPoint((0,0,0), color)

        if self.linkStyle == LINK_LINE:
            self.rc.drawPoint((0,0,0))
            for childJoint in joint.children:
                self.rc.drawLine((0,0,0), childJoint.offset)
                
        elif self.linkStyle == LINK_BONE:
#            self.rc.drawPoint((0,0,0))
            self.rc.drawLine((-.05,0,0), (.05,0,0))
            for childJoint in joint.children:
                self.rc.drawLine((0,0,0), childJoint.offset)
        
        elif self.linkStyle == LINK_SOLIDBOX or self.linkStyle == LINK_WIREBOX:        
            if len(joint.children) > 0:
                glPushMatrix()
                
                offset = numpy.array([0.,0.,0.])
                for childJoint in joint.children:
                    offset += childJoint.offset
                offset = offset/len(joint.children)
                
                defaultBoneV = numpy.array([0,0,mm.length(offset)])
                boneT = mm.R2T(mm.getSO3FromVectors(defaultBoneV, offset))
                glMultMatrixf(boneT.transpose())
        
                glTranslatef(-.05,-.05,0)
#                ygh.beginDraw()
#                ygh.drawCoordinate()
#                ygh.endDraw()
        
                self.rc.drawBox(.1,.1,mm.length(offset))
                glPopMatrix()
        
        if joint == self.selectedElement:
            glColor3ubv(self.totalColor)
        
        for childJoint in joint.children:
            self._renderJoint(childJoint, posture)
        glPopMatrix()
        
class PointMotionRenderer(Renderer):
    def __init__(self, target, color = (0,0,255)):
        Renderer.__init__(self, target, color)
        self.motion = target
    def render(self):
        glColor3ubv(self.totalColor)
        posture = self.motion[self.motion.frame]
        self.renderPointPosture(posture)
    def renderPointPosture(self, posture):
        for point in posture.positions:
#            if name in self.partColors:
#                glColor3ubv(self.partColors[name])
#            else:
#                glColor3ubv(self.totalColor)
            glPushMatrix()
            
            if point == self.selectedElement:
                glColor3ubv(SELECTION_COLOR)
                
            self.rc.drawPoint(point)

            if point == self.selectedElement:
                glColor3ubv(self.totalColor)
                
            glPopMatrix()

class MMMotionRenderer(Renderer):
    def __init__(self, target, color = (0,0,255)):
        Renderer.__init__(self, target, color)
        self.motion = target
    def render(self):
        glColor3ubv(self.totalColor)
        posture = self.motion[self.motion.frame]
        self.renderPointPosture(posture)
    def renderPointPosture(self, posture):
        for name, point in posture.pointMap.items():
#            if name in self.partColors:
#                glColor3ubv(self.partColors[name])
#            else:
#                glColor3ubv(self.totalColor)
            glPushMatrix()
            
            if point == self.selectedElement:
                glColor3ubv(SELECTION_COLOR)
                
            self.rc.drawPoint(point)

            if point == self.selectedElement:
                glColor3ubv(self.totalColor)
                
            glPopMatrix()
        for link in posture.skeleton.links:
            self.rc.drawLine(posture.pointMap[link[0]], posture.pointMap[link[1]])


class MeshRenderer(Renderer):
    def __init__(self, mesh, color = (127,127,127), drawStyle = POLYGON_LINE):
        Renderer.__init__(self, mesh, color)
        self.mesh = mesh
        self.rc.setPolygonStyle(drawStyle)
    def render(self):
        if isinstance(self.selectedElement, yms.Vertex):
            glColor3ubv(SELECTION_COLOR)
            self.rc.drawPoint(self.selectedElement.pos)
        
        pmid = None
        glPolygonMode(GL_FRONT, GL_LINE)
        glColor3ubv(self.totalColor)
        glBegin(GL_TRIANGLES)
        for f in self.mesh.faces:
            if f == self.selectedElement:
                glColor3ubv(SELECTION_COLOR)
                
            p0 = self.mesh.vertices[f.vertexIndex[0]].pos
            p1 = self.mesh.vertices[f.vertexIndex[1]].pos
            p2 = self.mesh.vertices[f.vertexIndex[2]].pos
            glVertex3f(p0[0], p0[1], p0[2])
            glVertex3f(p1[0], p1[1], p1[2])
            glVertex3f(p2[0], p2[1], p2[2])
            
            if f == self.selectedElement:
                pmid = (p0+p1+p2)/3.
                glColor3ubv(self.totalColor)
        glEnd()
        
        if pmid!=None:
            glColor3ubv(SELECTION_COLOR)            
            self.rc.drawPoint(pmid)
        
#===============================================================================
# # debugging renderers
#===============================================================================
class PointsRenderer(Renderer):
    def __init__(self, points, color = (255,0,0), pointStyle = POINT_CROSS):
        Renderer.__init__(self, points, color)
        self.points = points
        self.pointStyle = pointStyle
        self.rc.setLineWidth(2.)
    def render(self):
        self.rc.beginDraw()
        glColor3ubv(self.totalColor)
        for point in self.points:
            if point!=None:
                if self.pointStyle==POINT_POINT:
                    self.rc.drawPoint(point)
                elif self.pointStyle==POINT_CROSS:
                    self.rc.drawCross(point)
                elif self.pointStyle==POINT_CUBE:
                    self.rc.drawCube(point)

class VectorsRenderer(Renderer):
    def __init__(self, vectors, origins, color = (255,0,0)):
        Renderer.__init__(self, vectors, color)
        self.vectors = vectors
        self.origins = origins
    def render(self, renderType=RENDER_OBJECT):
        glColor3ubv(self.totalColor)
        for i in range(len(self.vectors)):
            if self.vectors[i] != None and self.origins[i] != None:
                origin = self.origins[i]; vector = self.vectors[i]
                self.rc.drawLine(origin, (origin[0]+vector[0],origin[1]+vector[1],origin[2]+vector[2]))
            
class PolygonRenderer(Renderer):
    def __init__(self, vertices, color = (0,255,0)):
        Renderer.__init__(self, vertices, color)
        self.points = vertices
        if len(self.points) == 3:
            self.polygonMode = GL_TRIANGLES
        else:
            self.polygonMode = GL_QUADS
    def render(self):
        glColor3ubv(self.totalColor)
        glDisable(GL_CULL_FACE)
        glPolygonMode(GL_FRONT, GL_FILL)
        glPolygonMode(GL_BACK, GL_LINE)
        glBegin(self.polygonMode)
        for v in self.points:
            glVertex3fv(v)
        glEnd()
        glEnable(GL_CULL_FACE)
        
class FramesRenderer(Renderer):
    def __init__(self, Ts, color = (0,255,0), axisLength = .5):
        Renderer.__init__(self, Ts, color)
        self.Ts = Ts
        self.axisLength = axisLength
    def render(self, renderType=RENDER_OBJECT):
        for T in self.Ts:
            if T!=None:
                glPushMatrix()
                glMultMatrixf(T.transpose())
                ygh.drawCoordinate(self.totalColor, self.axisLength)
                glPopMatrix()
        #        R, p = mm.T2Rp(self.T)
        #        axes = R.transpose()
        #        ygh.drawVector(axes[0], p, (255,0,0))
        #        ygh.drawVector(axes[1], p, (0,255,0))
        #        ygh.drawVector(axes[2], p, (0,0,255))

class OrientationsRenderer(Renderer):
    def __init__(self, Rs, ps, color = (0,255,0), axisLength = .5):
        Renderer.__init__(self, Rs, color)
        self.Rs = Rs
        self.ps = ps 
        self.axisLength = axisLength
    def render(self):
        for i in range(len(self.Rs)):
            if self.Rs[i]!=None and self.ps[i]!=None:
                T = mm.Rp2T(self.Rs[i], self.ps[i]) 
                glPushMatrix()
                glMultMatrixf(T.transpose())
                ygh.drawCoordinate(self.totalColor, self.axisLength)
                glPopMatrix()

class ForcesRenderer(Renderer):
    def __init__(self, forces, points, color=(255,0,0), ratio=1., lineWidth=.02, fromPoint=True):
        Renderer.__init__(self, None, color)
        self.forces = forces
        self.points = points
        self.ratio = ratio
        self.lineWidth = lineWidth
        self.fromPoint = fromPoint
        self.rc.setNormalStyle(NORMAL_SMOOTH)
    def render(self, renderType=RENDER_OBJECT):
        if renderType==RENDER_OBJECT:
            self.rc.beginDraw()
            glColor3ubv(self.totalColor)
            for i in range(len(self.forces)):
                if self.forces[i]!=None and self.points[i]!=None:
                    if self.fromPoint==False:
                        self.rc.drawArrow(None, self.points[i], mm.v3_scale(self.forces[i], self.ratio), self.lineWidth)
                    else:
                        self.rc.drawArrow(self.points[i], None, mm.v3_scale(self.forces[i], self.ratio), self.lineWidth)

class WideArrowRenderer(Renderer):
    def __init__(self, forces, points, color=(255,0,0), ratio=1., lineWidth=.02, heightRatio=.2, fromPoint=True, polygonStyle=POLYGON_FILL):
        Renderer.__init__(self, None, color)
        self.forces = forces
        self.points = points
        self.ratio = ratio
        self.lineWidth = lineWidth
        self.fromPoint = fromPoint
        self.heightRatio = heightRatio
        self.rc.setPolygonStyle(polygonStyle)
        self.rc.setNormalStyle(NORMAL_SMOOTH)
    def render(self, renderType=RENDER_OBJECT):
        if renderType==RENDER_OBJECT:
            self.rc.beginDraw()
            glColor3ubv(self.totalColor)
            for i in range(len(self.forces)):
                if self.forces[i]!=None and self.points[i]!=None:
                    glPushMatrix()
                    glScalef(1,self.heightRatio,1)
                    if self.fromPoint==False:
                        self.rc.drawArrow(None, self.points[i], mm.v3_scale(self.forces[i], self.ratio), self.lineWidth)
                    else:
                        self.rc.drawArrow(self.points[i], None, mm.v3_scale(self.forces[i], self.ratio), self.lineWidth)
                    glPopMatrix()

class TorquesRenderer(Renderer):
    def __init__(self, torques, points, color=(255,0,0), ratio=1., lineWidth=.02, radius=.1, fromPoint=True):
        Renderer.__init__(self, None, color)
        self.torques = torques
        self.points = points
        self.ratio = ratio
        self.radius = radius
        self.lineWidth = lineWidth
        self.fromPoint = fromPoint
        self.rc.setNormalStyle(NORMAL_SMOOTH)
    def render(self):
        self.rc.beginDraw()
        glColor3ubv(self.totalColor)
        for i in range(len(self.torques)):
            if self.torques[i]!=None and self.points[i]!=None:
                if self.fromPoint==False:
                    self.rc.drawCircularArrow(None, self.points[i], mm.v3_scale(self.torques[i], self.ratio), self.lineWidth, self.radius)
                else:
                    self.rc.drawCircularArrow(self.points[i], None, mm.v3_scale(self.torques[i], self.ratio), self.lineWidth, self.radius)

class VpBodyRenderer(Renderer):
    # boxSizes[i] = (lx, ly, lz)
    # Ts[i] = SE3
    def __init__(self, body, color=(255,0,0), polygonStyle=POLYGON_FILL, lineWidth=1.):
        Renderer.__init__(self, None, color)
        self.body = body
        self.rc.setPolygonStyle(polygonStyle)
        self.lineWidth = lineWidth
    def render(self, renderType=RENDER_OBJECT):
        self.rc.beginDraw()
        if renderType == RENDER_OBJECT:
            glColor3ubv(self.totalColor)
        else:
            glColor3ubv(self.shadowColor)
            
        boxSize = self.body.getShape(); T = self.body.getFrame()
        glPushMatrix()
        glMultMatrixf(T.T)
        glLineWidth(self.lineWidth)
        self.rc.drawBox(boxSize[0], boxSize[1], boxSize[2])
        glLineWidth(1.)
        glPopMatrix()
                
class VpBodiesRenderer(Renderer):
    # boxSizes[i] = (lx, ly, lz)
    # Ts[i] = SE3
    def __init__(self, bodies, color=(255,0,0), polygonStyle=POLYGON_FILL, lineWidth=1.):
        Renderer.__init__(self, None, color)
        self.bodies = bodies
        self.rc.setPolygonStyle(polygonStyle)
        self.lineWidth = lineWidth
        
#                randomSize = 100
#                variation = mm.v3(randomSize*random.random(), randomSize*random.random(), randomSize*random.random())
#                resultColor = variation + self.totalColor
#                for i in range(3):
#                    if resultColor[i]<0: resultColor[i]=0.
#                    elif resultColor[i]>255: resultColor[i]=255 
#                glColor3ubv(resultColor)
        
#        self.colors = [self.totalColor]*len(self.bodies)
        self.colors = [mm.s2v(self.totalColor)*(1-(float(i)/len(self.bodies) * .5)) for i in range(len(self.bodies))]
        
    def render(self, renderType=RENDER_OBJECT):
        self.rc.beginDraw()
            
        for i in range(len(self.bodies)):
            body = self.bodies[i]
            
            if renderType == RENDER_OBJECT:
                glColor3ubv(self.colors[i])
            else:
                glColor3ubv(self.shadowColor)
            
            boxSize = body.getShape(); T = body.getFrame()
            glPushMatrix()
            glMultMatrixf(T.T)
            glLineWidth(self.lineWidth)
            self.rc.drawBox(boxSize[0], boxSize[1], boxSize[2])
            glLineWidth(1.)
            glPopMatrix()
                
class MyFootRenderer(Renderer):
    def __init__(self, boxsize, T, color=(255,0,0), polygonStyle=POLYGON_FILL, lineWidth=1.):
        Renderer.__init__(self, None, color)
        self.boxsize = boxsize
        self.T = T
        self.lineWidth = lineWidth
    def render(self, renderType=RENDER_OBJECT):
        self.rc.beginDraw()
        if renderType == RENDER_OBJECT:
            glColor3ubv(self.totalColor)
        else:
            glColor3ubv(self.shadowColor)
        
        boxSize = self.boxsize; 
        glPushMatrix()
        glMultMatrixf(T.T)
        glLineWidth(self.lineWidth)
        self.rc.drawBox(boxSize[0], boxSize[1], boxSize[2])
        glLineWidth(1.)
        glPopMatrix()
                
class BoxesRenderer(Renderer):
    # boxSizes[i] = (lx, ly, lz)
    # Ts[i] = SE3
    def __init__(self, boxSizes, Ts, color=(255,0,0), polygonStyle=POLYGON_FILL, lineWidth=1.):
        Renderer.__init__(self, None, color)
        self.boxSizes = boxSizes
        self.Ts = Ts
        self.rc.setPolygonStyle(polygonStyle)
        self.lineWidth = lineWidth
    def render(self, renderType=RENDER_OBJECT):
        self.rc.beginDraw()
        if renderType == RENDER_OBJECT:
            glColor3ubv(self.totalColor)
        else:
            glColor3ubv(self.shadowColor)
        for i in range(len(self.boxSizes)):
            if self.boxSizes[i]!=None and self.Ts[i]!=None:
                boxSize = self.boxSizes[i]; T = self.Ts[i]
                glPushMatrix()
                glMultMatrixf(T.T)
                glLineWidth(self.lineWidth)
                self.rc.drawBox(boxSize[0], boxSize[1], boxSize[2])
                glLineWidth(1.)
                glPopMatrix()
                
class CylindersRenderer(Renderer):
    # cylinderSizes[i] = (radius, length_z)
    # Ts[i] = SE3
    def __init__(self, cylinderSizes, Ts, color=(255,0,0)):
        Renderer.__init__(self, None, color)
        self.cylinderSizes = cylinderSizes
        self.Ts = Ts
        self.rc.setNormalStyle(NORMAL_SMOOTH)
    def render(self, renderType=RENDER_OBJECT):
        self.rc.beginDraw()
        if renderType == RENDER_OBJECT:
            glColor3ubv(self.totalColor)
        else:
            glColor3ubv(self.shadowColor)
        for i in range(len(self.cylinderSizes)):
            if self.cylinderSizes[i]!=None and self.Ts[i]!=None:
                cylinderSize = self.cylinderSizes[i]; T = self.Ts[i]
                glPushMatrix()
                glMultMatrixf(T.T)
                self.rc.drawCylinder(cylinderSize[0], cylinderSize[1])
                glPopMatrix()
                
class SpheresRenderer(Renderer):
    # radius = radius of sphere
    # position = position of center of sphere
    def __init__(self, radiuses, positions, color=(255,0,0), polygonStyle = POLYGON_FILL):
        Renderer.__init__(self, None, color)
        self.radiuses = radiuses
        self.positions = positions
        self.rc.setNormalStyle(NORMAL_SMOOTH)
        self.rc.setPolygonStyle(polygonStyle)
        self.rc.setLineWidth(2.)
    def render(self, renderType=RENDER_OBJECT):
        self.rc.beginDraw()
        glColor3ubv(self.totalColor)
        for i in range(len(self.radiuses)):
            if self.radiuses[i]!=None and self.positions[i]!=None:
                rad = self.radiuses[i]; pos = self.positions[i]

#                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)   
                eqr=(0.0,-1.0, 0.0, 0.0001)
                glDisable(GL_CULL_FACE)
                glEnable(GL_CLIP_PLANE0)
                
#                glEnable(GL_COLOR_MATERIAL);
#                glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
#                glColorMaterial(GL_BACK, GL_DIFFUSE);
                
                glPushMatrix()
                glTranslatef(pos[0], pos[1], pos[2])
                glClipPlane(GL_CLIP_PLANE0, eqr)
                glRotatef(90, 1,0,0)
                self.rc.drawSphere(rad)
                glPopMatrix()
                
#                glDisable(GL_COLOR_MATERIAL);
                
                glDisable(GL_CLIP_PLANE0)
                glEnable(GL_CULL_FACE)
                
                
#===============================================================================
# # common class
#===============================================================================
class RenderContext:
    def __init__(self):
        self.quad = gluNewQuadric()
        gleSetNumSides(12)
        
        self.setPolygonStyle(POLYGON_FILL)
        self.setNormalStyle(NORMAL_FLAT)
        self.setLineWidth(1.)
        self.crossLength = .1
        
    def __del__(self):
        gluDeleteQuadric(self.quad)
        
    def setPolygonStyle(self, polygonStyle):
        self.polygonStyle = polygonStyle
        if polygonStyle == POLYGON_LINE:
            gluQuadricDrawStyle(self.quad, GLU_LINE)
        elif polygonStyle == POLYGON_FILL:
            gluQuadricDrawStyle(self.quad, GLU_FILL)
            
    def setNormalStyle(self, normalStyle):
        self.normalStyle = normalStyle
        if normalStyle == NORMAL_FLAT:
            gluQuadricDrawStyle(self.quad, GLU_FLAT)
        elif normalStyle == NORMAL_SMOOTH:
            gluQuadricDrawStyle(self.quad, GLU_SMOOTH)
            
    def setLineWidth(self, lineWidth):
        self.lineWidth = lineWidth
            
    def beginDraw(self):
        if self.polygonStyle == POLYGON_LINE:
            glPolygonMode(GL_FRONT, GL_LINE)
        elif self.polygonStyle == POLYGON_FILL:
            glPolygonMode(GL_FRONT, GL_FILL)
        
        if self.normalStyle == NORMAL_FLAT:
            gleSetJoinStyle(TUBE_NORM_FACET | TUBE_JN_CAP | TUBE_JN_CUT)
        elif self.normalStyle == NORMAL_SMOOTH:
            gleSetJoinStyle(TUBE_NORM_EDGE | TUBE_JN_CAP | TUBE_JN_CUT)
            
        glLineWidth(self.lineWidth)
            
    #===============================================================================
    # draw primitives at origin    
    #===============================================================================
    def drawBox(self, lx, ly, lz):
        glPushMatrix()
#        glTranslated(lx/2.,ly/2.,lz/2.)
        glScale(lx, ly, lz)
        if self.polygonStyle == POLYGON_LINE:
            glutWireCube(1)
        else:
            glutSolidCube(1)
        glPopMatrix()
    def drawCylinder(self, radius, length_z):
#        gluCylinder(self.quad, radius, radius, length_z, 16, 1)
        gleSetNumSides(20)
        glePolyCylinder(((0,0,-length_z/2.), (0,0,-length_z/2.), (0,0,length_z/2.), (0,0,length_z/2.)), None, radius)
        gleSetNumSides(12)

    def drawSphere(self, radius):
        SLICE = 20; STACK = 20
        if self.polygonStyle == POLYGON_LINE:
            glutWireSphere(radius, SLICE, STACK)
        else:
            glutSolidSphere(radius, SLICE, STACK)

    #===============================================================================
    # draw primitives at its position        
    #===============================================================================
    def drawPoint(self, point):
        glPointSize(3.0)
        glBegin(GL_POINTS)
        glVertex3fv(point)
        glEnd()
        
    def drawCross(self, point):
        glPushMatrix()
        glTranslatef(point[0], point[1], point[2])
        glBegin(GL_LINES)
        crossLength = self.crossLength
        glVertex3f(crossLength/2.,0,0)    # x
        glVertex3f(-crossLength/2.,0,0)
        glVertex3f(0,crossLength/2.,0)    # y
        glVertex3f(0,-crossLength/2.,0)
        glVertex3f(0,0,crossLength/2.)    # z
        glVertex3f(0,0,-crossLength/2.)
        glEnd()
        glPopMatrix()
        
    def drawCube(self, point):
        ygh.beginDraw()
        ygh.drawPoint(point)
        ygh.endDraw()
        glPushMatrix()
        glTranslated(point[0], point[1], point[2])
        glutWireCube(.1)
        glPopMatrix()
            
    def drawLine(self, startPos, endPos):
        glBegin(GL_LINES)
#        glVertex3fv(startPos)
#        glVertex3fv(endPos)
        glVertex3f(startPos[0], startPos[1], startPos[2])
        glVertex3f(endPos[0], endPos[1], endPos[2])
        glEnd()
        
    def draw2DArrow(self, startPos, endPos, vector=None, lineWidth=.02):
        if vector==None:
            vector = [endPos[i]-startPos[i] for i in range(3)]
        elif startPos==None:
            startPos = [endPos[i]-vector[i] for i in range(3)]
        
#        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        
        glDisable(GL_CULL_FACE)
        glPushMatrix()
        
        length = mm.length(vector)
        arrowT = mm.Rp2T(mm.getSO3FromVectors((length,0,0), vector), startPos)
        glMultMatrixf(arrowT.transpose())
        
        triWidth = lineWidth * 3
        triLength = triWidth * 1.2
        
        angles = [0, 90]
        for angle in angles:
            glRotatef(angle, 1,0,0)
        
            # line part
            glBegin(GL_QUADS)        
            glVertex3f(0,0,lineWidth/2)
            glVertex3f(0,0,-lineWidth/2)
            glVertex3f(length - triLength,0,-lineWidth/2)
            glVertex3f(length - triLength,0,+lineWidth/2)
            glEnd()
            
            # triangle part
            glBegin(GL_TRIANGLES)
            glVertex3f(length - triLength, 0, triWidth/2)
            glVertex3f(length - triLength, 0, -triWidth/2)
            glVertex3f(length, 0, 0)
            glEnd()
    
        glPopMatrix()
        glEnable(GL_CULL_FACE)
        
    def drawArrow(self, startPos, endPos, vector=None, lineWidth=.02):
        if vector==None:
            vector = [endPos[i]-startPos[i] for i in range(3)]
        elif startPos==None:
            startPos = [endPos[i]-vector[i] for i in range(3)]
        
        length = mm.length(vector)
        if length==0.: return

        glPushMatrix()
        
        arrowT = mm.Rp2T(mm.getSO3FromVectors((length,0,0), vector), startPos)
        glMultMatrixf(arrowT.transpose())
        
        triWidth = lineWidth * 3
        triLength = triWidth * 1.2
        
        # line + cone all parts
        glePolyCone(((0,0,0), (0,0,0), (length-triLength,0,0), (length-triLength,0,0), (length,0,0), (length,0,0)), None, 
                    (lineWidth/2., lineWidth/2., lineWidth/2., triWidth/2., 0, 0))
        
        glPopMatrix()
    
    def drawCircularArrow(self, startPos, endPos, rotVec=None, lineWidth=.02, radius=.1):
        if rotVec==None:
            rotVec = [endPos[i]-startPos[i] for i in range(3)]
        elif startPos==None:
            startPos = [endPos[i]-rotVec[i] for i in range(3)]
        
        length = mm.length(rotVec)
        if length==0.: return

        glPushMatrix()
        
        axisT = mm.Rp2T(mm.getSO3FromVectors((0,0,length), rotVec), startPos)
        glMultMatrixf(axisT.transpose())
        
        triWidth = lineWidth * 3
        triLength = triWidth * 1.2
        
        # axis
#        self.drawLine((0,0,0), (0,0,length))
        glePolyCylinder(((0,0,0), (0,0,0), (0,0,length), (0,0,length)), None, lineWidth/4.)
        
        # circular line part
#        gleHelicoid( rToroid , startRadius , drdTheta , startZ , dzdTheta , 
#                     startXform , dXformdTheta , startTheta , sweepTheta )
        sweepTheta = 2*math.pi*length*mm.DEG
        gleHelicoid( lineWidth/2., radius,       0.,        0.,    radius,
                        None,         None,            0.,     sweepTheta)
        
        # cone part
        glPushMatrix()
        glRotatef(sweepTheta, 0,0,1)
        glTranslatef(radius, 0, radius * (sweepTheta/360.))
        glRotatef(-90, 1,0,0)
        glePolyCone(((0,0,0), (0,0,0), (0,0,triLength), (0,0,triLength)), None, 
                    (triWidth/2., triWidth/2., 0, 0))
        glPopMatrix()
        
        glPopMatrix()
    
    def renderSelectedOdeGeom(self, geom, color):
        if type(geom) == ode.GeomBox:
            lx, ly, lz = geom.getLengths()
            x,y,z = geom.getPosition()
            R = geom.getRotation()
            se3 = [R[0], R[3], R[6], 0.,
                   R[1], R[4], R[7], 0.,
                   R[2], R[5], R[8], 0.,
                   x, y, z, 1.0]
            glPushMatrix()
            glMultMatrixd(se3)
            ygh.drawCoordinate(color)
            glScaled(1.1,1.1,1.1)
            glTranslated(-lx/2.,-ly/2.,-lz/2.)
            self.drawBox(lx, ly, lz)
            glPopMatrix()
            
    def renderOdeGeom(self, geom):
        if type(geom) == ode.GeomBox:
            lx, ly, lz = geom.getLengths()
            x,y,z = geom.getPosition()
            R = geom.getRotation()
            se3 = [R[0], R[3], R[6], 0.,
                   R[1], R[4], R[7], 0.,
                   R[2], R[5], R[8], 0.,
                   x, y, z, 1.0]
            glPushMatrix()
            glMultMatrixd(se3)
            glTranslated(-lx/2.,-ly/2.,-lz/2.)
            self.drawBox(lx, ly, lz)
            glPopMatrix()
            
        elif type(geom) == ode.GeomCapsule:
            radius, length_z = geom.getParams()
            
            x,y,z = geom.getPosition()
            R = geom.getRotation()
            se3 = [R[0], R[3], R[6], 0.,
                   R[1], R[4], R[7], 0.,
                   R[2], R[5], R[8], 0.,
                   x, y, z, 1.0]
            glPushMatrix()
            glMultMatrixd(se3)
            glTranslated(0,0,-length_z/2.)
            self.drawCylinder(radius, length_z)
            glPopMatrix()
    
        elif type(geom) == ode.GeomSphere:
            radius = geom.getRadius()
        
        elif type(geom) == ode.GeomPlane:
            (a, b, c), d = geom.getParams()
            glPushMatrix()
            glTranslatef(0,d,0)
            glScale(10,0,10)
            glutWireCube(1)
            glPopMatrix()
            
        elif type(geom) == ode.GeomRay:
            length = geom.getLength()
            

if __name__=='__main__':
    import psyco; psyco.full()
    from fltk import *
    import math
    import Resource.ysOgreDataLoader as yol
    import GUI.ysSimpleViewer as ysv
    import Resource.ysMotionLoader as yf
    
    def test_MeshRenderer():
        meshFilePath = '../samples/woody2_15.mesh.xml'
        mesh = yol.readOgreMeshFileAsMesh(meshFilePath)

        viewer = ysv.SimpleViewer()
        viewer.doc.addRenderer('mesh', MeshRenderer(mesh))
        viewer.doc.addObject('mesh', mesh)
        
        viewer.startTimer(1./30.)
        viewer.show()
        
        Fl.run()
        
    def test_FramesRenderer_OrientationsRenderer():
        frame0 = mm.I_SE3()
        frame1 = mm.Rp2T(mm.exp(mm.v3(0,1,0), math.pi/8.), (1,0,0))
        
        viewer = ysv.SimpleViewer()
        viewer.doc.addRenderer('frame0', FramesRenderer([frame0], (255,0,0)))
        viewer.doc.addRenderer('frame1', FramesRenderer([frame1], (255,0,0)))
        viewer.doc.addRenderer('orientation0', OrientationsRenderer([mm.T2R(frame0)], [mm.T2p(frame0)], (0,255,0)))
        viewer.doc.addRenderer('orientation1', OrientationsRenderer([mm.T2R(frame1)], [mm.T2p(frame1)], (0,255,0)))
        
        viewer.show()
        Fl.run()
        
    def test_ForcesRenderer_TorquesRenderer():
        forces = [None]*5
        points1 = [None]*5
        torques = [None]*5
        points2 = [None]*5
        
        for i in range(len(forces)):
            forces[i] = (0,i,0)
            points1[i] = (i,0,0)
            
        for i in range(len(torques)):
            points2[i] = (-i,0,0)
            torques[i] = (0,0,i)

        viewer = ysv.SimpleViewer()
        viewer.doc.addRenderer('forces', ForcesRenderer(forces, points1, (255,0,0), 1., .1))
        viewer.doc.addRenderer('torques', TorquesRenderer(torques, points2, (0,255,0), 1., .1, .3))

        viewer.show()
        Fl.run()
         
    def test_primitives_renderers():
        boxSize = [1,1,1]
        boxFrame = mm.I_SE3()
        
        cylinderSize = [1,1]
        cylinderFrame = mm.I_SE3()
        
        sphereRadius = 1.
        spherePosition = (0,0.5,0)
        
        arrowVector = (1,0,0)
        arrowPoint = (0,0,0)
        
        viewer = ysv.SimpleViewer()
#        viewer.doc.addRenderer('box', BoxesRenderer([boxSize], [boxFrame], (255,0,0)))
#        viewer.doc.addRenderer('cylinder', CylindersRenderer([cylinderSize], [cylinderFrame], (0,0,255)))
#        viewer.doc.addRenderer('sphere', SpheresRenderer([sphereRadius], [spherePosition], (0,0,255)))
        viewer.doc.addRenderer('sphere', WideArrowRenderer([arrowVector], [arrowPoint], (255,0,0), 1., .1))

        viewer.show()
        Fl.run()
         

    pass
#    test_MeshRenderer()
#    test_FramesRenderer_OrientationsRenderer()
#    test_ForcesRenderer_TorquesRenderer()
    test_primitives_renderers()
