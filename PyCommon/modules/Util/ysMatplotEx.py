# +-------------------------------------------------------------------------
# | ysMatplotEx.py
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

import matplotlib
#matplotlib.interactive(True)
matplotlib.use('FltkAgg')

from pylab import *
from matplotlib.widgets import*
from matplotlib import collections
from matplotlib.colors import colorConverter

import sys
if '..' not in sys.path:
    sys.path.append('..')
import GUI.ysBaseUI as ybu

class BasePlot:
    def __init__(self, title):
        self.title = title
        self.settings = ybu.BaseSettings()
    def loadSetting(self, window):
        # window position load
        self.settingsFile = self.title+'.settings'
        self.settings.load(self.settingsFile)
        self.settings.setToApp(window)
    def registerCallbacks(self, figure, window):
        self.window = window
        figure.canvas.mpl_connect('key_press_event', self.onKeyPress)
        window.callback(self.onClose)        
    # key event handling
    def onKeyPress(self, event):
        INC = 10
        if event.key == 'q':
            self.window.do_callback()
        elif event.key == 't':
            self.toggle()
        elif event.key == 'a':
            self.window.size(self.window.w()+INC, self.window.h())
        elif event.key == 's':
            self.window.size(self.window.w()-INC, self.window.h())
        elif event.key == 'd':
            self.window.size(self.window.w(), self.window.h()+INC)
        elif event.key == 'f':
            self.window.size(self.window.w(), self.window.h()-INC)
    # close event handling
    def onClose(self, widget, data):
        # window position save
        self.settings.getFromApp(self.window)
        self.settings.save(self.settingsFile)
        self.window.default_callback(widget, None)
    def toggle(self):
        pass
    def close(self):
        self.window.do_callback()

class InteractivePlot(BasePlot):
    def __init__(self, title='InteractivePlot'):
        BasePlot.__init__(self, title)
        self.dataSetMap = {}
        self.dataSetColorMap = {}
        self.newDataPoint = {}
        
        self.dataSetNames = []
        self.colors = ['b','g','r','c','m','y']
        self.colorIndex = -1

        self.fig = figure()
        self.fig.subplots_adjust(0.2, 0.05, .95, .95)   # left, bottom, right, top
        self.ax = self.fig.add_subplot(111)
        
        self.ly_col = collections.LineCollection([[(0,0),(0,0)]])
        self.ly_col.set_color(['k'])
        self.ax.add_collection(self.ly_col)
                
        self.window = get_current_fig_manager().window
        self.loadSetting(self.window)
        self.registerCallbacks(self.fig, self.window)
        
        self.drawing = True
    def addDataSet(self, name):
        self.dataSetNames.append(name)
        self.dataSetMap[name] = []
        self.dataSetColorMap[name] = self.getNextColor()
        
        i = len(self.dataSetNames)-1
        self.fig.text(0.02, .85-i*.07, name, color=self.dataSetColorMap[name], bbox=dict(facecolor='w', alpha=.5))
    def addDataPoint(self, name, x, y):
        self.newDataPoint[name] = (x, y)
    def updatePoints(self):
        segments = []
        colors = []
        for name, point in self.newDataPoint.items():
            if len(self.dataSetMap[name])>0:
                prevPoint = self.dataSetMap[name][-1]
                segments.append([prevPoint, point])
                colors.append(self.dataSetColorMap[name])
            self.dataSetMap[name].append(point)
        if len(segments)>0:
            col = collections.LineCollection(segments)
            col.set_color(colors)
            self.ax.add_collection(col)
            self.ax.draw_artist(col)

#            x = segments[0][1][0]
#            miny, maxy = self.ax.get_ylim()
#            self.ly_col.set_segments([[(x, miny), (x, maxy)]])
#            self.ax.draw_artist(self.ly_col)
            
            if self.drawing:
#                draw()
#                self.ax.redraw_in_frame()
                self.window.redraw()
        self.newDataPoint.clear()
    def updateVline(self, x):
        if self.drawing:
            miny, maxy = self.ax.get_ylim()
            self.ly_col.set_segments([[(x, miny), (x, maxy)]])
            self.ax.draw_artist(self.ly_col)
            self.ax.redraw_in_frame()
            self.window.redraw()
    def setXlimit(self, xmin, xmax):
        self.ax.set_xlim(xmin, xmax)
    def setYlimit(self, ymin, ymax):
        self.ax.set_ylim(ymin, ymax)
    def getNextColor(self):
        self.colorIndex += 1
        if self.colorIndex > len(self.colors)-1:
            self.colorIndex = 0
        return self.colors[self.colorIndex]
    def toggle(self):
        self.drawing = not self.drawing
        
class SmartPlot(BasePlot):
    def __init__(self, title='SmartPlot'):
        BasePlot.__init__(self, title)
        self.xDataName = ''
        self.xData = []
        self.yDatas = {}
        self.xLines = {}
        self.yLines = {}
        self.xSpans= {}
        self.ySpans = {}
        self.checkStates = {}
        self.colorStates = {}
        
        self.names = []
        self.colors = ['b','g','r','c','m','y','k']
        self.colorIndex = -1
        
        self.xLimit = None
        self.yLimit = None

        self.lineStyle = '+-'
        self.lineWidth = 1.
        
    def getNextColor(self):
        self.colorIndex += 1
        if self.colorIndex > len(self.colors)-1:
            self.colorIndex = 0
        return self.colors[self.colorIndex]
    def setXlimit(self, xmin, xmax):
        self.xLimit = (xmin, xmax)
    def setYlimit(self, ymin, ymax):
        self.yLimit = (ymin, ymax)
    def setXdata(self, name, list):
        self.xDataName = name
        self.xData = list
        
#===============================================================================
# character    color
# 'b'    blue
# 'g'    green
# 'r'    red
# 'c'    cyan
# 'm'    magenta
# 'y'    yellow
# 'k'    black
# 'w'    white
#
# character    description
# '-'    solid line style
# '--'    dashed line style
# '-.'    dash-dot line style
# ':'    dotted line style
# '.'    point marker
# ','    pixel marker
# 'o'    circle marker
# 'v'    triangle_down marker
# '^'    triangle_up marker
# '<'    triangle_left marker
# '>'    triangle_right marker
# '1'    tri_down marker
# '2'    tri_up marker
# '3'    tri_left marker
# '4'    tri_right marker
# 's'    square marker
# 'p'    pentagon marker
# '*'    star marker
# 'h'    hexagon1 marker
# 'H'    hexagon2 marker
# '+'    plus marker
# 'x'    x marker
# 'D'    diamond marker
# 'd'    thin_diamond marker
# '|'    vline marker
# '_'    hline marker        
#===============================================================================
    def setLineStyle(self, lineStyle):
        self.lineStyle = lineStyle
    def setLineWidth(self, lineWidth):
        self.lineWidth = lineWidth

    def addYdata(self, name, list, check=True, color=None):
        self.yDatas[name] = list
        self.addName(name, check, color)
    def addXlines(self, name, list, check=True, color=None):
        name = 'x='+name
        self.xLines[name] = list
        self.addName(name, check, color)
    def addYlines(self, name, list, check=True, color=None):
        name = 'y='+name
        self.yLines[name] = list
        self.addName(name, check, color)
    def addXspans(self, name, list, check=True, color=None):
        name = 'x=['+name+']'
        self.xSpans[name] = list
        self.addName(name, check, color)
    def addYspans(self, name, list, check=True, color=None):
        name = 'y=['+name+']'
        self.ySpans[name] = list
        self.addName(name, check, color)
    def addName(self, name, check, color=None):
        self.checkStates[name] = check
        if color==None:
            self.colorStates[name] = self.getNextColor()
        else:
            self.colorStates[name] = color
        self.names.append(name) 
    def showModeless(self):
        fig = figure(1)
        ax = axes([0.025, 0.5-len(self.checkStates)*.05*.5, 0.14, len(self.checkStates)*.05])
        self.yBtns = CheckButtons(ax, self.names, [self.checkStates[name] for name in self.names])
        self.yBtns.on_clicked(self.onItemselected)
        axes([0.2, 0.1, 0.75, 0.85])
        self.plot()
        
        manager = get_current_fig_manager()
        self.loadSetting(manager.window)
        self.registerCallbacks(fig, manager.window)
    def show(self):
        self.showModeless()
        show()
    def onItemselected(self, name):
        self.checkStates[name] = not self.checkStates[name]
        self.plot()
        draw()
    def plot(self):
        cla()
        self.plotYdata()
        self.plotXlines()
        self.plotYlines()
        self.plotXspans()
        self.plotYspans()
        xlabel(self.xDataName)
        grid(True)
        
        if self.xLimit!=None:
            xlim(self.xLimit[0], self.xLimit[1])
        else:
            xlim(self.xData[0], self.xData[-1])
            
        if self.yLimit!=None:
            ylim(self.yLimit[0], self.yLimit[1])
    def plotYdata(self):
        for name, data in self.yDatas.items():
            if self.checkStates[name]:
                plot(self.xData, data, self.lineStyle + self.colorStates[name], linewidth=self.lineWidth)
    def plotXlines(self):
        for name, xLines in self.xLines.items():
            if self.checkStates[name]:
                for x in xLines:
                    axvline(x, 0, 1, color = self.colorStates[name], linewidth=self.lineWidth)
    def plotYlines(self):
        for name, yLines in self.yLines.items():
            if self.checkStates[name]:
                for y in yLines:
                    axhline(y, 0, 1, color = self.colorStates[name], linewidth=self.lineWidth)
    def plotXspans(self):
        for name, xSpans in self.xSpans.items():
            if self.checkStates[name]:
                for x in xSpans:
                    axvspan(x[0], x[1], facecolor = self.colorStates[name], alpha = .1)
    def plotYspans(self):
        for name, ySpans in self.ySpans.items():
            if self.checkStates[name]:
                for y in ySpans:
                    axhspan(y[0], y[1], facecolor = self.colorStates[name], alpha = .1)
    

if __name__ == "__main__":
    import psyco; psyco.full()
    from fltk import *
    import time
    import Math.mmMath as mmMath
    import Resource.ysMotionLoader as yf
    import Renderer.ysRenderer as yr
    import GUI.ysSimpleViewer as ysv
  
    def test_matplot():
        t = arange(0.0, 2.0, 0.01)
        s = sin(2*pi*t)
        plot(t, s, linewidth=1.0)
        axvspan(0, .5, facecolor = 'r', alpha = .5)
        
        xlabel('time (s)')
        ylabel('voltage (mV)')
        title('About as simple as it gets, folks')
        grid(True)
        show()
    
    def test_matplot2():
        figure(1)
        subplot(221)
        plot([1,2,3], [2,4,5], '-b')
        plot([1,2,3], [2,4,5], '-r')
        xlabel('frame')
        ylabel('y')
        title('LeftFoot')
        grid(True)    
    
        subplot(222)
        plot([1,2,3], [2,4,5], '-b')
        plot([1,2,3], [2,4,5], '-r')
        xlabel('frame')
        ylabel('y')
        title('LeftFootEnd')
        grid(True)    
        
        subplot(223)
        plot([1,2,3], [2,4,5], '-b')
        plot([1,2,3], [2,4,5], '-r')
        xlabel('frame')
        ylabel('y')
        title('RightFoot')
        grid(True)    
        
        subplot(224)
        plot([1,2,3], [2,4,5], '-b')
        plot([1,2,3], [2,4,5], '-r')
        xlabel('frame')
        ylabel('y')
        title('RightFootEnd')
        grid(True)    
    
#        figure(2)
#        plot([1,2,3], tm.contactLeft, '+-b')
#        plot([1,2,3], tm.contactRight, '+-r')
#        xlabel('frame')
#        grid(True)    
#        cid = connect('button_press_event', Sclick)
        
        show()
    
    def test_SmartPlot():
        lPos = [0.06042668581384758, 0.060426910692448099, 0.06042812774832107, 0.06041833813844083, 0.060409130795800958, 0.060402730203184929, 0.060400557242888397, 0.060400987981923615, 0.060404348975209132, 0.060415212046884048, 0.060430352849700442, 0.060448986149254869, 0.060461257925718992, 0.060455973016620979, 0.060434928721486481, 0.060423231053146287, 0.060430838187995106, 0.06043645703701106, 0.060413737223606379, 0.060389155470753741, 0.060394340780917877, 0.060407353709821088, 0.060400570783128549, 0.060399249894219076, 0.060389759152503109, 0.060362607769103627, 0.060382595532041061, 0.060584802372489033, 0.060988563218939917, 0.061391691560062855, 0.061562143649315337, 0.06154599442797426, 0.061507141094831186, 0.061327796622075381, 0.060741264519832638, 0.059737212572460729, 0.058692867229799495, 0.058105463746687458, 0.057861815755900436, 0.057755646257795556, 0.057817244289792069, 0.058038381688855112, 0.05828313869289431, 0.058490544692743784, 0.05872011978019398, 0.058993214699919971, 0.059428534442005576, 0.0602091920492141, 0.061118982611583217, 0.061811059642059962, 0.062799859198514052, 0.065192056115661534, 0.069635822164357375, 0.076835348636301148, 0.08929256011032094, 0.11120768032452205, 0.14185593417255576, 0.17490236783710966, 0.20657812009809595, 0.23595407621115588, 0.25850563356301082, 0.27085425196507584, 0.27357224590290435, 0.26824841948291156, 0.25560924752255654, 0.23649837517128702, 0.21245097066154234, 0.18541165342230614, 0.15795112755978025, 0.13367486760712727, 0.11568701252957858, 0.1020932327668681, 0.087941002284554592, 0.072359611949009417, 0.060240482461257572, 0.055170504441999446, 0.054891602216408764, 0.055040404645954943, 0.054498859198988669, 0.054508509632959279, 0.054960169847387275, 0.05477931202597025, 0.054154112926752596, 0.05399130633282101, 0.054211244178527573, 0.054305971518433804, 0.054404275862807716, 0.054658741282728851, 0.054879509755403233, 0.055089769432891067, 0.055520377437395574, 0.056610454126351073, 0.059492416474343723, 0.065389051612351923, 0.074837219085816642, 0.089561519515354682, 0.11237825490620734, 0.14360757504211108, 0.17735286787710075, 0.20867814266776924, 0.23449940103184969, 0.24915177950832679, 0.25070878686658343, 0.24158424160067171, 0.2239909452160514, 0.20002790437724854, 0.17320487806925938, 0.14689280601090743, 0.12376264424552447, 0.10654255560178977, 0.097381690097234186, 0.093617501269765979, 0.088005802666424104, 0.076248024518592417, 0.062163775652249065, 0.052648992279146656, 0.049486251503779, 0.049367399935425271, 0.049023450710785066, 0.048567360131823056, 0.048904138021897414, 0.049329893090900523, 0.049085214012601353, 0.048705635570608974, 0.048656257896886212, 0.048592042106038791, 0.048521219277970207, 0.048881909431310655, 0.049498409162796797, 0.050036239504921642, 0.0508913725272786, 0.052563060192389099, 0.05534040278278235, 0.059955284192090952, 0.067280719455158067, 0.077835524852820526, 0.092929688019458834, 0.11495554061708774, 0.14358308708505269, 0.17308507964989389, 0.19964036825592119, 0.22133557211814747, 0.23377141311500438, 0.23535863414600214, 0.22808685920227423, 0.21388685437947808, 0.19459399352942625, 0.17275990060621971, 0.1502729024856046, 0.12832755698691128, 0.10810720616625946, 0.089776262822080732, 0.072946823381329906, 0.059114193475088728, 0.050890534847101931, 0.048472320189769547, 0.04926928318423307, 0.049854045199226038, 0.049607584899326318, 0.049259196159864793, 0.049037158425249527, 0.048814017726756698, 0.048579732400854503, 0.048482596167557512, 0.048438797482656559, 0.048402147438674936, 0.048492380470799734, 0.048630726240059818, 0.048585009982071492, 0.048425362886750922, 0.048384517224370482, 0.048455912486249197, 0.048431868567694214, 0.048321530006935232, 0.048307600600068279, 0.048398827483847606, 0.048466157218539752, 0.048507324032563137, 0.048600177997309979, 0.048697896554017406, 0.048716055958951565, 0.048720372515585142, 0.048796350658621757, 0.048890433290046831, 0.048952897385445782, 0.049117109191876673, 0.049523199780851956, 0.050082777992687233, 0.050717017113361351, 0.051687211569119429, 0.053335366174793619, 0.055420714678066418, 0.057022245737890498, 0.057476353532597835]
        lVel = [0.0011202756036718634, 0.00033781588556331917, 0.0004846206809068737, 0.00073700129188925651, 0.00050470968203249168, 0.00029650849798321322, 0.0013081544847480607, 0.00057509064574816479, 0.0013099335320174541, 0.00095355154580140913, 0.00055279979869076639, 0.00051669217493623641, 0.00018824000150451366, 0.0008031624792536183, 0.00057164318105100214, 0.00040391019673159098, 0.00036281066616313792, 0.00034193984238540642, 0.00072001310770125788, 0.00062834427426683643, 0.00091190701389257459, 0.00061065959274913368, 0.00052805347319978503, 0.0013317466576540664, 0.0042470157628043811, 0.008654485177938348, 0.012075191803805155, 0.015127657053251342, 0.015426871074838179, 0.011027610363160818, 0.0094138314977969314, 0.0077045850260083959, 0.010385501704421436, 0.019825921555762044, 0.035656601628427181, 0.044939543071928573, 0.032956388813803209, 0.013151043492956156, 0.01593890966592108, 0.018512908646977824, 0.01677426103909227, 0.01316815165882335, 0.013317389357399971, 0.018974406062925688, 0.021000835373653201, 0.016394758620914714, 0.023999898118448076, 0.032515827151142825, 0.028698283557469662, 0.031181861382464159, 0.052698059524779253, 0.11497708372082344, 0.20073525786459126, 0.33421923936045328, 0.62834157036040139, 1.0935026226263176, 1.5885785694360797, 1.9174140975412002, 2.0579788915996691, 2.0966542464166293, 2.0960319269753485, 2.1029484706001185, 2.1235771012402926, 2.1691038555524487, 2.2586873675664516, 2.3791972582408243, 2.4788858709078085, 2.5145226233231028, 2.4701233532196598, 2.31681509741068, 1.9987533479365329, 1.4983086621433033, 0.96634322029577824, 0.60354583307117515, 0.36710218219668367, 0.19736160970745986, 0.069844605372247195, 0.020255477726904968, 0.027160385781028033, 0.023364976611369704, 0.013199829518385027, 0.01654254660862494, 0.014155825596559111, 0.014790262768424474, 0.012194647059343576, 0.0100619313426941, 0.0098257831619633258, 0.010826734463642596, 0.010609185494175064, 0.017736721093973014, 0.031658961146377458, 0.077525611226610905, 0.16374084877467568, 0.25441824695457649, 0.4100824715284016, 0.70180349654848395, 1.1722388718019778, 1.7317478014078425, 2.1385039575026776, 2.3219988698817136, 2.3930881696284798, 2.470116455704622, 2.5908860481933038, 2.7103743151760877, 2.8323860464763033, 2.9821355354738133, 3.1143992396373026, 3.1558396424650521, 3.0666842596230017, 2.8556536687862981, 2.5204318401241257, 1.9980917232697977, 1.3539815552070542, 0.8695781119636683, 0.61615457717091815, 0.42218251965893722, 0.22277104183579491, 0.039787880846741071, 0.03221589958128003, 0.0083077173716689379, 0.02116706034361205, 0.0092940335660718788, 0.015893383247470586, 0.007783797367202713, 0.012393349451537021, 0.013005702044841274, 0.013964496803892742, 0.021262939680773363, 0.022800682618016054, 0.025289568419724504, 0.044174203471089996, 0.080054184287915345, 0.13336491833737985, 0.20381533385435388, 0.30082980051907426, 0.44906832709481109, 0.71712288428044335, 1.1493447952399889, 1.6223153842292577, 1.9239871063769889, 2.0372397941384768, 2.0781805892968883, 2.1118962848116127, 2.1405451467439018, 2.1526393407715507, 2.1737633558475471, 2.1686817729952335, 2.0500076839883978, 1.8168681256812502, 1.5246663459617826, 1.1697913370410455, 0.77023671995916354, 0.49804272027315222, 0.36582173807777851, 0.19343173725429655, 0.030526257331869238, 0.04550384727788527, 0.010044518736922288, 0.014653848518090404, 0.0086590632818380978, 0.0067723621121054747, 0.010585558048951402, 0.0066463016330596744, 0.011651153129178967, 0.0089112227393549574, 0.0076759193827398552, 0.012347218229952519, 0.01123901689229166, 0.0037154728833587647, 0.0060262138284225787, 0.0074727081329840246, 0.0077840725945317315, 0.0092843529301187468, 0.0093326737943536364, 0.007209313399140365, 0.0065902495056031135, 0.0066555551274311191, 0.0053355736831841392, 0.0046007691159147554, 0.0071374871799950359, 0.011654463458096044, 0.010661679075415284, 0.009006924590017059, 0.010729373217652121, 0.0098756877284412216, 0.012295107919981434, 0.027657577310565999, 0.061211966688519893, 0.12424295584677739, 0.25348444436569983, 0.44408285870985292, 0.55491399143569586, 0.40852590894338542, 0.25452108392809708]
        rPos = [0.061222919917946261, 0.061274660758366073, 0.061330164271055698, 0.061331442978247885, 0.061312635077405808, 0.061308251019657811, 0.061319076177355436, 0.061319323613206522, 0.061302127026188813, 0.061292509692716379, 0.061293361567797688, 0.061293501482043378, 0.061288193652352163, 0.061298183714006083, 0.061323311878140663, 0.061335434747286643, 0.061322150219358107, 0.06131029698713325, 0.061318982801737698, 0.061330355871306863, 0.061326904944247584, 0.061318399514690258, 0.0613099449527017, 0.061291806802249327, 0.061266080865655881, 0.061306282894755926, 0.061374006086604427, 0.061269757112902112, 0.060886992403610285, 0.06034738496170361, 0.059915575670946297, 0.059778018527962884, 0.059909334819423732, 0.060309433841403137, 0.061464350676823098, 0.065403778481374131, 0.076091827932630696, 0.096217137116292761, 0.12267536134585472, 0.14981038540975311, 0.17327357540889077, 0.19089387409024816, 0.20225613206247944, 0.20739089419565171, 0.20702200840954488, 0.20227188937124679, 0.19270582107915968, 0.17737292980443076, 0.15825729598387811, 0.13855107826123919, 0.11800774428800603, 0.095393986191911417, 0.073997613201273793, 0.060817200384563785, 0.057665462093604847, 0.058500723044149994, 0.058381028273640168, 0.05737240459495957, 0.057425875687260708, 0.0579764672934866, 0.057714007852277871, 0.05711526148556012, 0.057025535211740641, 0.056980053230189509, 0.056466728161800206, 0.056139479157075844, 0.056506855634784858, 0.056924250752536654, 0.056974193546307617, 0.057259035628094035, 0.057931352753801846, 0.05860665804715115, 0.059577752881315726, 0.061973239968458604, 0.067188768506297181, 0.077914386444020545, 0.097945202402471121, 0.13051157783709333, 0.169135313976992, 0.20245876000919899, 0.22475866672320738, 0.23285861275137149, 0.22672664922769065, 0.20998055079322592, 0.18630059601967086, 0.1598712733079467, 0.13601974120099219, 0.11777658500720195, 0.10533322258010358, 0.098921450535039979, 0.096991478716164925, 0.093742787762793489, 0.083678609586236186, 0.06919161122383588, 0.057787848511693662, 0.052834080878851497, 0.052231290179312384, 0.0519540236910363, 0.05138038056885702, 0.051280626556045372, 0.051465811649628879, 0.051412077713751891, 0.051575788162478597, 0.052277547724899842, 0.052857364390408612, 0.05286192590612343, 0.052785287176020323, 0.053066407129054305, 0.053648134440954787, 0.054864111347823596, 0.057007380712959355, 0.059831068178831881, 0.063499229471591589, 0.068922125889527175, 0.076751253667833197, 0.087907405789958704, 0.10551976105267658, 0.13350307701094916, 0.16834832283680351, 0.20030121368739778, 0.22449209016075788, 0.23726735694383333, 0.23565118320902917, 0.22142577252739892, 0.19770248661821183, 0.16839784278095521, 0.13998863321040411, 0.11805100759796194, 0.10360067983745225, 0.095699286935286676, 0.094011611858791644, 0.096156146273450427, 0.095298783650386021, 0.085361583063637614, 0.068816701360193966, 0.054981136894827809, 0.049581883717283559, 0.049352448371698721, 0.049002035780919495, 0.048027373287728325, 0.047752700510288182, 0.048058330088058709, 0.047770598804341313, 0.047127906231959116, 0.046971394370776187, 0.047138729406633217, 0.047195380457804859, 0.047405170521285289, 0.048074684166604154, 0.048822434223164068, 0.049297766978568358, 0.049735121754010958, 0.050326363417510622, 0.050620541426656396, 0.050455897155090079, 0.050827497908350838, 0.052288020600530172, 0.054725600310742695, 0.056869931015401276, 0.057878641432715983, 0.057935861927397247, 0.05753074736203595, 0.057079042157764603, 0.056589525837357157, 0.056100731433300577, 0.055729691418155092, 0.055383391362974266, 0.054897050805968628, 0.054290804556989813, 0.053755233720157947, 0.053382253138820834, 0.053098620936360197, 0.052814565547751102, 0.052541457793046531, 0.052350392416484148, 0.052280584210786352, 0.052347026802366858, 0.052510933829499451, 0.052637350000539485, 0.052547213185091535, 0.052162577056246551, 0.051614824997745223, 0.05111512304060134, 0.050780650207520706, 0.050584330076029083, 0.050439524726958784, 0.050291968632354966, 0.050096440870567804, 0.049846559493280873, 0.049599288389655327, 0.049447770513520284, 0.04938998770235925, 0.049378812961786589, 0.049383312329271123]
        rVel = [0.0046262395755879626, 0.0045937479647702398, 0.0022228436562269885, 0.0007679422198713816, 0.00076078207018263368, 0.00014816271428863694, 0.00090914043729275931, 0.00036761228135861432, 0.0019504187313277293, 0.0015859462656931907, 0.00022262509865310598, 0.00089034008661099538, 0.0002246708979771198, 0.00074381034211889454, 0.00065193101150904027, 0.00093742209732527201, 0.00095185768399108726, 0.00021031918620227164, 0.00084079119001994674, 0.00036936152605013571, 0.0011033752987517078, 0.00077457287215786498, 0.0023147030049134607, 0.0030352489394468336, 0.0042711406189746472, 0.0068412420598881196, 0.013012516364516122, 0.024161407407740497, 0.027581967899513886, 0.021083702491759192, 0.015388246325529027, 0.013731679512545157, 0.0096171246120520717, 0.026179229243782909, 0.089331242508054645, 0.26112409779886636, 0.55834671628366184, 0.88490624729716971, 1.1187097120770479, 1.2496875416520092, 1.3522572701463602, 1.448259440568989, 1.4987020344754165, 1.5132401224094632, 1.5256876457448389, 1.523367166913062, 1.4956000971027572, 1.4545311610981237, 1.3565134767881875, 1.1490910360565081, 0.90548947924290701, 0.74396887833666747, 0.59099965318296122, 0.37959623523890734, 0.22251146121566645, 0.082524946753436065, 0.018920313456876798, 0.015169450381279308, 0.019303677258572911, 0.016701688879735218, 0.020647111634701305, 0.010408556490088351, 0.0093980106197019339, 0.0095047022250743952, 0.016186487232168246, 0.0094256879636127906, 0.016629642687088979, 0.011612796594690753, 0.0089842326878907461, 0.01892450593175498, 0.023228714748788654, 0.030751899600871539, 0.059817448332257078, 0.12593382225453206, 0.26392031661418008, 0.54099234072567826, 1.0018022954935653, 1.5559129385809627, 1.9851594732451481, 2.1684595986890054, 2.2422242578978762, 2.387674002723096, 2.618282512134539, 2.8395551050292851, 3.0125631800769552, 3.1332113492688869, 3.1644041660333611, 3.0718840003558205, 2.8592642920054607, 2.5312455215267788, 2.0661520337561443, 1.513666231431898, 1.0299359245089217, 0.71547707784909409, 0.52355869580409986, 0.37751212790831618, 0.20509842427840488, 0.063351173079492312, 0.030381836844301745, 0.01656996970135053, 0.02481429450413215, 0.014511182299477855, 0.016599292099899635, 0.02725557326386743, 0.019494924924040966, 0.016407744036607722, 0.018685917488221921, 0.016464831799939451, 0.029961834355683309, 0.05441990593675005, 0.081254326525890944, 0.1122988032999207, 0.15982579087144425, 0.22747440136227076, 0.32636219004068251, 0.52920905009440811, 0.92748782480338055, 1.4764340997234537, 1.9591524418593993, 2.2134728137969697, 2.3104521705842522, 2.4183155681048789, 2.6234980559217376, 2.8813138641022151, 3.1162750737901224, 3.278377849143125, 3.3354925206252579, 3.2813578695186205, 3.1111555830834181, 2.8358609124270524, 2.4864077172155903, 2.0397405109746702, 1.4792619419394266, 0.99446127241109983, 0.74427417207864999, 0.57472163845417379, 0.45735746031399116, 0.28442876584837551, 0.098423042982583073, 0.062275424658535049, 0.034000855826344045, 0.018190375923911609, 0.018951906768457309, 0.014881850228903523, 0.020097977100052639, 0.014613931510659249, 0.013015157628775332, 0.024204935449941792, 0.029031872160647447, 0.021175011903432923, 0.019529658391625525, 0.023009106709226353, 0.018252756434828925, 0.021734103687737059, 0.0081537377506922804, 0.029078997215883317, 0.062243111364496263, 0.077131723778789366, 0.051515200812270227, 0.017650486763638163, 0.0096854958257432105, 0.023919299966272817, 0.023115631937429992, 0.020999950487571108, 0.016949584173177969, 0.0170179207550759, 0.018881200941094856, 0.025808289977578731, 0.02399338912497042, 0.015726666379373987, 0.009944393498749142, 0.008531332398070697, 0.0085846019897931475, 0.0090554862730762246, 0.0097704651672118458, 0.008206189472184303, 0.0060029099121172817, 0.0050216850912775937, 0.0023651094996464039, 0.0078900016475742327, 0.014914801555714105, 0.017041397435671121, 0.014052867203765834, 0.010826258368287586, 0.0088219972140234065, 0.0061809086857709571, 0.0054855728717769087, 0.0071269898643277407, 0.0083393830891470618, 0.0074703179808293177, 0.0048231688514143115, 0.0016353221434325749, 0.0007896144432903054, 0.00085066981845470123]
        contactLeft = [True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, False, False, False, False, False, False, False, False, False, False, False, False, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True]
        contactRight = [True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, False, False, False, False, False, False, False, False, False, False, False, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True]
        frameBlocks = [[0, 49], [50, 71], [72, 92], [93, 112], [113, 132], [133, 193]]
        lBlocks = [[50, 71], [93, 112], [133, 193]]
        rBlocks = [[0, 49], [72, 92], [113, 132]]
        lTakingFrames = [57, 97, 139]
        lLandingFrames = [72, 113, 151]
        rTakingFrames = [39, 78, 118]
        rLandingFrames = [50, 93, 133]
        lcenterFrames = [57, 64, 97, 105, 139, 145]
        rcenterFrames = [39, 44, 78, 85, 118, 125]
        hRef = 10
        vRef = 1.0
        
        plot = SmartPlot()
        plot.setXdata('frame', range(len(lPos)))
        plot.addYdata('lPos', lPos)
        plot.addYdata('lVel', lVel, False)
        plot.addYdata('rPos', rPos)
        plot.addYdata('rVel', rVel, False)
        plot.addYdata('lContact', contactLeft, False)
        plot.addYdata('rContact', contactRight, False)
        plot.addXspans('frameBlocks', frameBlocks)
        plot.addXspans('lBlocks', lBlocks)
        plot.addXspans('rBlocks', rBlocks)
        plot.addXlines('lTakingFrames', lTakingFrames)
        plot.addXlines('lLandingFrames', lLandingFrames)
        plot.addXlines('rTakingFrames', rTakingFrames)
        plot.addXlines('rLandingFrames', rLandingFrames)
        plot.addXlines('lcenterFrames', lcenterFrames)
        plot.addXlines('rcenterFrames', rcenterFrames)
        plot.addYlines('hRef', [hRef])
        plot.addYlines('vRef', [vRef])
        plot.show()
        
    def test_InteractivePlot():
        import Resource.ysMotionLoader as yf
        import Renderer.ysRenderer as yr
        import GUI.ysSimpleViewer as ysv

        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        jointMotion = yf.readBvhFile(bvhFilePath, .01)
        jointMotion = jointMotion[0:200]
        
        hips = jointMotion[0].skeleton.getJointIndex('Hips')
        lFoot = jointMotion[0].skeleton.getJointIndex('LeftFoot')
        rFoot = jointMotion[0].skeleton.getJointIndex('RightFoot')
        
        plot = InteractivePlot()
        plot.setXlimit(0, len(jointMotion))
        plot.setYlimit(0, 2)
        plot.addDataSet('root.y')
        plot.addDataSet('leftFoot.y')
        plot.addDataSet('rightFoot.y')

        viewer = ysv.SimpleViewer()
        viewer.doc.addRenderer('motion(%s)'%jointMotion.resourceName, yr.JointMotionRenderer(jointMotion, (0, 0, 255), yr.LINK_LINE))
        viewer.doc.addObject('motion(%s)'%jointMotion.resourceName, jointMotion)
        
        pt = [None]
        updated = [False]
        def preFrameCallback(frame):
            if updated[0] == False:
                plot.addDataPoint('root.y', frame, jointMotion[frame].getJointPositionGlobal(hips)[1])
                plot.addDataPoint('leftFoot.y', frame, jointMotion[frame].getJointPositionGlobal(lFoot)[1])
                plot.addDataPoint('rightFoot.y', frame, jointMotion[frame].getJointPositionGlobal(rFoot)[1])
                plot.updatePoints()
            if frame == viewer.getMaxFrame():
                updated[0] = True
            
            if frame==0:
                pt[0] = time.time()
            if frame==50:
                print time.time()-pt[0]
        
        def preFrameCallback_Always(frame):
            plot.updateVline(frame)
            
        def viewer_onClose(data):
            plot.close()
            viewer.onClose(data)
            
        viewer.setPreFrameCallback(preFrameCallback)
        viewer.setPreFrameCallback_Always(preFrameCallback_Always)
        viewer.callback(viewer_onClose)
            
        viewer.startTimer(1/30.)
        viewer.show()
        Fl.run()

    def test_interactive_mode():
        '''
import matplotlib
matplotlib.interactive(True)
matplotlib.use('FltkAgg')

from pylab import *
plot([1,2,3])
xlabel('time (s)')

from fltk import *
window = Fl_Window(300,300)
window.label("Window Test")
window.show()
Fl.run()
        '''

        import matplotlib
        matplotlib.interactive(True)
#        matplotlib.use('FltkAgg')
        
        import pylab
#        manager = pylab.get_current_fig_manager()
        
        pylab.plot([1,2,3])
        pylab.xlabel('time (s)')
        pylab.draw()
        
        window = Fl_Window(300,300)
        window.label("Window Test")
        window.show()
        Fl.run()
        
    def test_plot_LineCollection_speed():
        import GUI.ysViewer as yv
        import Motion.ysMotion as ym
        import numpy as np
        from matplotlib import collections
        import time
        
        pt = time.time()
         
        count = 5
        
        # plot
        xs = []
        ys = []
#        lines = []
        for i in range(count):
            xs.append([])
            ys.append([])
#            line, = plot(range(100), range(100))
#            lines.append(line)
        def beforeFrameCallback(frame):
#            print frame
            for i in range(count):
                xs[i].append(frame)
                ys[i].append(frame+i)
                
#                plot(xs[i], ys[i], 'b')
                plot([frame, frame+1], [frame+i, frame+i+1], 'b')
#                lines[i].set_xydata(frame,frame+i)
            
#            xys = []
#            for i in range(count):
#                xys.append(xs[i])
#                xys.append(ys[i])
#            plot(xys)

            xlim(0,100)
            ylim(0,100)
            draw()
            if frame == 90:
                print time.time() - pt

##        # LineCollection
#        fig = figure()
#        a = fig.add_subplot(111)
#        xys = []
#        for i in range(count):
#            xys.append([])
#        def beforeFrameCallback(frame):
##            print frame
#            for i in range(count):
#                xys[i].append((frame, frame+i))
#                
##                col = collections.LineCollection([xys[i]])
#                col = collections.LineCollection([[(frame, frame+i), (frame+1, frame+i+1)]])
#                a.add_collection(col)
#                
##            col = collections.LineCollection(xys)
##            a.add_collection(col)
#
#            xlim(0,100)
#            ylim(0,100)
#            draw()
#            if frame == 90:
#                print time.time() - pt
        
        tempMotion = ym.Motion([None]*100)
        motionSystem = ym.MotionSystem()
        motionSystem.addMotion(tempMotion)
    
        viewer = yv.Viewer(100, 100, 800, 650, None, motionSystem, [])
        
        viewer.beforeFrameCallback = beforeFrameCallback
        
        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()
        
    def test_showModeless():
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        jointMotion = yf.readBvhFile(bvhFilePath, .01)
#        jointMotion = jointMotion[0:10]
        
        plot = SmartPlot()
        plot.setXdata('frame', [0])
        plot.showModeless()
        
        viewer = ysv.SimpleViewer()
        viewer.doc.addRenderer('motion(%s)'%jointMotion.resourceName, yr.JointMotionRenderer(jointMotion, (0, 0, 255), yr.LINK_LINE))
        viewer.doc.addObject('motion(%s)'%jointMotion.resourceName, jointMotion)
        
        viewer.startTimer(1/30.)
        viewer.show()
        Fl.run()

    def test_keypress_event():
        import numpy as n
        
        manager = get_current_fig_manager()

        def press(event):
            if event.key == 'escape' or event.key == None:
                print 'close'
                manager.window.do_callback()
                
        fig = figure(1)
        fig.canvas.mpl_connect('key_press_event', press)
        
        manager.window.position(100,100)
#        manager.window.callback(onClose)
        
        show()
        
    def test_fltk_window_manipulation():
        window = Fl_Window(300, 300)
        
        def onClose(widget):
            print 'onClose'
            print widget
            window.default_callback(widget, None)
        
        def handle(event):
            print 'handle'
            print event
            return 0

#        print window.modal()
        window.callback(onClose)
        window.handle = handle
        window.size_range(100,100)
        window.label('test')
        
        window.show()
        Fl.run()
        
    def test_matplot_fltk_window_manipulation():
        manager = get_current_fig_manager()
        print 'manager: ', manager
        for key, value in manager.__dict__.items():
            print key, ':', value
        print
        
        def onClose(widget, data):
#            print 'onClose'
#            print widget, data
            manager.window.default_callback(widget, None)
        manager.window.callback(onClose)
            

#        def handle(event):
#            print 'handle'
#            print widget ,event
#            return 0
        
        def press(event):
            if event.key == 'escape' or event.key == None:
                manager.window.do_callback()
        fig = figure(1)
        fig.canvas.mpl_connect('key_press_event', press)
        print 'fig: ', fig
        for key, value in fig.__dict__.items():
            print key, ':', value
        print 
        
        print 'fig.canvas: ', fig.canvas    
        for key, value in fig.canvas.__dict__.items():
            print key, ':', value
        print 
        
#        manager.window.set_modal()
#        print manager.window.modal()
        
#        fig.canvas.resize((100,100))
        
#        manager.window.position(100,100)
#        manager.window.handle = handle
        manager.window.size_range(100,100)
#        manager.window.resize(200,200,200,200)
#        manager.window.size(100,100)
#        manager.window.resizable(manager.window)
#        manager.window.fullscreen()
#        manager.window.label('test')
        
        show()

    def test_matplot_tk_window_manipulation():
        manager = get_current_fig_manager()
        window = manager.window
        window.geometry('800x300+100+200')
        
        window.update()
        print window.geometry()
        print window.winfo_x()
        print window.winfo_y()
        print window.winfo_width()
        print window.winfo_height()
        print
        
        show()
        
    def test_LineCollection():
        import matplotlib.pyplot as P
        from matplotlib import collections, axes, transforms
        from matplotlib.colors import colorConverter
        import numpy as N
        
        fig = figure()
        a = fig.add_subplot(2,2,1)
        col = collections.LineCollection([((0,0),(1,1))])
        a.add_collection(col, autolim=True)
        
        a.autoscale_view()
        a.set_title('Successive data offsets')
        a.set_xlabel('Zonal velocity component (m/s)')
        a.set_ylabel('Depth (m)')
        # Reverse the y-axis so depth increases downward
        a.set_ylim(a.get_ylim()[::-1])
        
        P.show()
    
    def test_anim_py():
        ion()
        tstart = time.time()               # for profiling
        x = arange(0,2*pi,0.01)            # x-array
        line, = plot(x,sin(x))
        for i in arange(1,200):
            line.set_ydata(sin(x+i/10.0))  # update the data
            draw()                         # redraw the canvas
        print 'FPS:' , 200/(time.time()-tstart)
        
    pass
#    test_showModeless()
#    test_anim_py()
#    test_LineCollection()
#    test_matplot()
#    test_matplot2()
    test_InteractivePlot()
#    test_SmartPlot()
#    test_interactive_mode()
#    test_plot_LineCollection_speed()
#    test_fltk_window_manipulation()
#    test_matplot_tk_window_manipulation()
#    test_matplot_fltk_window_manipulation()
#    test_keypress_event()
