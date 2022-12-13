import sys
import struct

from PyQt5.QtWidgets import (QButtonGroup)
from gui_threads import *
from graphUtilities import *
from gl_classes import GLTextItem
from oob_parser import uartParserSDK

import pkg_resources

pkg_resources.require("pyqt5==5.15.4", "pyopengl==3.1.5", "numpy==1.19.4",
                      "pyserial==3.5", "pyqtgraph==0.11.0", "matplotlib==3.5.2",
                      "playsound==1.2.2", "pyttsx3==2.90")


class Window(QDialog):
    def __init__(self, size):
        super(Window, self).__init__(None)
        # set window toolbar options, and title.
        self.lastFrameHadTargets = False
        self.uart_thread = None
        self.parseTimer = None
        self.get_thread = None
        self.plotStart = None
        self.averagePlot = None
        self.staticLine = None
        self.boundaryLine = None
        self.cfg = None
        self.cThread = None
        self.setWindowFlags(
            Qt.Window |
            Qt.CustomizeWindowHint |
            Qt.WindowTitleHint |
            Qt.WindowMinimizeButtonHint |
            Qt.WindowMaximizeButtonHint |
            Qt.WindowCloseButtonHint
        )
        self.setWindowTitle("Navigation Assistant")

        self.frameTime = 50
        self.graphFin = 1
        self.hGraphFin = 1
        self.threeD = 1
        self.lastFramePoints = np.zeros((5, 1))
        self.plotTargets = 1
        self.frameNum = 0
        self.lastTID = []
        self.profile = {'startFreq': 60.25, 'numLoops': 64, 'numTx': 3,
                        'sensorHeight': 0.82, 'maxRange': 10, 'az_tilt': 0, 'elev_tilt': 0}
        self.sensorHeight = 0.82
        self.numFrameAvg = 10
        self.configSent = 0
        self.previousFirstZ = -1
        self.yzFlip = 0

        self.zRange = [-3, 3]
        self.plotHeights = 1

        # gui size
        if size:
            left = math.ceil(size.width() * 0.5)
            top = 50
            width = math.ceil(size.width() * 0.5)
            self.setGeometry(left, top, width, 0)

        # Persistent point cloud
        self.previousCloud = np.zeros((6, 1150, 10))
        self.previousPointCount = np.zeros((10, 1))

        # remove points outside boundary box
        self.bbox = [-4, 4, -0.5, 6, 0, 3]

        # Setup graph pyqtgraph
        self.pcPlot = gl.GLViewWidget()
        self.gz = gl.GLGridItem()
        self.boundaryBoxViz = [gl.GLLinePlotItem(), gl.GLLinePlotItem(), gl.GLLinePlotItem()]
        self.bottomSquare = [gl.GLLinePlotItem(), gl.GLLinePlotItem(), gl.GLLinePlotItem()]
        self.scatter = gl.GLScatterPlotItem(size=5)
        self.coordStr = []
        self.ellipsoids = []
        self.evmBox = None
        self.plot3DQTGraph()

        # Setup connection Layout
        self.comBox = QGroupBox('Connect to Com Ports')
        self.uartCom = QLineEdit('6')
        self.dataCom = QLineEdit('5')
        self.uartLabel = QLabel('UART COM:')
        self.dataLabel = QLabel('DATA COM:')
        self.connectStatus = QLabel('Not Connected')
        self.connectButton = QPushButton('Connect')
        self.connectButton.clicked.connect(self.connectCom)
        self.configLabel = QLabel('Config Type:')
        self.comLayout = QGridLayout()
        self.setConnectionLayout()

        # Setup stats layout
        self.statBox = QGroupBox('Statistics')
        self.frameNumDisplay = QLabel('Frame: 0')
        self.plotTimeDisplay = QLabel('Average Plot Time: 0 ms')
        self.numPointsDisplay = QLabel('Points: 0')
        self.numTargetsDisplay = QLabel('Targets: 0')
        self.statsLayout = QVBoxLayout()
        self.setStatsLayout()

        # Setup plots layout
        self.plotControlBox = QGroupBox('Plot Controls')
        self.staticClutter = QCheckBox('Display Static Points')
        self.plotByIndex = QCheckBox('Plot Point Color by Index')
        self.plotByHeight = QCheckBox('Plot Point Color By Height')
        self.plotTracks = QCheckBox('Plot Tracks')
        self.pointColorGroup = QButtonGroup()
        self.persistentFramesInput = QComboBox()
        self.pFILabel = QLabel('# of Persistent Frames')
        self.plotControlLayout = QGridLayout()
        self.setPlotControlLayout()

        # Setup configuration layout
        self.configBox = QGroupBox('Configuration')
        self.selectConfig = QPushButton('Select Configuration')
        self.sendConfig = QPushButton('Start and Send Configuration')
        self.start = QPushButton("Start without Send Configuration ")
        self.configTable = QTableWidget(5, 2)
        self.configLayout = QVBoxLayout()
        self.graphTabs = QTabWidget()
        self.setConfigLayout()

        # Setup boundary box controls
        self.boundaryBoxes = []
        self.boxTab = QTabWidget()
        self.setUpBoundaryBoxControls()

        # Setup sensor position controls
        self.az_tilt = QLineEdit('0')
        self.az_tiltL = QLabel('Azimuth Tilt')
        self.elev_tilt = QLineEdit('0')
        self.elev_tiltL = QLabel('Elevation Tilt')
        self.s_height = QLineEdit(str(self.profile['sensorHeight']))
        self.s_heightL = QLabel('Sensor Height')
        self.spLayout = QGridLayout()
        self.spBox = QGroupBox('Sensor Position')
        self.setSensorPositionControls()

        # create tab for different graphing options
        self.graphTabs = QTabWidget()
        self.graphTabs.addTab(self.pcPlot, '3D Plot')
        self.graphTabs.currentChanged.connect(self.whoVisible)

        self.parser = uartParserSDK()
        self.useFilter = 0

        gridlayout = QGridLayout()
        gridlayout.addWidget(self.comBox, 0, 0, 1, 1)
        gridlayout.addWidget(self.statBox, 1, 0, 1, 1)
        gridlayout.addWidget(self.configBox, 2, 0, 1, 1)
        gridlayout.addWidget(self.plotControlBox, 3, 0, 1, 1)
        gridlayout.addWidget(self.boxTab, 4, 0, 1, 1)
        gridlayout.addWidget(self.spBox, 5, 0, 1, 1)
        gridlayout.addWidget(self.graphTabs, 0, 1, 6, 1)
        gridlayout.setColumnStretch(0, 1)
        gridlayout.setColumnStretch(1, 3)
        self.setLayout(gridlayout)

        print('Python is ', struct.calcsize("P") * 8, ' bit')
        print('Python version: ', sys.version_info)

    #
    # left side pane layout
    #
    def setConnectionLayout(self):
        self.comLayout.addWidget(self.uartLabel, 0, 0)
        self.comLayout.addWidget(self.uartCom, 0, 1)
        self.comLayout.addWidget(self.dataLabel, 1, 0)
        self.comLayout.addWidget(self.dataCom, 1, 1)
        self.comLayout.addWidget(self.connectButton, 2, 0)
        self.comLayout.addWidget(self.connectStatus, 2, 1)
        self.comBox.setLayout(self.comLayout)

    def setStatsLayout(self):
        self.statBox = QGroupBox('Statistics')
        self.frameNumDisplay = QLabel('Frame: 0')
        self.plotTimeDisplay = QLabel('Average Plot Time: 0 ms')
        self.numPointsDisplay = QLabel('Points: 0')
        self.numTargetsDisplay = QLabel('Targets: 0')
        self.statsLayout = QVBoxLayout()
        self.statsLayout.addWidget(self.frameNumDisplay)
        self.statsLayout.addWidget(self.plotTimeDisplay)
        self.statsLayout.addWidget(self.numPointsDisplay)
        self.statsLayout.addWidget(self.numTargetsDisplay)
        self.statBox.setLayout(self.statsLayout)

    def setPlotControlLayout(self):
        self.pointColorGroup.addButton(self.plotByIndex)
        self.pointColorGroup.addButton(self.plotByHeight)
        self.pointColorGroup.setExclusive(True)
        self.persistentFramesInput.addItems(['1', '2', '3', '4', '5', '6', '7', '8', '9', '10'])
        self.persistentFramesInput.setCurrentIndex(2)
        self.plotControlLayout.addWidget(self.plotByIndex, 0, 0, 1, 1)
        self.plotControlLayout.addWidget(self.plotByHeight, 1, 0, 1, 1)
        self.plotControlLayout.addWidget(self.plotTracks, 2, 0, 1, 1)
        self.plotControlLayout.addWidget(self.staticClutter, 3, 0, 1, 1)
        self.plotControlLayout.addWidget(self.persistentFramesInput, 4, 0, 1, 1)
        self.plotControlLayout.addWidget(self.pFILabel, 4, 1, 1, 1)
        self.plotControlBox.setLayout(self.plotControlLayout)
        # initialize button values
        self.plotByHeight.setChecked(False)
        self.plotByIndex.setChecked(True)
        self.plotTracks.setChecked(True)

    def setConfigLayout(self):
        self.selectConfig.clicked.connect(self.selectCfg)
        self.sendConfig.clicked.connect(self.sendCfg)
        self.start.clicked.connect(self.startApp)
        self.configLayout.addWidget(self.selectConfig)
        self.configLayout.addWidget(self.sendConfig)
        self.configLayout.addWidget(self.start)
        self.configBox.setLayout(self.configLayout)

    # Boundary box control section
    def setBoxControlLayout(self, name):
        # Set up one boundary box control
        boxControl = QGroupBox(name)
        # Input boxes
        lx = QLineEdit('-1')
        rx = QLineEdit('1')
        ny = QLineEdit('2')
        fy = QLineEdit('4')
        bz = QLineEdit('0')
        tz = QLineEdit('3')
        enable = QCheckBox()
        # Labels
        lxL = QLabel('Left X')
        rxL = QLabel('Right X')
        nyL = QLabel('Near Y')
        fyL = QLabel('Far Y')
        bzL = QLabel('Bottom Z')
        tzL = QLabel('Top Z')
        enableL = QLabel('Enable Box')
        boxConLayout = QGridLayout()
        boxConLayout.addWidget(lxL, 0, 0, 1, 1)
        boxConLayout.addWidget(lx, 0, 1, 1, 1)
        boxConLayout.addWidget(rxL, 0, 2, 1, 1)
        boxConLayout.addWidget(rx, 0, 3, 1, 1)
        boxConLayout.addWidget(nyL, 1, 0, 1, 1)
        boxConLayout.addWidget(ny, 1, 1, 1, 1)
        boxConLayout.addWidget(fyL, 1, 2, 1, 1)
        boxConLayout.addWidget(fy, 1, 3, 1, 1)
        boxConLayout.addWidget(bzL, 2, 0, 1, 1)
        boxConLayout.addWidget(bz, 2, 1, 1, 1)
        boxConLayout.addWidget(tzL, 2, 2, 1, 1)
        boxConLayout.addWidget(tz, 2, 3, 1, 1)
        boxConLayout.addWidget(enableL, 3, 0, 1, 1)
        boxConLayout.addWidget(enable, 3, 1, 1, 1)
        boxControl.setLayout(boxConLayout)
        boundList = [lx, rx, ny, fy, bz, tz]
        for text in boundList:
            text.textEdited.connect(self.changeBoundaryBox)
        enable.stateChanged.connect(self.changeBoundaryBox)
        return {'boxCon': boxControl, 'boundList': boundList, 'checkEnable': enable, 'boxNum': -1}

    def setSensorPositionControls(self):
        self.spLayout.addWidget(self.az_tilt, 0, 1, 1, 1)
        self.spLayout.addWidget(self.az_tiltL, 0, 0, 1, 1)
        self.spLayout.addWidget(self.elev_tilt, 1, 1, 1, 1)
        self.spLayout.addWidget(self.elev_tiltL, 1, 0, 1, 1)
        self.spLayout.addWidget(self.s_height, 2, 1, 1, 1)
        self.spLayout.addWidget(self.s_heightL, 2, 0, 1, 1)
        self.spBox.setLayout(self.spLayout)
        self.s_height.textEdited.connect(self.updateSensorPosition)
        self.az_tilt.textEdited.connect(self.updateSensorPosition)
        self.elev_tilt.textEdited.connect(self.updateSensorPosition)

    def updateSensorPosition(self):
        try:
            float(self.s_height.text())
            float(self.az_tilt.text())
            float(self.elev_tilt.text())
        except ValueError:
            print("fail to update")
            return
        command = "sensorPosition " + self.s_height.text() + " " + self.az_tilt.text() + \
                  " " + self.elev_tilt.text() + " \n"
        self.cThread = sendCommandThread(self.parser, command)
        self.cThread.start(priority=QThread.HighestPriority - 2)
        self.gz.translate(dx=0, dy=0, dz=self.profile['sensorHeight'])
        self.profile['sensorHeight'] = float(self.s_height.text())
        self.gz.translate(dx=0, dy=0, dz=-self.profile['sensorHeight'])

    def setUpBoundaryBoxControls(self):
        # Set up all boundary box controls
        for i in range(3):
            name = 'Box' + str(i)
            self.boundaryBoxes.append(self.setBoxControlLayout(name))
            toAdd = self.boundaryBoxes[i]
            toAdd['boxNum'] = i
            self.boxTab.addTab(toAdd['boxCon'], name)

    # for live tuning when available
    def changeBoundaryBox(self):
        # send box values
        numBoxes = 0
        for box in self.boundaryBoxes:
            if box['checkEnable'].isChecked():
                numBoxes += 1
        boundaryString = "LiveScenery " + str(numBoxes) + " "
        for box in self.boundaryBoxes:
            if box['checkEnable'].isChecked():
                for text in box['boundList']:
                    val = text.text()
                    val = val.replace(" ", "")
                    try:
                        float(val)
                    except ValueError:
                        print('nothing here')
                        return
                    boundaryString += text.text() + " "
                self.drawBoundaryBox3d(box['boxNum'])
            else:
                print("Setting box ", box['boxNum'], " invisible")
                self.boundaryBoxViz[box['boxNum']].setVisible(False)
                self.bottomSquare[box['boxNum']].setVisible(False)
        boundaryString += "\n"
        if self.configSent:
            print(boundaryString)
            self.cThread = sendCommandThread(self.parser, boundaryString)
            self.cThread.start(priority=QThread.HighestPriority - 2)

    def setBoundaryTextVals(self, profile):
        # Update box text values based on config
        for box in self.boundaryBoxes:
            bList = box['boundList']
            bList[0].setText(str(profile['leftX']))
            bList[1].setText(str(profile['rightX']))
            bList[2].setText(str(profile['nearY']))
            bList[3].setText(str(profile['farY']))
            bList[4].setText(str(profile['bottomZ']))
            bList[5].setText(str(profile['topZ']))

    def drawBoundaryBox3d(self, boxIndex):
        bList = self.boundaryBoxes[boxIndex]['boundList']
        xl = float(bList[0].text())
        xr = float(bList[1].text())
        yl = float(bList[2].text())
        yr = float(bList[3].text())
        zl = float(bList[4].text()) - self.profile['sensorHeight']  # set z low of bbox to world coordinates
        zr = float(bList[5].text()) - self.profile['sensorHeight']  # set z hi of bbox to world coordinates
        print('Sensor Height = ', str(self.profile['sensorHeight']))
        print(xl, yl, zl, xr, yr, zr)
        self.bbox = [xl, xr, yl, yr, zl, zr]
        boxLines = getBoxLines(xl, yl, zl, xr, yr, zr)
        squareLine = getSquareLines(xl, yl, xr, yr, zl)
        if not self.boundaryBoxViz[boxIndex].visible():
            print("Setting Box ", str(boxIndex), " to visible")
            self.boundaryBoxViz[boxIndex].setVisible(True)
            self.bottomSquare[boxIndex].setVisible(True)
        self.boundaryBoxViz[boxIndex].setData(pos=boxLines, color=pg.glColor('r'),
                                              width=2, antialias=True, mode='lines')
        self.bottomSquare[boxIndex].setData(pos=squareLine, color=pg.glColor('b'), width=2, antialias=True,
                                            mode='line_strip')

    def plot3DQTGraph(self):
        dummy = np.zeros((1, 3))
        # create the background grids
        self.gz.translate(0, 0, -1 * self.profile['sensorHeight'])
        for box in self.boundaryBoxViz:
            box.setVisible(False)
        self.scatter.setData(pos=dummy)
        self.pcPlot.addItem(self.gz)
        self.pcPlot.addItem(self.boundaryBoxViz[0])
        self.pcPlot.addItem(self.boundaryBoxViz[1])
        self.pcPlot.addItem(self.boundaryBoxViz[2])
        self.pcPlot.addItem(self.bottomSquare[0])
        self.pcPlot.addItem(self.bottomSquare[1])
        self.pcPlot.addItem(self.bottomSquare[2])
        self.pcPlot.addItem(self.scatter)
        # create box to represent device
        verX = 0.0625
        verZ = 0.125
        offsetZ = 0
        vertices = np.empty((2, 3, 3))
        vertices[0, 0, :] = [-verX, 0, verZ + offsetZ]
        vertices[0, 1, :] = [-verX, 0, -verZ + offsetZ]
        vertices[0, 2, :] = [verX, 0, -verZ + offsetZ]
        vertices[1, 0, :] = [-verX, 0, verZ + offsetZ]
        vertices[1, 1, :] = [verX, 0, verZ + offsetZ]
        vertices[1, 2, :] = [verX, 0, -verZ + offsetZ]
        self.evmBox = gl.GLMeshItem(vertexes=vertices, smooth=False, drawEdges=True, edgeColor=pg.glColor('r'),
                                    drawFaces=False)
        self.pcPlot.addItem(self.evmBox)
        for m in range(0, 3):
            mesh = gl.GLLinePlotItem()
            mesh.setVisible(False)
            self.pcPlot.addItem(mesh)
            self.ellipsoids.append(mesh)
            # add track coordinate string
            text = GLTextItem()
            text.setGLViewWidget(self.pcPlot)
            text.setVisible(False)
            self.pcPlot.addItem(text)
            self.coordStr.append(text)

    def updateGraph(self, parsedData):
        pointCloud = parsedData[0]
        targets = parsedData[1]
        indexes = parsedData[2]
        numPoints = parsedData[3]
        numTargets = parsedData[4]
        self.frameNum = parsedData[5]
        fail = parsedData[6]
        rotTargetDataX, rotTargetDataY, rotTargetDataZ = rotX(targets[1], targets[2], targets[3],
                                                              -1 * self.profile['elev_tilt'])
        targets[1] = rotTargetDataX
        targets[2] = rotTargetDataY
        targets[3] = rotTargetDataZ

        # pass pointCloud XYZ vals and rotate due to elevation tilt angle (rotX uses Euler rotation around X axis)
        for i in range(numPoints):
            rotPointDataX, rotPointDataY, rotPointDataZ = rotX([pointCloud[0, i]], [pointCloud[1, i]],
                                                               [pointCloud[2, i]], -1 * self.profile['elev_tilt'])
            pointCloud[0, i] = rotPointDataX
            pointCloud[1, i] = rotPointDataY
            pointCloud[2, i] = rotPointDataZ

        if fail != 1:
            # left side
            pointStr = 'Points: ' + str(numPoints)
            targetStr = 'Targets: ' + str(numTargets)
            self.numPointsDisplay.setText(pointStr)
            self.numTargetsDisplay.setText(targetStr)
        if len(targets) < 13:
            targets = []
        if fail:
            return

        # remove static points
        if not self.staticClutter.isChecked():
            statics = np.where(pointCloud[3, :] == 0)
            try:
                firstZ = statics[0][0]
                numPoints = firstZ
                pointCloud = pointCloud[:, :firstZ]
                indexes = indexes[:, :self.previousFirstZ]
                self.previousFirstZ = firstZ
            except:
                firstZ = -1
        # point cloud persistence
        fNum = self.frameNum % 10
        if numPoints:
            self.previousCloud[:5, :numPoints, fNum] = pointCloud[:5, :numPoints]
            self.previousCloud[5, :len(indexes), fNum] = indexes
        self.previousPointCount[fNum] = numPoints
        # plotting 3D - get correct point cloud (persistent points and synchronize the frame)
        totalPoints = 0
        persistentFrames = int(self.persistentFramesInput.currentText())
        # allocate new array for all the points
        for i in range(1, persistentFrames + 1):
            totalPoints += self.previousPointCount[fNum - i]
        pointIn = np.zeros((5, int(totalPoints)))
        indicesIn = np.ones((1, int(totalPoints))) * 255
        totalPoints = 0
        # fill array
        for i in range(1, persistentFrames + 1):
            prevCount = int(self.previousPointCount[fNum - i])
            pointIn[:, totalPoints:totalPoints + prevCount] = self.previousCloud[:5, :prevCount, fNum - i]
            if numTargets > 0:
                indicesIn[0, totalPoints:totalPoints + prevCount] = self.previousCloud[5, :prevCount, fNum - i]
            totalPoints += prevCount
        if self.graphFin:
            self.plotStart = int(round(time.time() * 1000))
            self.graphFin = 0
            try:
                indicesIn = indicesIn[0, :]
            except:
                indicesIn = []
            self.get_thread = updateQTTargetThread3D(pointIn, targets, indicesIn, self.scatter, self.pcPlot,
                                                     numTargets, self.ellipsoids, self.coordStr, self.zRange, self.bbox,
                                                     self.plotByIndex.isChecked(), self.plotTracks.isChecked(),
                                                     self.boundaryBoxes[0]['checkEnable'].isChecked())
            self.get_thread.done.connect(self.graphDone)
            self.get_thread.start(priority=QThread.HighestPriority - 1)
        else:
            return

        # state trackingOverflowError
        if numTargets > 0:
            self.lastFrameHadTargets = True
        else:
            self.lastFrameHadTargets = False
        if numTargets:
            self.lastTID = targets[0, :]
        else:
            self.lastTID = []

    def graphDone(self):
        plotEnd = int(round(time.time() * 1000))
        plotTime = plotEnd - self.plotStart
        try:
            if self.frameNum > 1:
                self.averagePlot = (plotTime * 1 / self.frameNum) + (
                        self.averagePlot * (self.frameNum - 1) / self.frameNum)
            else:
                self.averagePlot = plotTime
        except ValueError:
            self.averagePlot = plotTime
        self.graphFin = 1
        plotString = 'Average Plot time: ' + str(plotTime)[:5] + ' ms'
        frameNumberString = 'Frame: ' + str(self.frameNum)
        self.frameNumDisplay.setText(frameNumberString)
        self.plotTimeDisplay.setText(plotString)

    def connectCom(self):
        # get parser
        self.parser.frameTime = self.frameTime
        # init threads and timers
        self.uart_thread = parseUartThread(self.parser)
        self.uart_thread.fin.connect(self.updateGraph)
        self.parseTimer = QTimer()
        self.parseTimer.setSingleShot(False)
        self.parseTimer.timeout.connect(self.parseData)
        try:
            uart = "COM" + self.uartCom.text()
            data = "COM" + self.dataCom.text()
            self.parser.connectComPorts(uart, data)
            self.connectStatus.setText('Connected')
            self.connectButton.setText('Disconnect')
        except Exception as e:
            print(e)
            self.connectStatus.setText('Unable to Connect')

    # Select and parse the configuration file
    def selectCfg(self):
        try:
            self.parseCfg("AOP_Custom.cfg")
        except Exception as e:
            print(e)
            print('No cfg file selected!')

    def parseCfg(self, filename):
        cfg_file = open(filename, 'r')
        self.cfg = cfg_file.readlines()
        counter = 0
        chirpCount = 0
        for line in self.cfg:
            args = line.split()
            if len(args) > 0:
                if args[0] == 'SceneryParam' or args[0] == 'boundaryBox':
                    self.boundaryLine = counter
                    self.profile['leftX'] = float(args[1])
                    self.profile['rightX'] = float(args[2])
                    self.profile['nearY'] = float(args[3])
                    self.profile['farY'] = float(args[4])
                    self.profile['bottomZ'] = float(args[5])
                    self.profile['topZ'] = float(args[6])
                    self.setBoundaryTextVals(self.profile)
                    self.boundaryBoxes[0]['checkEnable'].setChecked(True)
                elif args[0] == 'staticBoundaryBox':
                    self.staticLine = counter
                elif args[0] == 'profileCfg':
                    self.profile['startFreq'] = float(args[2])
                    self.profile['idle'] = float(args[3])
                    self.profile['adcStart'] = float(args[4])
                    self.profile['rampEnd'] = float(args[5])
                    self.profile['slope'] = float(args[8])
                    self.profile['samples'] = float(args[10])
                    self.profile['sampleRate'] = float(args[11])
                    print(self.profile)
                elif args[0] == 'frameCfg':
                    self.profile['numLoops'] = int(args[3])
                    self.profile['numTx'] = int(args[2]) + 1
                elif args[0] == 'chirpCfg':
                    chirpCount += 1
                elif args[0] == 'sensorPosition':
                    self.profile['sensorHeight'] = float(args[1])
                    print('Sensor Height from cfg = ', str(self.profile['sensorHeight']))
                    self.profile['az_tilt'] = int(args[2])
                    self.profile['elev_tilt'] = int(args[3])
                # Only used for Small Obstacle Detection
                elif args[0] == 'zoneDef':
                    zoneIdx = int(args[1])
                    minX = float(args[2])
                    maxX = float(args[3])
                    minY = float(args[4])
                    maxY = float(args[5])
                    # Offset by 3 so it is in center of screen
                    minZ = float(args[6])
                    maxZ = float(args[7])

                    self.boundaryBoxes[zoneIdx]['boundList'][0].setText(str(minX))
                    self.boundaryBoxes[zoneIdx]['boundList'][1].setText(str(maxX))
                    self.boundaryBoxes[zoneIdx]['boundList'][2].setText(str(minY))
                    self.boundaryBoxes[zoneIdx]['boundList'][3].setText(str(maxY))
                    self.boundaryBoxes[zoneIdx]['boundList'][4].setText(str(minZ))
                    self.boundaryBoxes[zoneIdx]['boundList'][5].setText(str(maxZ))
                    self.boundaryBoxes[zoneIdx]['checkEnable'].setChecked(True)

            counter += 1
        self.profile['maxRange'] = self.profile['sampleRate'] * 1e3 * 0.9 * 3e8 / (2 * self.profile['slope'] * 1e12)
        self.gz.translate(0, 0,
                          0.82 - self.profile['sensorHeight'])  # reposition the ground level to be at sensor height
        self.changeBoundaryBox()  # redraw bbox from cfg file values
        # update sensor position
        self.az_tilt.setText(str(self.profile['az_tilt']))
        self.elev_tilt.setText(str(self.profile['elev_tilt']))
        self.s_height.setText(str(self.profile['sensorHeight']))

    def sendCfg(self):
        try:
            self.parser.sendCfg(self.cfg)
            self.configSent = 1
            self.parseTimer.start(self.frameTime)  # need this line

        except Exception as e:
            print(e)
            print('No cfg file selected!')

    def startApp(self):
        self.configSent = 1
        self.parseTimer.start(self.frameTime)  # need this line

    def parseData(self):
        self.uart_thread.start(priority=QThread.HighestPriority)

    def whoVisible(self):
        if self.threeD:
            self.threeD = 0
        else:
            self.threeD = 1
        print('3d: ', self.threeD)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    screen = app.primaryScreen()
    main = Window(size=screen.size())
    main.show()
    sys.exit(app.exec_())
