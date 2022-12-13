from PyQt5.QtCore import QDateTime, Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QDateTimeEdit,
        QDial, QDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
        QProgressBar, QPushButton, QRadioButton, QScrollBar, QSizePolicy,
        QSlider, QSpinBox, QStyleFactory, QTableWidget, QTabWidget, QTextEdit,
        QVBoxLayout, QWidget, QFileDialog)
from PyQt5.QtGui import QPainter, QColor, QFont

import pyqtgraph as pg
import pyqtgraph.opengl as gl
import random
import numpy as np
import time

from PyQt5.QtCore import QThread, pyqtSignal
from graphUtilities import *
from cgNavigation import *


class parseUartThread(QThread):
    fin = pyqtSignal('PyQt_PyObject')

    def __init__(self, uParser):
        QThread.__init__(self)
        self.parser = uParser

    def run(self):
        pointCloud = self.parser.readAndParseUart()
        self.fin.emit(pointCloud)


class sendCommandThread(QThread):
    done = pyqtSignal()

    def __init__(self, uParser, command):
        QThread.__init__(self)
        self.parser = uParser
        self.command = command

    def run(self):
        self.parser.sendLine(self.command)
        self.done.emit()


class updateQTTargetThread3D(QThread):
    done = pyqtSignal()

    def __init__(self, pointCloud, targets, indexes, scatter, pcPlot, numTargets, ellipsoids, coordinates, zRange=None,
                 bbox=None, colorByIndex=False, drawTracks=True, bbox_en=0):
        QThread.__init__(self)
        if bbox is None:
            bbox = [0, 0, 0, 0, 0, 0]
        if zRange is None:
            zRange = [-3, 3]
        self.gw = None
        self.pointCloud = pointCloud
        self.targets = targets
        self.indexes = indexes
        self.scatter = scatter
        self.pcPlot = pcPlot
        self.colorArray = ('r', 'g', 'b', 'w')
        self.numTargets = numTargets
        self.ellipsoids = ellipsoids
        self.coordStr = coordinates
        self.zRange = zRange
        self.colorByIndex = colorByIndex
        self.drawTracks = drawTracks
        self.bbox = bbox
        self.bbox_en = bbox_en

    def drawTrack(self, index):
        # get necessary track data
        tid = int(self.targets[0, index])
        x = round(self.targets[1, index], 2)  # centroid x
        y = round(self.targets[2, index], 2)  # centroid y
        z = round(self.targets[3, index], 2)  # centroid z
        x_spread = round(self.targets[11, index], 3)  # dimension x
        y_spread = round(self.targets[10, index], 3)  # dimension y
        z_spread = round(self.targets[12, index], 3)  # dimension z
        # cgary
        edge_color = pg.glColor('g')  # default track color is green
        mesh = getBoxLinesCoordinates(x, y, z, x_spread, y_spread, z_spread)  # create track box

        try:
            if abs(x) < 1:  # if track centroid is within 1m to the left or right
                edge_color = update_edge_color(x, x_spread, y)  # change track color
            # draw track on graph
            track = self.ellipsoids[tid]
            track.setData(pos=mesh, color=edge_color, width=2, antialias=True, mode='lines')  # GLLinePlotItem
            track.setVisible(True)
            # add text coordinates
            ctext = self.coordStr[tid]
            # cgary handle navigation
            navigate(x, y, z, x_spread, edge_color, ctext)
        except Exception as e:
            print('exception' + e)

    def run(self):
        # sanity check indexes = points
        if (len(self.indexes) != np.shape(self.pointCloud)[1]) and (len(self.indexes)):
            print('I: ', len(self.indexes), ' P: ', np.shape(self.pointCloud)[1])
        # clear all previous targets
        for e in self.ellipsoids:
            if e.visible():
                e.hide()
        for c in self.coordStr:
            if c.visible():
                c.hide()
        # remove points outside boundary box
        self.bbox = [-4, 4, -0.5, 6, 0, 3]
        if self.bbox:
            to_delete = []
            for i in range(np.shape(self.pointCloud)[1]):
                x = self.pointCloud[0, i]
                y = self.pointCloud[1, i]
                z = self.pointCloud[2, i]
                if x < self.bbox[0] or x > self.bbox[1]:
                    to_delete.append(i)
                elif y < self.bbox[2] or y > self.bbox[3]:
                    to_delete.append(i)
                elif z < self.bbox[4] or z > self.bbox[5]:
                    to_delete.append(i)
            self.pointCloud = np.delete(self.pointCloud, to_delete, 1)
        # graph the points with colors
        toPlot = self.pointCloud[0:3, :].transpose()
        size = np.log2(self.pointCloud[4, :].transpose())
        colors = np.zeros((np.shape(self.pointCloud)[1], 4))
        if self.colorByIndex:
            if len(self.indexes) > 0:
                try:
                    for i in range(len(self.indexes)):
                        ind = int(self.indexes[i])
                        if ind < 100:
                            color = pg.glColor(self.colorArray[ind % 3])
                        else:
                            color = pg.glColor(self.colorArray[3])
                        colors[i, :] = color[:]
                    self.scatter.setData(pos=toPlot, color=colors, size=size)
                except:
                    self.scatter.setData(pos=toPlot, size=size)
            else:
                self.scatter.setData(pos=toPlot, size=size)
        else:
            for i in range(np.shape(self.pointCloud)[1]):
                zs = self.pointCloud[2, i]
                if (zs < self.zRange[0]) or (zs > self.zRange[1]):
                    colors[i] = pg.glColor('k')
                else:
                    colorRange = self.zRange[1] + abs(self.zRange[0])
                    zs = self.zRange[1] - zs
                    colors[i] = pg.glColor(self.gw.getColor(abs(zs / colorRange)))
            self.scatter.setData(pos=toPlot, color=colors, size=size)
        # sort and graph the targets
        if self.drawTracks:
            try:
                if self.numTargets > 3:
                    var = self.targets[self.targets[2, :].argsort()]  # sort by y coordinate
                    for i in range(len(var[1, :])):
                        if abs(var[1, i]) < 1:
                            self.drawTrack(i)
                        break
                else:
                    self.drawTrack(0)  # only draw the object closest to the sensor
            except:
                print('missing track data')
            # for i in range(self.numTargets):
            #     try:
            #         self.drawTrack(i)
            #     except:
            #         # pass
            #         print('missing track data')
            #         # print(self.numTargets)
        self.done.emit()
