from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem
from pyqtgraph.Qt import QtCore, QtGui


class GLTextItem(GLGraphicsItem):
    def __init__(self, X=0, Y=0, Z=0, text=None):
        GLGraphicsItem.__init__(self)

        self.GLViewWidget = None
        self.text = text
        self.X = X
        self.Y = Y
        self.Z = Z

    def setGLViewWidget(self, GLViewWidget):
        self.GLViewWidget = GLViewWidget

    def setText(self, text):
        self.text = text
        self.update()

    def setX(self, X):
        self.X = X
        self.update()

    def setY(self, Y):
        self.Y = Y
        self.update()

    def setZ(self, Z):
        self.Z = Z
        self.update()

    def setPosition(self, X, Y, Z, text):  # ,xr,yr,zr):
        self.X = X + 0.25
        self.Z = Z + 0.6
        self.Y = Y
        self.text = text  # '(' + str(X)[:4] + ', ' + str(Y)[:4] + ', ' + str(Z)[:4] + ')'
        self.update()

    def paint(self):
        # select font
        font = QtGui.QFont()
        font.setFamily("Tahoma")
        font.setPixelSize(21)
        font.setBold(True)

        self.GLViewWidget.qglColor(QtCore.Qt.white)
        self.GLViewWidget.renderText(self.X, self.Y, self.Z, self.text, font)
