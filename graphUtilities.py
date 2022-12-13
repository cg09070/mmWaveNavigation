# functions for creating ellipsoids to represent tracks
import math
import numpy as np


# helper functions
def getSphereVertexes(xRadius=0.75, yRadius=0.75, zRadius=1.5, xc=0, yc=0, zc=0, stacks=6, sectors=6):
    stackStep = math.pi / stacks
    sectorStep = 2 * math.pi / sectors
    vertices = np.empty((3, (stacks + 1) * sectors))
    for i in range(0, stacks + 1):
        stackAngle = math.pi / 2 - i * stackStep
        xr = xRadius * math.cos(stackAngle)
        yr = yRadius * math.cos(stackAngle)
        z = zRadius * math.sin(stackAngle) + zc
        for j in range(0, sectors):
            sectorAngle = j * sectorStep
            # vertex position
            x = xr * math.cos(sectorAngle) + xc
            y = yr * math.sin(sectorAngle) + yc
            vertices[0, i * stacks + j] = x
            vertices[1, i * stacks + j] = y
            vertices[2, i * stacks + j] = z
    return vertices


def getSphereTriangles(vertices, stacks=6, sectors=6):
    trigVertices = np.empty(((stacks * sectors * 2), 3, 3))
    ind = 0
    for i in range(0, stacks):
        k1 = i * sectors
        k2 = k1 + sectors
        for j in range(0, sectors):
            k1v = k1 + j
            k1v2 = k1 + ((j + 1) % sectors)
            k2v = k2 + j
            k2v2 = k2 + ((j + 1) % sectors)
            if i != 0:
                trigVertices[ind, 0, :] = vertices[:, k1v]
                trigVertices[ind, 1, :] = vertices[:, k2v]
                trigVertices[ind, 2, :] = vertices[:, k1v2]
                ind += 1
            if i != stacks - 1:
                trigVertices[ind, 0, :] = vertices[:, k1v2]
                trigVertices[ind, 1, :] = vertices[:, k2v]
                trigVertices[ind, 2, :] = vertices[:, k2v2]
                ind += 1
    return trigVertices


def getSphereMesh(xRadius=0.75, yRadius=0.75, zRadius=0.75, xc=0, yc=0, zc=0, stacks=6, sectors=6):
    vertices = getSphereVertexes(xRadius=xRadius, yRadius=yRadius, zRadius=zRadius, xc=xc, yc=yc, zc=zc, stacks=stacks,
                                 sectors=sectors)
    return getSphereTriangles(vertices=vertices, stacks=stacks, sectors=sectors)


def getBoxVertices(xl, yl, zl, xr, yr, zr):
    vertices = np.zeros((8, 3))
    vertices[0, :] = [xl, yl, zl]
    vertices[1, :] = [xr, yl, zl]
    vertices[2, :] = [xl, yr, zl]
    vertices[3, :] = [xr, yr, zl]
    vertices[4, :] = [xl, yl, zr]
    vertices[5, :] = [xr, yl, zr]
    vertices[6, :] = [xl, yr, zr]
    vertices[7, :] = [xr, yr, zr]
    return vertices


def getBoxLinesFromVertices(vertices):
    lines = np.zeros((24, 3))
    # v0
    lines[0] = vertices[0]
    lines[1] = vertices[1]
    lines[2] = vertices[0]
    lines[3] = vertices[2]
    lines[4] = vertices[0]
    lines[5] = vertices[4]
    # v3
    lines[6] = vertices[3]
    lines[7] = vertices[1]
    lines[8] = vertices[3]
    lines[9] = vertices[2]
    lines[10] = vertices[3]
    lines[11] = vertices[7]
    # v5
    lines[12] = vertices[5]
    lines[13] = vertices[4]
    lines[14] = vertices[5]
    lines[15] = vertices[7]
    lines[16] = vertices[5]
    lines[17] = vertices[1]
    # v6
    lines[18] = vertices[6]
    lines[19] = vertices[2]
    lines[20] = vertices[6]
    lines[21] = vertices[4]
    lines[22] = vertices[6]
    lines[23] = vertices[7]
    return lines


def getBoxLines(xl, yl, zl, xr, yr, zr):
    vertices = getBoxVertices(xl, yl, zl, xr, yr, zr)
    return getBoxLinesFromVertices(vertices)


def getBoxLinesCoordinates(x, y, z, xRad=0.25, yRad=0.25, zRad=0.5):
    xl = x - xRad / 2
    xr = x + xRad / 2
    yl = y - yRad / 2
    yr = y + yRad / 2
    zl = z - zRad / 2
    zr = z + zRad / 2
    vertices = getBoxVertices(xl, yl, zl, xr, yr, zr)
    return getBoxLinesFromVertices(vertices)


def getSquareLines(xl, yL, xr, yr, z):
    vertices = np.zeros((5, 3))
    vertices[0, :] = [xl, yL, z]
    vertices[1, :] = [xr, yL, z]
    vertices[2, :] = [xr, yr, z]
    vertices[3, :] = [xl, yr, z]
    vertices[4, :] = [xl, yL, z]
    return vertices


# function to rotate a point [x,y,z] about the x-axis by an angle theta
def rotX(x, y, z, theta):
    theta = np.deg2rad(theta)  # convert to radians
    Rx = np.matrix([[1, 0, 0],
                    [0, math.cos(theta), -math.sin(theta)],
                    [0, math.sin(theta), math.cos(theta)]])
    target = np.array([x, y, z])
    rotTarget = Rx * target
    return rotTarget[0], rotTarget[1], rotTarget[2]
