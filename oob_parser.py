import struct
import serial
import time
import numpy as np
import math


# Initialize this Class to create a UART Parser.
# Once initialize, call connectComPorts(self, UartComPort, DataComPort) to connect to device com ports.
# Then call readAndParseUart() to read one frame of data from the device.
# The gui this is packaged with calls this every frame period.
# readAndParseUart() will return all radar detection and tracking information.
class uartParserSDK:
    def __init__(self):
        self.dataCom = None
        self.uartCom = None
        self.saveNumTxt = 0
        self.saveTextFile = 0
        self.headerLength = 52
        self.magicWord = 0x708050603040102
        self.maxPoints = 1150

        # data storage
        self.pcPolar = np.zeros((5, self.maxPoints))
        self.pcBufPing = np.zeros((5, self.maxPoints))
        self.numDetectedObj = 0
        self.targetBufPing = np.ones((10, 20)) * -1
        self.frameNum = 0
        self.missedFrames = 0
        self.byteData = bytes(1)
        self.indexes = []
        self.numDetectedTarget = 0
        self.fail = 0
        self.unique = []
        self.getUnique = 0

        # 3D people counting format
        # [frame #][header,pt cloud data,target info]
        # [][header][magic, version, packetLength, platform, frameNum, subFrameNum, chirpMargin, frameMargin,
        # uartSentTime, trackProcessTime, numTLVs, checksum]
        # [][pt cloud][pt index][#elev, azim, doppler, range, snr]
        # [][target][Target #][TID,x,y,z,vx,vy,vz,ax,ay,az,dx,dy,dz] cgary changed 10 to 13
        self.textStructCapon3D = np.zeros(1000 * 3 * self.maxPoints * 13).reshape(
            (1000, 3, self.maxPoints, 13))  # [frame #][header,pt cloud data,target info]

    # convert 3D people counting polar to 3D cartesian
    def polar2Cart3D(self):
        self.pcBufPing = np.empty((5, self.numDetectedObj))
        for n in range(0, self.numDetectedObj):
            self.pcBufPing[2, n] = self.pcPolar[0, n] * math.sin(self.pcPolar[2, n])  # z
            self.pcBufPing[0, n] = self.pcPolar[0, n] * math.cos(self.pcPolar[2, n]) * math.sin(self.pcPolar[1, n])  # x
            self.pcBufPing[1, n] = self.pcPolar[0, n] * math.cos(self.pcPolar[2, n]) * math.cos(self.pcPolar[1, n])  # y
        self.pcBufPing[3, :] = self.pcPolar[3, 0:self.numDetectedObj]  # doppler
        self.pcBufPing[4, :] = self.pcPolar[4, 0:self.numDetectedObj]  # snr

    # decode 3D People Counting Point Cloud TLV
    def parseDetectedObjects3D(self, data, tlvLength):
        objStruct = '5f'
        objSize = struct.calcsize(objStruct)
        self.numDetectedObj = int(tlvLength / 20)
        for i in range(self.numDetectedObj):
            try:
                self.pcPolar[0, i], self.pcPolar[1, i], self.pcPolar[2, i], self.pcPolar[3, i], self.pcPolar[
                    4, i] = struct.unpack(objStruct, data[:objSize])
                data = data[20:]
            except:
                self.numDetectedObj = i
                print('failed to get point cloud')
                break
        self.polar2Cart3D()

    # support for Capon 3D point cloud
    # decode Capon 3D point Cloud TLV
    def parseCapon3DPolar(self, data, tlvLength):
        pUnitStruct = '5f'  # elev, azim, doppler, range, snr
        pUnitSize = struct.calcsize(pUnitStruct)
        pUnit = struct.unpack(pUnitStruct, data[:pUnitSize])
        data = data[pUnitSize:]
        objStruct = '2bh2H'  # 2 int8, 1 int16, 2 uint16
        objSize = struct.calcsize(objStruct)
        self.numDetectedObj = int((tlvLength - pUnitSize) / objSize)
        for i in range(self.numDetectedObj):
            try:
                elev, az, doppler, ran, snr = struct.unpack(objStruct, data[:objSize])
                # print(elev, az, doppler, ran, snr)
                data = data[objSize:]
                # get range, azimuth, doppler, snr
                self.pcPolar[0, i] = ran * pUnit[3]  # range
                if az >= 128:
                    print('Az greater than 127')
                    az -= 256
                if elev >= 128:
                    print('Elev greater than 127')
                    elev -= 256
                if doppler >= 32768:
                    print('Doppler greater than 32768')
                    doppler -= 65536
                self.pcPolar[1, i] = az * pUnit[1]  # azimuth
                self.pcPolar[2, i] = elev * pUnit[0]  # elevation
                self.pcPolar[3, i] = doppler * pUnit[2]  # doppler
                self.pcPolar[4, i] = snr * pUnit[4]  # snr
            except:
                self.numDetectedObj = i
                # print('Point Cloud TLV Parser Failed ' + str(i))
                break
        self.polar2Cart3D()

    # decode Target Index TLV
    def parseTargetAssociations(self, data):
        targetStruct = 'B'
        targetSize = struct.calcsize(targetStruct)
        numIndexes = int(len(data) / targetSize)
        self.indexes = []
        self.unique = []
        try:
            for i in range(numIndexes):
                ind = struct.unpack(targetStruct, data[:targetSize])
                self.indexes.append(ind[0])
                data = data[targetSize:]
            if self.getUnique:
                uTemp = self.indexes[math.ceil(numIndexes / 2):]
                self.indexes = self.indexes[:math.ceil(numIndexes / 2)]
                for i in range(math.ceil(numIndexes / 8)):
                    for j in range(8):
                        self.unique.append(getBit(uTemp[i], j))
        except:
            print('TLV Index Parse Fail')

    # below is for labs that are compliant with SDK 3.x  This code can parse the point cloud TLV and point cloud
    # side info TLV from the OOB demo.
    # It can parse the SDK3.x Compliant People Counting demo "tracker_dpc"
    # get SDK3.x Cartesian Point Cloud
    def parseSDK3xPoints(self, dataIn, numObj):
        pointStruct = '4f'
        pointLength = struct.calcsize(pointStruct)
        try:
            for i in range(numObj):
                self.pcBufPing[0, i], self.pcBufPing[1, i], self.pcBufPing[2, i], self.pcBufPing[3, i] = struct.unpack(
                    pointStruct, dataIn[:pointLength])
                dataIn = dataIn[pointLength:]
            self.pcBufPing = self.pcBufPing[:, :numObj]
        except Exception as e:
            print('Ex ' + e)
            self.fail = 1

    # get Side Info SDK 3.x
    def parseSDK3xSideInfo(self, dataIn, numObj):
        sideInfoStruct = '2h'
        sideInfoLength = struct.calcsize(sideInfoStruct)
        try:
            for i in range(numObj):
                self.pcBufPing[4, i], unused = struct.unpack(sideInfoStruct, dataIn[:sideInfoLength])
                dataIn = dataIn[sideInfoLength:]
        except Exception as e:
            print(e)
            self.fail = 1

    # convert SDK compliant Polar Point Cloud to Cartesian
    def polar2CartSDK3(self):
        self.pcBufPing = np.empty((5, self.numDetectedObj))
        for n in range(0, self.numDetectedObj):
            self.pcBufPing[2, n] = self.pcPolar[0, n] * math.sin(self.pcPolar[2, n])  # z
            self.pcBufPing[0, n] = self.pcPolar[0, n] * math.cos(self.pcPolar[2, n]) * math.sin(self.pcPolar[1, n])  # x
            self.pcBufPing[1, n] = self.pcPolar[0, n] * math.cos(self.pcPolar[2, n]) * math.cos(self.pcPolar[1, n])  # y
        self.pcBufPing[3, :] = self.pcPolar[3, 0:self.numDetectedObj]  # doppler

    # decode SDK3.x Format Point Cloud in Polar Coordinates
    def parseSDK3xPolar(self, dataIn, tlvLength):
        pointStruct = '4f'
        pointLength = struct.calcsize(pointStruct)
        self.numDetectedObj = int(tlvLength / pointLength)
        try:
            for i in range(self.numDetectedObj):
                self.pcPolar[0, i], self.pcPolar[1, i], self.pcPolar[2, i], self.pcPolar[3, i] = struct.unpack(
                    pointStruct, dataIn[:pointLength])
                dataIn = dataIn[pointLength:]
        except:
            self.fail = 1
            return
        self.polar2CartSDK3()

    # decode 3D People Counting Target List TLV

    # 3D Struct format

    # uint32_t     tid;     /*! @brief   tracking ID */
    # float        posX;    /*! @brief   Detected target X coordinate, in m */
    # float        posY;    /*! @brief   Detected target Y coordinate, in m */
    # float        posZ;    /*! @brief   Detected target Z coordinate, in m */
    # float        velX;    /*! @brief   Detected target X velocity, in m/s */
    # float        velY;    /*! @brief   Detected target Y velocity, in m/s */
    # float        velZ;    /*! @brief   Detected target Z velocity, in m/s */
    # float        accX;    /*! @brief   Detected target X acceleration, in m/s2 */
    # float        accY;    /*! @brief   Detected target Y acceleration, in m/s2 */
    # float        accZ;    /*! @brief   Detected target Z acceleration, in m/s2 */
    # float        dimX;    /*! @brief   Detected target X spread (width), in m */
    # float        dimY;    /*! @brief   Detected target Y spread (length), in m */
    # float        dimZ;    /*! @brief   Detected target Z spread (height), in m */
    # float        ec[16];  /*! @brief   Target Error covariance matrix, [4x4 float],
    #                           in row major order, range, azimuth, elev, doppler */
    # float        g;
    # float        confidenceLevel;    /*! @brief   Tracker confidence metric*/

    def parseDetectedTracksSDK3x(self, data, tlvLength):
        targetStruct = 'I30f'  # cgary updated from 'I27f'
        targetSize = struct.calcsize(targetStruct)

        self.numDetectedTarget = int(tlvLength / targetSize)
        targets = np.empty((13, self.numDetectedTarget))
        try:
            for i in range(self.numDetectedTarget):
                targetData = struct.unpack(targetStruct, data[:targetSize])

                # tid, pos x, pos y
                targets[0:3, i] = targetData[0:3]
                # pos z
                targets[3, i] = targetData[3]
                # vel x, vel y
                targets[4:6, i] = targetData[4:6]
                # vel z
                targets[6, i] = targetData[6]
                # acc x, acc y
                targets[7:9, i] = targetData[7:9]
                # acc z
                targets[9, i] = targetData[9]
                # dim x, dim y
                targets[10:12, i] = targetData[10:12]
                # dim z
                targets[12, i] = targetData[12]
                # g
                # targets[13, i] = targetData[13]
                # confidenceLevel
                # targets[14, i] = targetData[14]
                # ec[16] error covariance
                # targets[15:18,i]=targetData[15:18]  # Chris 2020-12-18
                data = data[targetSize:]

                if self.saveTextFile:
                    self.textStructCapon3D[self.frameNum % 1000, 2, i, 0] = targets[0, i]  # TID
                    self.textStructCapon3D[self.frameNum % 1000, 2, i, 1] = targets[1, i]  # x
                    self.textStructCapon3D[self.frameNum % 1000, 2, i, 2] = targets[2, i]  # y
                    self.textStructCapon3D[self.frameNum % 1000, 2, i, 3] = targets[3, i]  # z
                    self.textStructCapon3D[self.frameNum % 1000, 2, i, 4] = targets[4, i]  # vx
                    self.textStructCapon3D[self.frameNum % 1000, 2, i, 5] = targets[5, i]  # vy
                    self.textStructCapon3D[self.frameNum % 1000, 2, i, 6] = targets[6, i]  # vz
                    self.textStructCapon3D[self.frameNum % 1000, 2, i, 7] = targets[7, i]  # ax
                    self.textStructCapon3D[self.frameNum % 1000, 2, i, 8] = targets[8, i]  # ay
                    self.textStructCapon3D[self.frameNum % 1000, 2, i, 9] = targets[9, i]  # az
                    self.textStructCapon3D[self.frameNum % 1000, 2, i, 10] = targets[10, i]  # dimX
                    self.textStructCapon3D[self.frameNum % 1000, 2, i, 11] = targets[11, i]  # dimY
                    self.textStructCapon3D[self.frameNum % 1000, 2, i, 12] = targets[12, i]  # dimZ
                    print('target added to textStructCapon3D')
        except:
            print('Target TLV parse failed')
        self.targetBufPing = targets

    # parsing for 3D People Counting lab
    def Capon3DHeader(self, dataIn):

        # reset point buffers
        self.pcBufPing = np.zeros((5, self.maxPoints))
        self.pcPolar = np.zeros((5, self.maxPoints))
        self.targetBufPing = np.zeros((13, 20))
        self.numDetectedTarget = 0
        self.numDetectedObj = 0
        self.indexes = []
        tlvHeaderLength = 8
        headerLength = 48
        # stay in this loop until we find the magic word or run out of data to parse
        while 1:
            try:
                magic, version, packetLength, platform, frameNum, subFrameNum, chirpMargin, frameMargin, \
                    uartSentTime, trackProcessTime, numTLVs, checksum = struct.unpack('Q9I2H', dataIn[:headerLength])
            except:
                self.fail = 1
                return dataIn
            if magic != self.magicWord:
                # wrong magic word, increment pointer by 1 and try again
                dataIn = dataIn[1:]
            else:
                # got magic word, proceed to parse
                break

        dataIn = dataIn[headerLength:]
        remainingData = packetLength - len(dataIn) - headerLength
        # check to ensure we have all the data
        if remainingData > 0:
            newData = self.dataCom.read(remainingData)
            dataIn += newData

        # now check TLVs
        for i in range(numTLVs):
            # try:
            # print("DataIn Type", type(dataIn))
            try:
                tlvType, tlvLength = tlvHeaderDecode(dataIn[:tlvHeaderLength])
                dataIn = dataIn[tlvHeaderLength:]
                dataLength = tlvLength - tlvHeaderLength
            except:
                # print('TLV Header Parsing Failure')
                self.fail = 1
                return dataIn
            if tlvType == 6:
                # Point cloud points. Not necessary for target plotting
                self.parseCapon3DPolar(dataIn[:dataLength], dataLength)
            if tlvType == 7:
                # target 3D
                self.parseDetectedTracksSDK3x(dataIn[:dataLength], dataLength)
            # elif tlvType == 8:
            # target index. Not necessary for target plotting
            # self.parseTargetAssociations(dataIn[:dataLength])
            dataIn = dataIn[dataLength:]
            # except Exception as e:
            #    print(e)
            #    print ('failed to read OOB SDK3.x TLV')
        if self.frameNum + 1 != frameNum:
            self.missedFrames += frameNum - (self.frameNum + 1)
        self.frameNum = frameNum
        return dataIn

    # This function is always called - first read the UART, then call a function to parse the specific demo output
    # This will return 1 frame of data. This must be called for each frame of data that is expected.
    # It will return a dict containing:
    #   1. Point Cloud
    #   2. Target List
    #   3. Target Indexes
    #   4. number of detected points in point cloud
    #   5. number of detected targets
    #   6. frame number
    #   7. Fail - if one, data is bad
    #   8. classifier output
    # Point Cloud and Target structure are liable to change based on the lab. Output is always cartesian.
    def readAndParseUart(self):
        self.fail = 0
        numBytes = 4666

        data = self.dataCom.read(numBytes)
        if self.byteData is None:
            self.byteData = data
        else:
            self.byteData += data

        self.byteData = self.Capon3DHeader(self.byteData)
        # except Exception as e:
        #    print(e)
        #    self.fail = 1
        # return data after parsing and save to replay file
        if self.fail:
            return self.pcBufPing, self.targetBufPing, self.indexes, self.numDetectedObj, \
                   self.numDetectedTarget, self.frameNum, self.fail

        return self.pcBufPing, self.targetBufPing, self.indexes, self.numDetectedObj, \
            self.numDetectedTarget, self.frameNum, self.fail

    # find various utility functions here for connecting to COM Ports, send data, etc...
    # connect to COM ports
    # Call this function to connect to the comport. This takes arguments self (intrinsic), uartCom,
    # and dataCom. No return, but sets internal variables in the parser object.
    def connectComPorts(self, uartCom, dataCom):
        self.uartCom = serial.Serial(uartCom, 115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                     timeout=0.3)
        self.dataCom = serial.Serial(dataCom, 921600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                     timeout=0.025)
        self.dataCom.reset_output_buffer()
        print('Connected')

    # send cfg over uart
    def sendCfg(self, cfg):
        for line in cfg:
            time.sleep(.03)
            self.uartCom.write(line.encode())
            ack = self.uartCom.readline()
            print(ack)
            ack = self.uartCom.readline()
            print(ack)
        time.sleep(1)
        self.uartCom.reset_input_buffer()
        self.uartCom.close()

    # send single command to device over UART Com.
    def sendLine(self, line):
        self.uartCom.write(line.encode())
        ack = self.uartCom.readline()
        print(ack)
        ack = self.uartCom.readline()
        print(ack)


# decode People Counting TLV Header
def tlvHeaderDecode(data):
    # print(len(data))
    tlvType, tlvLength = struct.unpack('2I', data)
    return tlvType, tlvLength


def getBit(byte, bitNum):
    mask = 1 << bitNum
    if byte & mask:
        return 1
    else:
        return 0
