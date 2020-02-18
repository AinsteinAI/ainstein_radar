# mmwave_CLI.py
#
# This module connect to a TI xWR radar Soc via UART and sends configuration commands
# that are read by TI's command line interface (CLI).

import serial
import struct
import time
import getopt
import sys
from collections import OrderedDict
import numpy as np

def send_config(config_file, cmd_port, cmd_baud, silent, in_background):

    CLIport = serial.Serial(cmd_port, cmd_baud)
        # Check that the serial port is open
    if CLIport.isOpen() != True:
        print "Command Port didn't open, can't send configuration to EVM"

        # if we're running as a background process, assume that we're running
        # this script automatically on bootup, so keep trying to open the
        # serial port in case we're still just waiting for the sensor to
        # start up
        if in_background:
            # update the room status to say we're waiting for the sensor

            # if we need to indicate that the sensor isn't
            # talking to us, do nothing else but create a
            # file to indicate this status
            stuck = open(NoSensorFile, "w+")
            stuck.write("I'm waiting to get a connection with the sensor.")
            stuck.close

            while not CLIport.isOpen():
                time.sleep(1)

            # When we exit the while loop, it means the port is now open.
            # Remove the status file and then move on
            os.remove(NoSensorFile)
        else:
            # if we're not running as a background process, just
            # stop immediately
            return False

    # read config file
    config = [line.rstrip('\r\n') for line in open(config_file)]

    # write config file to the EVM via CLI
    print "Writing configuration to radar..."

    for i in config:
        CLIport.write(i+'\n')

        if not silent:
            print i

            time.sleep(0.2)
            # read response from sensor
            bytesAvailable = CLIport.inWaiting()
            CLIresponse = CLIport.read(bytesAvailable)
            print CLIresponse


        time.sleep(0.5)
    return True


def display_data(port, baud):
    uart = serial.Serial(port, baud)
    # Check that the serial port is open
    if uart.isOpen() != True:
        print "Data Port didn't open, can't receive data from EVM"
        return False

    # standard setup
    uart.flushInput()
    uart.flushOutput()

    count = 0
    b = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

    while (True):
        byte = struct.unpack('B', uart.read(1))[0]
        b[count] = byte
        count += 1
        if count >= len(b):
            for i in b:
                print hex(i) + " ",

            print ""
            count = 0


class PeopleCountingDemoTLVs:
    def return_TLV_struct(self, TLV_number):
        if TLV_number == self.MMWDEMO_OUTPUT_MSG_POINT_CLOUD:
            ret_dict = self.pointStruct
        elif TLV_number == self.MMWDEMO_OUTPUT_MSG_TARGET_LIST:
            ret_dict = self.targetStruct
        elif TLV_number == self.MMWDEMO_OUTPUT_MSG_TARGET_INDEX:
            ret_dict = self.targetIndexStruct
        return ret_dict

    def __init__(self):
        self.MMWDEMO_OUTPUT_MSG_POINT_CLOUD = 6
        self.pointStruct = OrderedDict()
        self.pointStruct['range']   = ['float', 4] # Range, in m
        self.pointStruct['angle']   = ['float', 4] # Azimuth angle, in rad
        self.pointStruct['elev']    = ['float', 4] # Elevation, in rad
        self.pointStruct['doppler'] = ['float', 4] # Doplper, in m/s
        self.pointStruct['snr']     = ['float', 4] # SNR, ratio

        self.MMWDEMO_OUTPUT_MSG_TARGET_LIST = 7
        self.targetStruct = OrderedDict()
        self.targetStruct['tid']  = ['uint32', 4]    # Track ID
        self.targetStruct['posX'] = ['float', 4]     # Target position in X dimension, m
        self.targetStruct['posY'] = ['float', 4]     # Target position in Y dimension, m
        self.targetStruct['posZ'] = ['float', 4]     # Target position in Z dimension, m
        self.targetStruct['velX'] = ['float', 4]     # Target velocity in X dimension, m/s
        self.targetStruct['velY'] = ['float', 4]     # Target velocity in Y dimension, m/s
        self.targetStruct['velZ'] = ['float', 4]     # Target velocity in Z dimension, m/s
        self.targetStruct['accX'] = ['float', 4]     # Target acceleration in X dimension, m/s
        self.targetStruct['accY'] = ['float', 4]     # Target acceleration in Y dimension, m/s
        self.targetStruct['accZ'] = ['float', 4]     # Target acceleration in Z dimension, m/s
        # targetStruct['EC']   = ['float', 16*4]   # Tracking error covariance matrix, [3x3], in range/angle/doppler coordinates
        # targetStruct['G']    = ['float', 4]     # Gating function gain

        self.MMWDEMO_OUTPUT_MSG_TARGET_INDEX = 8
        self.targetIndexStruct = OrderedDict()
        self.targetIndexStruct['ind'] = ['uint8' ,1]


class PeopleCountingCLI:
    def validateChecksum(self, header): # TODO: move this outside the class
        count = 0

        h = []

        for i in range(len(header) - 1):
            count += 1
            if (count > 1):
                h.append(struct.unpack("<H", header[i:i+2])[0])
                count = 0

        a = np.int32(0)
        for i in range(len(h)):
            a += h[i]
        b = np.uint16(a)

        return np.uint16(~b)

    def readToStruct(self, S, ByteArray, start): # TODO: move this outside the class
        offset = start
        R = OrderedDict()

        for key, value in S.items():
            [fieldType, fieldLength] = value

            if (fieldType == 'uint64'):
                R[key] = struct.unpack("<Q", ByteArray[offset:offset+fieldLength])[0]
            elif (fieldType == 'uint32'):
                R[key] = struct.unpack("<I", ByteArray[offset:offset+fieldLength])[0]
            elif (fieldType == 'uint16'):
                R[key] = struct.unpack("<H", ByteArray[offset:offset+fieldLength])[0]
            elif (fieldType == 'uint8'):
                R[key] = struct.unpack("<B", ByteArray[offset:offset+fieldLength])[0]
            elif (fieldType == 'float'):
                R[key] = struct.unpack("<f", ByteArray[offset:offset+fieldLength])[0]
            offset += fieldLength

        return R


    def lengthFromStruct(self, S): # TODO: move this outside the class

        length = 0
        fieldLength = 0

        for key, value in S.items():
            fieldLength = value[1]
            length += fieldLength

        return length


    def read_data(self,silent, TLVs_to_use):
        # Data Structures
        syncPattern = [2,1,4,3,6,5,8,7]
        syncPatternUINT64 = np.uint64(np.uint16([0x0102, 0x0304, 0x0506, 0x0708]))
        syncPatternUINT8 = '\x02\x01\x04\x03\x06\x05\x08\x07'
        frameNumber = 1
        frameHeaderLengthInBytes = self.lengthFromStruct(self.frameHeaderStructType)
        tlvHeaderLengthInBytes = self.lengthFromStruct(self.tlvHeaderStruct)

        maxBytesAvailable = 0
        gotHeader = False
        lostSync = False
        targetFrameNum = 0
        frameNum = 1


        tlvs = PeopleCountingDemoTLVs() # TODO: abstract this; maybe pass a class name to this function instead?

        # Prepare the structure to hold the radar data that the user wants
        RadarOutput = {}
        for k in TLVs_to_use:
            RadarOutput.update({k: []})

        #standard setup
        uart = serial.Serial(self.port, self.baud)
        uart.flushInput()
        uart.flushOutput()

        while (uart.isOpen() == True):
            while not lostSync:
                # clear out data; assuming it's been process/saved by the callback already
                for k in TLVs_to_use:
                    RadarOutput[k] = []

                bytesAvailable = uart.inWaiting()
                if (bytesAvailable > maxBytesAvailable):
                    maxBytesAvailable = bytesAvailable

                if  not gotHeader:
                    # read header byte first
                    rxHeader = uart.read(frameHeaderLengthInBytes)
                    byteCount = len(rxHeader)

                # set magicBytes
                magicBytes = []
                for i in range(8):
                    magicBytes.append(struct.unpack('B',rxHeader[i])[0])

                if not np.array_equal(magicBytes, syncPattern):
                    print "array isn't equal, magicBytes = "+str(magicBytes)+", syncPattern = "+str(syncPattern)
                    reason = "No SYNC Pattern"
                    lostSync = True
                    break

                if (len(rxHeader) != frameHeaderLengthInBytes):
                    reason = "Header Size is wrong"
                    lostSync = True
                    break

                if not self.validateChecksum(rxHeader):
                    reason = "Header Checksum is wrong"
                    lostSync = True
                    break

                frameHeader = self.readToStruct(self.frameHeaderStructType, rxHeader, 0)

                if gotHeader:
                    if (frameHeader['frameNumber'] > targetFrameNum):
                        targetFrameNum = frameHeader['frameNumber']
                        print "Found sync at frame " + str(targetFrameNum) + "(" + str(frameNum) + ")"
                        gotHeader = False
                    else:
                        print "frameNumber = " + str(frameHeader['frameNumber'])
                        reason = "Old Frame"
                        gotHeader = False
                        lostSync  = True
                        break

                # We have a valid header
                targetFrameNum = frameHeader['frameNumber']

                dataLength = frameHeader['packetLength'] - frameHeaderLengthInBytes
                if not silent:
                    print "frameHeader['packetLength'] = "+str(frameHeader['packetLength'])

                numInputPoints = 0
                numTargets = 0

                if (dataLength > 0):
                    if not silent:
                        print "dataLength = " + str(dataLength) + ", frameHeaderLength = " + str(frameHeaderLengthInBytes)
                    # Read all packet
                    rxData = uart.read(dataLength)
                    byteCount = len(rxData)

                    if (byteCount != dataLength):
                        reason = "Data Size is wrong"
                        lostSync = True
                        break

                    offset = 0

                    # TLV Parsing
                    for nTlv in range(frameHeader['numTLVs']):
                        tlvType   = struct.unpack("<I", rxData[offset:offset+4])[0]
                        tlvLength = struct.unpack("<I", rxData[offset+4:offset+8])[0]

                        if ((tlvLength + offset) > dataLength):
                            reason = "TLV Size is wrong"
                            lostSync = True
                            break
                        offset = offset + tlvHeaderLengthInBytes
                        valueLength = tlvLength - tlvHeaderLengthInBytes

                        for type in TLVs_to_use:
                            if (tlvType == type): # check to see if the user wants to save this type of message
                                s = tlvs.return_TLV_struct(type)
                                s_length = self.lengthFromStruct(s)
                                numItems = valueLength/s_length
                                for n in range( numItems ):
                                    RadarOutput[type].append(self.readToStruct(s, rxData, offset))
                                    offset += s_length
                            else:
                                offset += valueLength

                    if not silent:
                        print str(RadarOutput)
                    ###############################################
                    # Run "application" code after data is received
                    ###############################################
                    self.callback_fcn(RadarOutput)

                # Cleanup the While Loop
                frameNum += 1
                if (frameNum > 10000):
                    frameNum = 1

                # Determine if we're reading from EVM too slowly
                if (bytesAvailable > 32000):
                    runningSlow = False
                else:
                    runningSlow = True

                # Pause for a moment only if we're not running slow
                if not runningSlow:
                    time.sleep(0.01)
                # end Second While Loop

            if targetFrameNum:
                bytesAvailable = uart.inWaiting()
                print "Lost sync at frame " + str(targetFrameNum) + "(" + str(frameNum) + "), Reason: " + reason + ", " + str(bytesAvailable) + " bytes in Rx buffer"
            print "Lost sync at frame " + str(targetFrameNum) + "(" + str(frameNum) + "), Reason: " + reason + ", " + str(bytesAvailable) + " bytes in Rx buffer"

            while lostSync:

                while lostSync:
                    count = 0

                    for i in range(8):
                        byte = struct.unpack('<B', uart.read(1))[0]
                        count += 1

                        if (byte != syncPattern[i]):
                            break

                    if (count == 8):
                        lostSync = False
                        frameNum += 1

                        if (frameNum > 10000):
                            frameNum = 1

                        header = uart.read(frameHeaderLengthInBytes - 8)
                        rxHeader = syncPatternUINT8 + header
                        byteCount += 8
                        gotHeader = True


    def __init__(self, port, baud, callback):
        self.port = port
        self.baud = baud

        self.callback_fcn = callback

        # define mmWave common TLV data structures
        self.frameHeaderStructType = OrderedDict()
        self.frameHeaderStructType['sync']               = ['uint64', 8] # See SyncPattern
        self.frameHeaderStructType['version']            = ['uint32', 4]
        self.frameHeaderStructType['platform']           = ['uint32', 4]
        self.frameHeaderStructType['timestamp']          = ['uint32', 4] # 600MHz clocks
        self.frameHeaderStructType['packetLength']       = ['uint32', 4] # In bytes, including header
        self.frameHeaderStructType['frameNumber']        = ['uint32', 4] # Starting from 1
        self.frameHeaderStructType['subframeNumber']     = ['uint32', 4]
        self.frameHeaderStructType['chirpMargin']        = ['uint32', 4] # Chirp Processing margin, in ms
        self.frameHeaderStructType['frameMargin']        = ['uint32', 4] # Frame Processing margin, in ms
        self.frameHeaderStructType['uartSentTime']       = ['uint32', 4] # Time spent to send data, in ms
        self.frameHeaderStructType['trackProcessTime']   = ['uint32', 4] # Tracking Processing time, in ms
        self.frameHeaderStructType['numTLVs']            = ['uint16', 2] # Number of TLVs in thins frame
        self.frameHeaderStructType['checksum']           = ['uint16', 2]  # Header checksum

        self.tlvHeaderStruct = OrderedDict()
        self.tlvHeaderStruct['type']   = ['uint32', 4]  # TLV object Type
        self.tlvHeaderStruct['length'] = ['uint32', 4]  # TLV object length, in bytes, including TLV header
