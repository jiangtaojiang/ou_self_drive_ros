import struct
import ctypes
import datetime
import os
from decimal import Decimal
import time, sys

from RADARDetection import RADARDetection

sendStatus = False


def receiveAndPackAndSendRadarData(sock, index, short_range_q, long_range_q):
    nearScanTimeStamp = 0
    farScanTimeStamp = 0
    short_range_list = []
    long_range_list = []
    long_range_messages_received = []
    short_range_messages_received = []
    for i in range(0, 6):
        short_range_messages_received.append(0)
        long_range_messages_received.append(0)

    while True:
        data, addr = sock.recvfrom(2000)  # buffer size is 2000 bytes
        HeaderID = int.from_bytes(data[0:4], byteorder='big')
        SOMEIPLength = int.from_bytes(data[4:8], byteorder='big')
        ClientID = int.from_bytes(data[8:10], byteorder='big')
        SessionID = int.from_bytes(data[10:12], byteorder='big')
        ProtocolVersion = int.from_bytes(data[12:13], byteorder='big')
        InterfaceVersion = int.from_bytes(data[13:14], byteorder='big')
        MessageType = int.from_bytes(data[14:15], byteorder='big')
        ReturnCode = int.from_bytes(data[15:16], byteorder='big')
        E2EP06_CRC = int.from_bytes(data[16:18], byteorder='big')
        E2EP06_Length = int.from_bytes(data[18:20], byteorder='big')
        E2EP06_Counter = int.from_bytes(data[20:21], byteorder='big')
        if HeaderID == 13107200:
            if (sendStatus == False):
                continue
            startIndex = 21

            SensorStatus_partNumber = int.from_bytes(data[startIndex:startIndex + 8], byteorder='big', signed=False)
            startIndex += 8
            SensorStatus_assemblyPartNumber = int.from_bytes(data[startIndex:startIndex + 8], byteorder='big',
                                                             signed=False)
            startIndex += 8
            SensorStatus_swPartNumber = int.from_bytes(data[startIndex:startIndex + 8], byteorder='big', signed=False)
            startIndex += 8
            SensorStatus_serialNumber = data[startIndex:startIndex + 26]
            startIndex += 26
            SensorStatus_blVersion = int.from_bytes(data[startIndex:startIndex + 3], byteorder='big', signed=False)
            startIndex += 3
            SensorStatus_blCRC = int.from_bytes(data[startIndex:startIndex + 4], byteorder='big', signed=False)
            startIndex += 4
            SensorStatus_swVersion = int.from_bytes(data[startIndex:startIndex + 3], byteorder='big', signed=False)
            startIndex += 3
            SensorStatus_swCRC = int.from_bytes(data[startIndex:startIndex + 4], byteorder='big', signed=False)
            startIndex += 4
            SensorStatus_utcTimeStamp = int.from_bytes(data[startIndex:startIndex + 8], byteorder='big', signed=False)
            startIndex += 8
            SensorStatus_timeStamp = int.from_bytes(data[startIndex:startIndex + 4], byteorder='big', signed=False)
            startIndex += 4
            SensorStatus_currentDamping = int.from_bytes(data[startIndex:startIndex + 4], byteorder='big', signed=False)
            startIndex += 4
            SensorStatus_opState = int.from_bytes(data[startIndex:startIndex + 1], byteorder='big', signed=False)
            startIndex += 1
            SensorStatus_currentFarCF = int.from_bytes(data[startIndex:startIndex + 1], byteorder='big', signed=False)
            startIndex += 1
            SensorStatus_currentNearCF = int.from_bytes(data[startIndex:startIndex + 1], byteorder='big', signed=False)
            startIndex += 1
            SensorStatus_defective = int.from_bytes(data[startIndex:startIndex + 1], byteorder='big', signed=False)
            startIndex += 1
            SensorStatus_supplViltLimit = int.from_bytes(data[startIndex:startIndex + 1], byteorder='big', signed=False)
            startIndex += 1
            SensorStatus_SensorOffTemp = int.from_bytes(data[startIndex:startIndex + 1], byteorder='big', signed=False)
            startIndex += 1
            SensorStatus_gmMissing = int.from_bytes(data[startIndex:startIndex + 1], byteorder='big', signed=False)
            startIndex += 1
            SensorStatus_txOutReduced = int.from_bytes(data[startIndex:startIndex + 1], byteorder='big', signed=False)
            startIndex += 1
            SensorStatus_maximumRangeFar = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big',
                                                          signed=False)
            startIndex += 2
            SensorStatus_maximumRangeNear = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big',
                                                           signed=False)

            # Status Message fields are received

        elif HeaderID == 14417921 or HeaderID == 14417922 \
                or HeaderID == 14417923 or HeaderID == 14417924 or HeaderID == 14417925:
            startIndex = 21
            messageCounter = int.from_bytes(data[startIndex:startIndex + 1], byteorder='big', signed=False)
            startIndex += 1

            index = messageCounter - 1
            if (HeaderID == 14417921 or HeaderID == 14417922):
                is_long_range = True
                long_range_messages_received[index] += 1
            else:
                is_long_range = False
                short_range_messages_received[index] += 1

            utcTimeStamp = int.from_bytes(data[startIndex:startIndex + 8], byteorder='big', signed=False)
            startIndex += 8
            timeStamp = int.from_bytes(data[startIndex:startIndex + 4], byteorder='big', signed=False)
            startIndex += 4
            mesurmentCounter = int.from_bytes(data[startIndex:startIndex + 4], byteorder='big', signed=False)
            startIndex += 4
            cycleCounter = int.from_bytes(data[startIndex:startIndex + 4], byteorder='big', signed=False)
            startIndex += 4
            nofDetections = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=False)
            startIndex += 2
            vAmbig = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=True)
            startIndex += 2
            centerFrequency = int.from_bytes(data[startIndex:startIndex + 1], byteorder='big', signed=False)
            startIndex += 1
            detectionsInPacket = int.from_bytes(data[startIndex:startIndex + 1], byteorder='big', signed=False)
            startIndex += 1
            for i in range(detectionsInPacket):
                f_Range = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=False)
                startIndex += 2
                f_VrelRad = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=True)
                startIndex += 2
                f_AzAng0 = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=True)
                startIndex += 2
                f_AzAng1 = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=True)
                startIndex += 2
                f_ElAng = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=True)
                startIndex += 2
                f_RCS0 = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=True)
                startIndex += 2
                f_RCS1 = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=True)
                startIndex += 2
                f_Prob0 = int.from_bytes(data[startIndex:startIndex + 1], byteorder='big', signed=False)
                startIndex += 1
                f_Prob1 = int.from_bytes(data[startIndex:startIndex + 1], byteorder='big', signed=False)
                startIndex += 1
                f_RangeVar = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=False)
                startIndex += 2
                f_VrelRadVar = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=False)
                startIndex += 2
                f_AzAngVar0 = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=False)
                startIndex += 2
                f_AzAngVar1 = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=False)
                startIndex += 2
                f_ElAngVar = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=False)
                startIndex += 2
                f_Pdh0 = int.from_bytes(data[startIndex:startIndex + 1], byteorder='big', signed=False)
                startIndex += 1
                f_SNR = int.from_bytes(data[startIndex:startIndex + 1], byteorder='big', signed=False)
                startIndex += 1
                # radarDetectionList can be appended by an object created from all the extracted values in the current for-loop iteration
                detection = RADARDetection()
                detection.r = f_Range * 0.004577776
                detection.theta = f_AzAng1 * 9.58767E-05
                detection.phi = f_ElAng * 9.58767E-05
                detection.rcs = f_RCS0 * 0.003051851
                detection.pdh = int(f_Pdh0)
                detection.snr = f_SNR
                detection.long_range = is_long_range;
                if is_long_range:
                    long_range_list.append(detection)
                else:
                    short_range_list.append(detection)


            #this forces the thread to wait until the UI has cleared out the queue before adding more points. Probably
            #not a good idea to have this in the production code

            packet_ready = True
            for message_rxd in short_range_messages_received:
                if message_rxd < 3:
                    packet_ready = False

            if packet_ready and len(short_range_list) > 0:
                short_range_q.put(short_range_list.copy())
                for i in range(0,6):
                    short_range_messages_received[i] = 0
                short_range_list.clear()

            packet_ready = True
            for message_rxd in long_range_messages_received:
                if message_rxd < 3:
                    packet_ready = False

            if packet_ready and len(long_range_list) > 0:
                long_range_q.put(long_range_list.copy())
                for i in range(0, 6):
                    long_range_messages_received[i] = 0
                long_range_list.clear()



            if (HeaderID == 14417921 or HeaderID == 14417922):

                #----FAR SCAN --- DO NOTHING
                is_long_range = True
                if (farScanTimeStamp > utcTimeStamp):
                    continue;
                if (farScanTimeStamp < utcTimeStamp):
                    # FarFullScan is complete (all the packets related to far scan has been received)
                    farScanTimeStamp = utcTimeStamp
                #if (HeaderID == 14417921):
                    # radarFarFullScanMessage.far0 should be appended with the received packet
                #else:
                    # radarFarFullScanMessage.far1 should be appended with the received packet
            else:
                is_long_range = False


                if (nearScanTimeStamp > utcTimeStamp):
                    continue;
                if (nearScanTimeStamp < utcTimeStamp):
                    # NearFullScan is complete (all the packets related to far scan has been received)
                    nearScanTimeStamp = utcTimeStamp
                #if (HeaderID == 14417923):
                # radarNearFullScanMessage.near0 should be appended with the received packet
                #elif (HeaderID == 14417924):
                # radarNearFullScanMessage.near1 should be appended with the received packet
                #else:
            # radarNearFullScanMessage.near2 should be appended with the received packet




        else:
            print("Invalid header ID received from radar unit.")


