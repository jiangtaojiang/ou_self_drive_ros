import struct
import ctypes
import datetime
import os
from decimal import Decimal
import time, sys

from RADARDetection import RADARDetection

sendStatus = False


def receiveAndPackAndSendRadarData(sock, index, short_range_q):
    nearScanTimeStamp = 0
    farScanTimeStamp = 0
    short_range_list = []
    long_range_list = []
    long_range_messages_received = []
    short_range_messages_received = []
    counter = 0
    for i in range(0, 6):
        short_range_messages_received.append(0)
        long_range_messages_received.append(0)

    while True:
        data, addr = sock.recvfrom(2000)  # buffer size is 2000 bytes
        HeaderID = int(data[0:4].encode('hex'), 16)
        #HeaderID = int.from_bytes(data[0:4], byteorder='big')
        if HeaderID == 13107200:
            continue
        elif HeaderID == 14417921 or HeaderID == 14417922 \
                or HeaderID == 14417923 or HeaderID == 14417924 or HeaderID == 14417925:
            startIndex = 21
            messageCounter = int(data[startIndex:startIndex + 1].encode('hex'), 16)
            startIndex += 1

            index = messageCounter - 1
            if (HeaderID == 14417921 or HeaderID == 14417922):
                continue #skip long range messages
            else:
                is_long_range = False
                short_range_messages_received[index] += 1

            utcTimeStamp = int(data[startIndex:startIndex + 8].encode('hex'), 16)
            startIndex += 8
            timeStamp = int(data[startIndex:startIndex + 4].encode('hex'), 16)
            startIndex += 4
            #mesurmentCounter = int.from_bytes(data[startIndex:startIndex + 4], byteorder='big', signed=False)
            startIndex += 4
            #cycleCounter = int.from_bytes(data[startIndex:startIndex + 4], byteorder='big', signed=False)
            startIndex += 4
            #nofDetections = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=False)
            startIndex += 2
            vAmbig = int(data[startIndex:startIndex + 2].encode('hex'), 16)
            startIndex += 2
            #centerFrequency = int.from_bytes(data[startIndex:startIndex + 1], byteorder='big', signed=False)
            startIndex += 1
            detectionsInPacket = int(data[startIndex:startIndex + 1].encode('hex'), 16)
            startIndex += 1
            for i in range(detectionsInPacket):
                f_Range = int(data[startIndex:startIndex + 2].encode('hex'), 16)
                #to convert to signed in python 2.7 ughhh
                if f_Range > 0x7FFF:
                    f_Range -= 0x10000
                startIndex += 2
                f_VrelRad = int(data[startIndex:startIndex + 2].encode('hex'), 16)
                if f_VrelRad > 0x7FFF:
                    f_VrelRad -= 0x10000
                startIndex += 2
                f_AzAng0 = int(data[startIndex:startIndex + 2].encode('hex'), 16)
                if f_AzAng0 > 0x7FFF:
                    f_AzAng0 -= 0x10000
                startIndex += 2
                #f_AzAng1 = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=True)
                startIndex += 2
                f_ElAng = int(data[startIndex:startIndex + 2].encode('hex'), 16)
                if f_ElAng > 0x7FFF:
                    f_ElAng -= 0x10000
                startIndex += 2
                #f_RCS0 = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=True)
                startIndex += 2
                f_RCS1 = int(data[startIndex:startIndex + 2].encode('hex'), 16)
                if f_RCS1 > 0x7FFF:
                    f_RCS1 -= 0x10000
                startIndex += 2
                #f_Prob0 = int.from_bytes(data[startIndex:startIndex + 1], byteorder='big', signed=False)
                startIndex += 1
                #f_Prob1 = int.from_bytes(data[startIndex:startIndex + 1], byteorder='big', signed=False)
                startIndex += 1
                #f_RangeVar = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=False)
                startIndex += 2
                #f_VrelRadVar = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=False)
                startIndex += 2
                #f_AzAngVar0 = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=False)
                startIndex += 2
                #f_AzAngVar1 = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=False)
                startIndex += 2
                #f_ElAngVar = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=False)
                startIndex += 2
                f_Pdh0 = int(data[startIndex:startIndex + 1].encode('hex'), 16)
                if f_Pdh0 > 0x7F:
                    f_Pdh0 -= 0x100
                startIndex += 1
                f_SNR = int(data[startIndex:startIndex + 1].encode('hex'), 16)
                if f_SNR > 0x7F:
                    f_SNR -= 0x100
                startIndex += 1
                # radarDetectionList can be appended by an object created from all the extracted values in the current for-loop iteration
                detection = RADARDetection()
                detection.r = f_Range * 0.004577776
                detection.theta = f_AzAng0 * 9.58767E-05
                detection.phi = f_ElAng * 9.58767E-05
                detection.rcs = f_RCS1 * 0.003051851
                detection.pdh = int(f_Pdh0)
                detection.snr = f_SNR
                detection.vrel = f_VrelRad * 0.004577776
                detection.long_range = is_long_range

                short_range_list.append(detection)

            packet_ready = True
            for message_rxd in short_range_messages_received:
                if message_rxd < 3:
                    packet_ready = False

            if packet_ready and len(short_range_list) > 0:
                #copy the list
                short_range_q.put_nowait(list(short_range_list))
                for i in range(0, 6):
                    short_range_messages_received[i] = 0
                #clear the list
                del short_range_list[:]


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


