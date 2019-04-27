import socket
import pygame
import pygame.freetype
import math
import operator
import time
import queue
import multiprocessing
import rospy

import RADARDetection
from RADARMessageProcessor import receiveAndPackAndSendRadarData
from multiprocessing import Process, Queue
from collections import deque
from socket import AF_PACKET, SOCK_RAW

pygame.init()
pygame.freetype.init()
size = width, height = 1250, 1000
maximum_distance = 100  # meters
pixel_distance_ratio = height / maximum_distance #  pixels per meter

# circle sizing
minimum_circle_size = 2
maximum_circle_size = 10
minimum_rcs = 0
maximum_rcs = 50

black = 0x00, 0x00, 0x00
white = 0xFF, 0xFF, 0xFF
red = 0xFF, 0x00, 0x00

#a fine assortment of colors
colors = []
colors.append((0xFF, 00, 00))
colors.append((00, 0xFF, 00))
colors.append((00, 00, 0xFF))
colors.append((0xFF, 0xFF, 00))
colors.append((0xFF, 00, 0xFF))
colors.append((00, 0xFF, 0xFF))

screen = pygame.display.set_mode(size)

#the detection_lifetime will keep a set of detections around for 
#n number of draw cycles. This helps visualize a path that detections take.
detection_lifetime = 1

font_path = pygame.freetype.get_default_font()
distance_text_font = pygame.freetype.SysFont("ubuntu", 16)

def DrawBackground():
    pygame.draw.rect(screen, black, (0, 0, height, height))
    for radius in short_range_distance_marks:
        pygame.draw.circle(screen, white, (int(width/2), height), int(radius * pixel_distance_ratio), 1)
        position = (int(width/2), height - 16 - int((radius*pixel_distance_ratio)))
        circle_label = str(radius) + " meters"
        distance_text_font.render_to(screen, position, circle_label, white, pygame.color.Color(0,0,0,0), pygame.freetype.STYLE_DEFAULT, 0, 16)
    return

PC_IP = "192.168.2.141"
RADAR_DESTINATION_PORTS = [31122]#, 31122, 31122, 31122, 31122, 31122, 31122]
RADAR_DESTINATION_IPS = ["225.0.0.1"]#, "226.0.0.1", "227.0.0.1", "228.0.0.1", "229.0.0.1", "230.0.0.1", "231.0.0.1"]
num_processes = range(len(RADAR_DESTINATION_PORTS))

socks = []
processes = []
short_range_queues = []

for i in range(len(RADAR_DESTINATION_PORTS)):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 32)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
    sock.bind((RADAR_DESTINATION_IPS[i], RADAR_DESTINATION_PORTS[i]))
    sock.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(PC_IP))
    sock.setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(RADAR_DESTINATION_IPS[i]) + socket.inet_aton(PC_IP))
    socks.append(sock)
    short_range_queues.append(Queue())
    processes.append(Process(target=receiveAndPackAndSendRadarData, args=(socks[-1], i, short_range_queues[i])))
    processes[-1].start()

short_range_distance_marks = []
short_range_distance_marks.append(5)
short_range_distance_marks.append(10)
short_range_distance_marks.append(25)
short_range_distance_marks.append(50)
short_range_distance_marks.append(75)

detection_buffer = deque()



while True:

    for i in range(len(RADAR_DESTINATION_PORTS)):
		#if the queue size is more than 1 this means that the drawing
		#is not keeping up with the RADAR messages.
        if len(short_range_queues) > 1:
            print("RADAR Queue is growing (This is bad)")

		#detection buffer is used for the detection_lifetime
        detection_buffer.append(short_range_queues[i].get(True))

		#pop the oldest detection list out of the buffer once it's full.
        if len(detection_buffer) > detection_lifetime:
            detection_buffer.popleft().clear()

	
        for detections in detection_buffer:
            detections.sort(key=operator.attrgetter('rcs'))
            count = 0

	    	#this is some code I was expiermenting with. Ignore I guess.
            '''sorted_by_r = []
            
            for detection in reversed(detections):
                sorted_by_r.sort(key=operator.attrgetter('r'))

                too_close = False
                for neighbor in sorted_by_r:
                    if detection.distancetoother(neighbor) < 20:
                        too_close = True

                if not too_close:
                    sorted_by_r.append(detection)
            '''

	    	#main processing
            for detection in detections:
                x = math.sin(detection.theta) * (detection.r)
                y = math.cos(detection.theta) * (detection.r)
                x = (width/2) + (x * pixel_distance_ratio)
                y = height - (y * pixel_distance_ratio)


                '''detection_height = 2 + (math.sin(detection.phi) * detection.r)
                height_channel = detection_height * 50;
                if height_channel < 0:
                    height_channel = 0
                if height_channel > 255:
                    height_channel = 255'''

				#take the absolute value of the velocity
                if detection.vrel < 0:
                    detection.vrel = detection.vrel * -1

				#scale the velocity in m/s up to 0-FF for the color channel.
				# the 13 here is arbitrary so just use whatever looks best.
                vrel_channel = (detection.vrel * 13)
                if vrel_channel < 0:
                    vrel_channel = 0
                if vrel_channel > 255:
                    vrel_channel = 255

				#scale the signal to noise ratio up to 0-FF for the color channel
                snr_channel = int(detection.snr - 50 * (255/150))
                if snr_channel < 0:
                    snr_channel = 0
                if snr_channel > 255:
                    snr_channel = 255

				#change these to visualize various detection parameters.
                red_channel = snr_channel
                blue_channel = 0x00
                green_channel = 0x00

				#uncomment this code to not bother drawing detections less than some channel threshold.
                #if red_channel <= 5 and blue_channel <= 5 and green_channel <= 5:
                #    continue

                color = red_channel, green_channel, blue_channel

				#uncomment this line to have every detection rotate through pretty colors. 
				#possibly useful if you've filtered down to the object level                
				#color = colors[ count % len(colors)]

				#scales the circle's size to the RCS value. Again, pretty much arbitrary values here.
                rcs_percentage = (detection.rcs - 0) / (50 - 0)
                size = int((rcs_percentage * (maximum_circle_size - minimum_circle_size)) + minimum_circle_size)
                if size > maximum_circle_size:
                    size = maximum_circle_size
                if size < minimum_circle_size:
                    size = minimum_circle_size
 
                pygame.draw.circle(screen, color, (int(x), int(y)), 3, 2)
				#swap to the line below to scale the detection size by the RCS value.
				#pygame.draw.circle(screen, color, (int(x), int(y)), size, 2)

    pygame.display.flip()
    pygame.draw.rect(screen, black, (0, 0, width, height))
    DrawBackground()

for i in range(len(RADAR_DESTINATION_PORTS)):
    processes[i].join()

pygame.freetype.quit()

