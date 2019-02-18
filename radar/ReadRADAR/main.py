import socket
import pygame
import pygame.freetype
import math
import time
import queue
import multiprocessing

from RADARMessageProcessor import receiveAndPackAndSendRadarData
from multiprocessing import Process, Queue
from socket import AF_PACKET, SOCK_RAW

pygame.init()
pygame.freetype.init()
size = width, height = 1250, 1000
maximum_distance = 175  # meters
pixel_distance_ratio = height / maximum_distance #  pixels per meter

# circle sizing
minimum_circle_size = 1
maximum_circle_size = 5
minimum_rcs = 50
maximum_rcs = 150

black = 0, 0, 0
white = 0xFF, 0xFF, 0xFF
red = 0xFF, 0x0, 0x0

screen = pygame.display.set_mode(size)

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
long_range_queues = []

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
    long_range_queues.append(Queue())
    processes.append(Process(target=receiveAndPackAndSendRadarData, args=(socks[-1], i, short_range_queues[i], long_range_queues[i])))
    processes[-1].start()

short_range_distance_marks = []
short_range_distance_marks.append(10)
short_range_distance_marks.append(20)
short_range_distance_marks.append(50)
short_range_distance_marks.append(100)
short_range_distance_marks.append(150)


while True:
    count = 0
    for i in range(len(RADAR_DESTINATION_PORTS)):
        detections = short_range_queues[i].get(True)
        for detection in detections:

            x = math.sin(detection.theta) * (detection.r)
            y = math.cos(detection.theta) * (detection.r)
            x = (width/2) + (x * pixel_distance_ratio)
            y = height - (y * pixel_distance_ratio)
            count += 1
            if detection.long_range:
                pygame.draw.circle(screen, red, (int(x), int(y)), 3, 1)
            else:
                pygame.draw.circle(screen, white, (int(x), int(y)), 3, 1)
        detections.clear()

        detections = long_range_queues[i].get(True)
        for detection in detections:

            x = math.sin(detection.theta) * (detection.r)
            y = math.cos(detection.theta) * (detection.r)
            x = (width / 2) + (x * pixel_distance_ratio)
            y = height - (y * pixel_distance_ratio)
            count += 1
            if detection.long_range:
                pygame.draw.circle(screen, red, (int(x), int(y)), 3, 1)
            else:
                pygame.draw.circle(screen, white, (int(x), int(y)), 3, 1)
        detections.clear()


    pygame.display.flip()
    pygame.draw.rect(screen, black, (0, 0, width, height))
    for radius in short_range_distance_marks:
        pygame.draw.circle(screen, white, (int(width / 2), height), int(radius * pixel_distance_ratio), 1)
        position = (int(width / 2), height - 16 - int((radius * pixel_distance_ratio)))
        distance_text_font.render_to(screen, position, str(radius) + "m", white, pygame.color.Color(0, 0, 0, 0),
            pygame.freetype.STYLE_DEFAULT, 0, 16)



for i in range(len(RADAR_DESTINATION_PORTS)):
    processes[i].join()

pygame.freetype.quit()

