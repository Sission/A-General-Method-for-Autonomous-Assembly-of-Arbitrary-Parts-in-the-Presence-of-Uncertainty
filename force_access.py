#!/usr/bin/env python3

import pandas as pd
import rospy
import socket  # for socket
import struct
import os
import torch
import numpy as np
import csv
from kinematics import *
import time
from datetime import datetime
from std_msgs.msg import String, Float32
import argparse

COMMAND = 2
NUM_SAMPLES = 1
END_SIGNAL = False
START_SIGNAL = False
IDLE_SIGNAL = True

parser = argparse.ArgumentParser()
parser.add_argument("--task", required=True)
parser.add_argument("--file_name", required=False)

args = parser.parse_args()


class Request:
    def __init__(self):
        self.command_header = socket.htons(0x1234)

        self.command_header = struct.pack('<h', self.command_header)

        self.command = socket.htons(COMMAND)
        self.command = struct.pack('<h', self.command)

        self.sample_count = socket.htonl(NUM_SAMPLES)
        self.sample_count = struct.pack('<i', self.sample_count)
        self.request = self.command_header + self.command + self.sample_count


# def force_callback():

def create_socket():
    IP = '192.168.125.185'
    port = 49152
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # print("Socket successfully created")
        sock.connect((IP, port))
        # print('connected')
        return sock

    except socket.error as err:
        print("socket creation failed with error %s" % (err))


def rt_force_display(s, request):
    s.send(request)
    resp = s.recv(1024)
    temp = []
    for i in range(6):
        response = struct.unpack('>i', resp[12 + 4 * i:4 * i + 16])
        temp.append(float(response[0]) / (10 ** 6))
    time.sleep(0.5)
    print(temp)


def record_force(data, s, request):
    s.send(request)
    resp = s.recv(1024)
    temp = []
    for i in range(6):
        response = struct.unpack('>i', resp[12 + 4 * i:4 * i + 16])
        temp.append(float(response[0]) / (10 ** 6))
    data.append(temp)
    return data


def read_force():
    s = create_socket()
    request = Request().request
    t_end = time.time() + 0.5
    data = []
    while time.time() < t_end:
        s.send(request)
        resp = s.recv(1024)
        temp = []
        for i in range(6):
            response = struct.unpack('>i', resp[12 + 4 * i:4 * i + 16])
            temp.append(float(response[0]) / (10 ** 6))
        data.append(temp)
    data = np.asarray(data[:])
    measured_force = np.mean(data, axis=0)
    return measured_force


def force_collect(dc, file, roll, pitch, yaw):
    s = create_socket()
    request = Request().request

    t_end = time.time() + 3
    data = []
    while time.time() < t_end:
        s.send(request)
        resp = s.recv(1024)
        temp = []
        for i in range(6):
            response = struct.unpack('>i', resp[12 + 4 * i:4 * i + 16])
            temp.append(float(response[0]) / (10 ** 6))
        data.append(temp)
    data = np.asarray(data[:])
    measured_force = np.mean(data, axis=0)
    force_data = np.hstack((np.array([roll, pitch, yaw]), measured_force)).reshape(1, 9)
    with open(f"{dc.save_path}/{file}", "ab") as f:
        np.savetxt(f, force_data, delimiter=',')


def temp_file():
    s = create_socket()
    request = Request().request

    t_end = time.time() + 3
    data = []
    while time.time() < t_end:
        s.send(request)
        resp = s.recv(1024)
        temp = []
        for i in range(6):
            response = struct.unpack('>i', resp[12 + 4 * i:4 * i + 16])
            temp.append(float(response[0]) / (10 ** 6))
        print('temp', temp)
        data.append(temp)


def error_record(dc, roll, pitch, yaw):
    data = np.array([roll, pitch, yaw]).reshape(1, 3)
    with open(f"{dc.save_path}/error_record.csv", "ab") as f:
        np.savetxt(f, data, delimiter=',')


def force_monitor():
    rospy.init_node('force_monitor', anonymous=True)
    pub = rospy.Publisher('Tz_force', Float32, queue_size=10)

    s = create_socket()
    request = Request().request

    while True:
        s.send(request)
        resp = s.recv(1024)
        tz_force = []
        for i in range(6):
            response = struct.unpack('>i', resp[12 + 4 * i:4 * i + 16])
            tz_force.append(float(response[0]) / (10 ** 6))
        pub.publish(tz_force[2])


def force_call_back(data):
    global START_SIGNAL, END_SIGNAL, IDLE_SIGNAL
    if data.data == 'start':
        START_SIGNAL = True
        IDLE_SIGNAL = False
        END_SIGNAL = False
    elif data.data == 'end':
        END_SIGNAL = True
        START_SIGNAL = False
        IDLE_SIGNAL = True


def force_node(fields, s, dc, request, filename):
    rospy.init_node('force_recording', anonymous=True)
    rospy.Subscriber('force_signal', String, force_call_back)

    while True:
        if START_SIGNAL:
            # if info:
            format_printing('Collecting Data')
            data = []
            recording = True
            while recording:
                data = record_force(data, s, request)
                if END_SIGNAL:
                    recording = False
                    break
            data = np.asarray(data[:])
            data2 = groupedAvg(data)
            print(data2.shape)

            np.savetxt(f"{dc.save_path}/{filename}", data2, delimiter=',', header=fields)
            format_printing('Collecting Done')

        elif IDLE_SIGNAL:
            rt_force_display(s, request)


def groupedAvg(myArray, N=10):
    result = np.cumsum(myArray, 0)[N - 1::N] / float(N)
    result[1:] = result[1:] - result[:-1]
    return result


def main(arguments):
    rt_fields = 'Fx measured force, Fy measured force, Fz measured force, Tx measured force, Ty measured force, Tz measured force'
    full_fields = 'irb q4 offset, irb q5 offset, irb q6 offset', \
                  'roll uncertainty, pitch uncertainty, yaw uncertainty', \
                  'x uncertainty, y uncertainty, z uncertainty', \
                  'Fx measured force, Fy measured force, Fz measured force', \
                  'Tx measured force, Ty measured force, Tz measured force', \
                  'Fx computed force, Fy computed force, Fz computed force', \
                  'Tx computed force, Ty computed force, Tz computed force'
    s = create_socket()
    dc = DefaultConfigurations()
    request = Request().request
    if arguments.task == 'rt':
        force_node(rt_fields, s, dc, request, arguments.file_name)
    if arguments.task == 'monitor':
        force_monitor()
    if arguments.task == 'test':
        temp_file()


if __name__ == "__main__":
    main(args)
