#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
from time import sleep
import numpy as np
import threading

from informer import Informer

ifm_r2e =None
ifm_e2c =None

def parse_img(message):
    relay_img(message)
    nparr = np.frombuffer(message, np.uint8)
    img = cv2.imdecode(nparr,  cv2.IMREAD_COLOR)
    # cv2.imshow('Image',img)
    # cv2.waitKey(1)

def relay_img(message):
    global ifm_e2c
    if ifm_e2c is not None:
        print('send to cloud')
        ifm_e2c.send_img(message)

def relay_cmd(message):
    global ifm_r2e
    if ifm_r2e is not None:
        ifm_r2e.send_cmd(message)

class ServerR2E(Informer):
    def img_recv(self):
        self.recv('img', parse_img)

    def send_cmd(self, message):
        self.send(message, 'cmd')

class ServerE2C(Informer):
    def send_img(self, message):
        self.send(message, 'img')
    
    def cmd_recv(self):
        self.recv('cmd', relay_cmd)

def start_r2e():
    global ifm_r2e
    ifm_r2e = ServerR2E(config = 'config.yaml')

def start_e2c():
    global ifm_e2c
    ifm_e2c = ServerE2C(config = 'config_e2c.yaml')

if __name__ == '__main__':
    start_r2e_thread = threading.Thread(
        target = start_r2e, args=()
    )
    start_e2c_thread = threading.Thread(
        target = start_e2c, args=()
    )
    start_r2e_thread.start()
    start_e2c_thread.start()

    while True:
        sleep(0.1)