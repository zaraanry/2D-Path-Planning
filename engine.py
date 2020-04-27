#!/usr/bin/env python

import json
import threading
from time import sleep

from game import GameEngine
from config import TIME_EPSILLON
from SimpleWebSocketServer import SimpleWebSocketServer, WebSocket

NUM_OF_OBSTACLE = 20
METHOD = 'simple'


class SimpleEcho(WebSocket):
    def handleMessage(self):
        global NUM_OF_OBSTACLE, METHOD
        if self.data == 'get':
            self.sendMessage(json.dumps(SHARED_DATA))
        elif self.data.startswith('slider_'):
            NUM_OF_OBSTACLE = int(self.data.replace('slider_', ''))
        elif self.data.startswith('method_'):
            METHOD = self.data.replace('method_', '')
        else:
            print 'recieved unknown request "%s"' % self.data

    def handleConnected(self):
        print 'socket connected - ', self.address

    def handleClose(self):
        print 'closed - ', self.address


SHARED_DATA = {}
_IS_THREAD_RUNNING = True
def thread_function(name):
    print 'start listening'
    server = SimpleWebSocketServer('', 8000, SimpleEcho, 2)
    while _IS_THREAD_RUNNING:
        server.serveonce()
    print 'stopping network connections'


GAME_SPEED = 1.0
FRAMES_LIMIT = 1000000
def main_function():
    global SHARED_DATA
    ge = GameEngine()
    for frame in range(FRAMES_LIMIT):
        ge.set_params(NUM_OF_OBSTACLE, METHOD)
        ge.cycle()
        SHARED_DATA = ge.to_json()
        sleep(TIME_EPSILLON * GAME_SPEED)


if __name__ == "__main__":
    x = threading.Thread(target=thread_function, args=(1,))
    x.start()

    try:
        main_function()
    except KeyboardInterrupt:
        print 'shutting down'
    finally:
        _IS_THREAD_RUNNING = False
        print 'wating for connection to close...'
        x.join()
        print 'exiting successfully'

