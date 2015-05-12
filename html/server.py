#!/usr/bin/env python
# -*- coding: utf8 -*-

import tornado.web
import tornado.websocket
import tornado.ioloop
import os
import webbrowser
import sys

class ClientTracker:
    def __init__(self, should_log):
        self.clients = []
        self.clients_id = {}
        self.should_log = should_log

    def broadcast(self, message):
        client_idx = 0
        for client in self.clients:
            client_idx += 1
            if (self.should_log):
                print("Broadcasting to client " + str(client_idx))
            client.write_message(message)

    def add(self, client):
        self.clients.append(client)
        self.clients_id[client] = len(self.clients)
        if (self.should_log):
            print("Connecting client " + str(len(self.clients)))

    def remove(self, client):
        self.clients.remove(client)
        if (self.should_log):
            print("Closing client " + str(self.clients_id[client]))
        self.clients_id.pop(client, None)

class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("websocket_test.html")
 
class WebSocketHandler(tornado.websocket.WebSocketHandler):
    def check_origin(self, origin):
        return True

    def initialize(self, client_tracker):
        print("Initializing websocket handler")
        self.client_tracker = client_tracker

    def on_message(self, message):  
        print("message received: " + message)
        self.client_tracker.broadcast(message)

    def open(self):
        self.client_tracker.add(self) 
      
    def on_close(self):
        self.client_tracker.remove(self)


class SimulatorGUI:
    def __init__(self, address, port):
        self.client_tracker = ClientTracker(True)
        suffix = "main"
        self.port = port
        if getattr(sys, 'frozen', False):
            # frozen
            dir_ = os.path.dirname(sys.executable)
        else:
            # unfrozen
            dir_ = os.path.dirname(__file__)
        handlers = [
            (r"/"+suffix, MainHandler),
            (r"/",        WebSocketHandler, dict(client_tracker=self.client_tracker)),
        ]
        settings = {
             "static_path": os.path.join(os.path.dirname(dir_), "static"),
        }
        self.application = tornado.web.Application(handlers, **settings)
        self.url = address + ":" + str(port) + "/" + suffix

    def open_html_page_in_browser(self, url):
        # Open in a new tab, if possible
        webbrowser.open(url,new=2)

    def run(self):
        self.application.listen(self.port)
        self.mainLoop = tornado.ioloop.IOLoop.instance()
        #self.open_html_page_in_browser(self.url)
        print("Initialized: now starting main server loop.")
        self.mainLoop.start()
        

if __name__ == "__main__":
    #gui = SimulatorGUI("http://127.0.0.1", 9002)
    gui = SimulatorGUI("http://130.66.124.225", 9002)
    gui.run()
