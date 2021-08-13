#!/usr/bin/env python

from builtins import object

from time import sleep
from threading import Thread, Event
from pymavlink import mavutil
mavlink = mavutil.mavlink

class mavactive(object):
    """ A class for managing an active mavlink connection. """
    def __init__(self, connection, type_=mavlink.MAV_TYPE_GENERIC, autopilot=mavlink.MAV_AUTOPILOT_INVALID, base_mode=0, custom_mode=0,
                 mavlink_version=0, heartbeat_period=0.95):
        """ Initialises the program state and starts the heartbeat thread. """
        self.connection = connection
        self.type = type_
        self.autopilot = autopilot
        self.base_mode = base_mode
        self.custom_mode = custom_mode
        self.mavlink_version = mavlink_version
        self.heartbeat_period = heartbeat_period
        self._kill = Event()
        
        self._birth()

    def _birth(self):
        """ Creates and starts the heartbeat thread. """
        self._kill.clear()
        self.heartbeat_thread = Thread(target=self.heartbeat_repeat)
        self.heartbeat_thread.start()

    @property
    def is_alive(self):
        return not self._kill.is_set()

    def heartbeat_repeat(self):
        """ Sends a heartbeat to 'self.connection' with 'self.heartbeat_period'. """
        while self.is_alive:
            self.connection.mav.heartbeat_send(
                self.type,
                self.autopilot,
                self.base_mode,
                self.custom_mode,
                self.mavlink_version
            )
            sleep(self.heartbeat_period)

    def kill(self):
        """ Stops the heartbeat, if not already dead. """
        if not self.is_alive:
            return # already dead

        self._kill.set()
        self.heartbeat_thread.join()
        del self.heartbeat_thread

    def revive(self):
        """ Starts the heartbeat, if not already alive. """
        if self.is_alive:
            return # already alive

        self._birth()

    def __del__(self):
        """ End the thread cleanly on program end. """
        self.kill()
