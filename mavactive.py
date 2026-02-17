#!/usr/bin/env python

from builtins import object

import weakref
from time import sleep
from threading import Thread, Event, Lock
from pymavlink import mavutil, mavlink


class WriteLockedFile(object):
    """ A file with thread-locked writing. """
    def __init__(self, file):
        self._base_file = file
        self._write_lock = Lock()
        
    def write(self, *args, **kwargs):
        with self._write_lock:
            self._base_file.write(*args, **kwargs)
    
    def __getattr__(self, name):
        return getattr(self._base_file, name)
    
    def __dir__(self):
        return dir(self._base_file) + ["_base_file", "_write_lock"]


class mavactive(object):
    """ A class for managing an active mavlink connection. """
    def __init__(self, connection, type_=mavlink.MAV_TYPE_GENERIC,
                 autopilot=mavlink.MAV_AUTOPILOT_INVALID, base_mode=0,
                 custom_mode=0, mavlink_version=0, heartbeat_period=0.95):
        """ Initialises the program state and starts the heartbeat thread. """
        self.connection = connection
        self.type = type_
        self.autopilot = autopilot
        self.base_mode = base_mode
        self.custom_mode = custom_mode
        self.mavlink_version = mavlink_version
        self.heartbeat_period = heartbeat_period
        
        # replace internal file with a thread-safe one
        self.connection.mav.file = WriteLockedFile(self.connection.mav.file)
        # set up the kill event and initialise the heartbeat thread
        self._kill = Event()
        self._finalizer = weakref.finalize(self, self.kill)
        self._birth()

    def _birth(self):
        """ Creates and starts the heartbeat thread. """
        self._kill.clear()
        self.heartbeat_thread = Thread(target=self.heartbeat_repeat)
        self.heartbeat_thread.daemon = True
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
