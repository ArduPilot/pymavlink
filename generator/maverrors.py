#!/usr/bin/env python
'''
Define MAVError and MAVParseError
Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later
'''


class MAVError(Exception):
        '''MAVLink error class'''
        def __init__(self, msg):
            Exception.__init__(self, msg)
            self.message = msg


class MAVParseError(Exception):
    def __init__(self, message, inner_exception=None):
        self.message = message
        self.inner_exception = inner_exception
        self.exception_info = sys.exc_info()
    def __str__(self):
        return self.message
