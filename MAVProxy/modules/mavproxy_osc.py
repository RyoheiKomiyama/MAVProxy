#!/usr/bin/env python
'''
osc Module
Ryo Komiyama, December 2017
'''

import os
import os.path
import sys
from pymavlink import mavutil
import errno
import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings

from pythonosc import osc_message_builder
from pythonosc import udp_client


class osc(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(osc, self).__init__(mpstate, "osc", "")
        self.status_callcount = 0
        self.boredom_interval = 10 # seconds
        self.last_bored = time.time()

        self.packets_mytarget = 0
        self.packets_othertarget = 0
        self.verbose = False

        self.osc_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),
          ])
        self.add_command('osc', self.cmd_osc, "osc module", ['status','set (LOGSETTING)'])

        # for osc
        self.client = udp_client.UDPClient("127.0.0.1", 9999)


    def usage(self):
        '''show help on command line options'''
        return "Usage: osc <status|set>"

    def cmd_osc(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "status":
            print(self.status())
        elif args[0] == "set":
            self.osc_settings.command(args[1:])
        else:
            print(self.usage())

    def status(self):
        '''returns information about module'''
        self.status_callcount += 1
        self.last_bored = time.time() # status entertains us
        return("status called %(status_callcount)d times.  My target positions=%(packets_mytarget)u  Other target positions=%(packets_mytarget)u" %
               {"status_callcount": self.status_callcount,
                "packets_mytarget": self.packets_mytarget,
                "packets_othertarget": self.packets_othertarget,
               })

    def idle_task(self):
        '''called rapidly by mavproxy'''
        now = time.time()
        if now-self.last_bored > self.boredom_interval:
            self.last_bored = now
            message = self.boredom_message()
            self.say("%s: %s" % (self.name,message))
            # See if whatever we're connected to would like to play:
            self.master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,
                                            message)


    def mavlink_packet(self, m):
        # osc
        if m.get_type() == 'ATTITUDE':
            # roll
            msg = osc_message_builder.OscMessageBuilder(address = "/roll")
            msg.add_arg(m.roll)
            msg = msg.build()
            self.client.send(msg)
            # pitch
            msg = osc_message_builder.OscMessageBuilder(address = "/pitch")
            msg.add_arg(m.pitch)
            msg = msg.build()
            self.client.send(msg)
            # yaw
            msg = osc_message_builder.OscMessageBuilder(address = "/yaw")
            msg.add_arg(m.yaw)
            msg = msg.build()
            self.client.send(msg)
            


def init(mpstate):
    '''initialise module'''
    return osc(mpstate)
