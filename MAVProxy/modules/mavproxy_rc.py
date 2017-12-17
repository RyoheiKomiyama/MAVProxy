#!/usr/bin/env python
'''rc command handling'''

import time, os, struct
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module

from pythonosc import dispatcher
from pythonosc import osc_server
import threading


class RCModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(RCModule, self).__init__(mpstate, "rc", "rc command handling", public = True)
        self.override = [ 0 ] * 16
        self.last_override = [ 0 ] * 16
        self.override_counter = 0
        self.add_command('rc', self.cmd_rc, "RC input control", ['<1|2|3|4|5|6|7|8|all>'])
        self.add_command('switch', self.cmd_switch, "flight mode switch control", ['<0|1|2|3|4|5|6>'])
        if self.sitl_output:
            self.override_period = mavutil.periodic_event(20)
        else:
            self.override_period = mavutil.periodic_event(1)

        # for osc server
        self.dispatcher = dispatcher.Dispatcher()
        self.dispatcher.map("/debug_from_max", self.print_debug)
        self.dispatcher.map("/thrust_from_max", self.osc_thrust)
        self.dispatcher.map("/roll_from_max", self.osc_roll)
        self.dispatcher.map("/pitch_from_max", self.osc_pitch)
        self.dispatcher.map("/yaw_from_max", self.osc_yaw)
        self.server = osc_server.ThreadingOSCUDPServer(("127.0.0.1", 9998), self.dispatcher)
        print("Serving on {}".format(self.server.server_address))
        server_thread = threading.Thread(target=self.server.serve_forever)
        server_thread.start()

    def idle_task(self):
        if self.override_period.trigger():
            if (self.override != [ 0 ] * 16 or
                self.override != self.last_override or
                self.override_counter > 0):
                self.last_override = self.override[:]
                self.send_rc_override()
                if self.override_counter > 0:
                    self.override_counter -= 1

    def send_rc_override(self):
        '''send RC override packet'''
        if self.sitl_output:
            buf = struct.pack('<HHHHHHHHHHHHHHHH',
                              *self.override)
            self.sitl_output.write(buf)
        else:
            chan8 = self.override[:8]
            self.master.mav.rc_channels_override_send(self.target_system,
                                                           self.target_component,
                                                           *chan8)

    def cmd_switch(self, args):
        '''handle RC switch changes'''
        mapping = [ 0, 1165, 1295, 1425, 1555, 1685, 1815 ]
        if len(args) != 1:
            print("Usage: switch <pwmvalue>")
            return
        value = int(args[0])
        if value < 0 or value > 6:
            print("Invalid switch value. Use 1-6 for flight modes, '0' to disable")
            return
        if self.vehicle_type == 'copter':
            default_channel = 5
        else:
            default_channel = 8
        if self.vehicle_type == 'rover':
            flite_mode_ch_parm = int(self.get_mav_param("MODE_CH", default_channel))
        else:
            flite_mode_ch_parm = int(self.get_mav_param("FLTMODE_CH", default_channel))
        self.override[flite_mode_ch_parm - 1] = mapping[value]
        self.override_counter = 10
        self.send_rc_override()
        if value == 0:
            print("Disabled RC switch override")
        else:
            print("Set RC switch override to %u (PWM=%u channel=%u)" % (
                value, mapping[value], flite_mode_ch_parm))

    def set_override(self, newchannels):
        '''this is a public method for use by drone API or other scripting'''
        self.override = newchannels
        self.override_counter = 10
        self.send_rc_override()

    def set_override_chan(self, channel, value):
        '''this is a public method for use by drone API or other scripting'''
        self.override[channel] = value
        self.override_counter = 10
        self.send_rc_override()

    def get_override_chan(self, channel):
        '''this is a public method for use by drone API or other scripting'''
        return self.override[channel]

    def cmd_rc(self, args):
        '''handle RC value override'''
        if len(args) != 2:
            print("Usage: rc <channel|all> <pwmvalue>")
            return
        value = int(args[1])
        if value > 65535 or value < -1:
            raise ValueError("PWM value must be a positive integer between 0 and 65535")
        if value == -1:
            value = 65535
        channels = self.override
        if args[0] == 'all':
            for i in range(16):
                channels[i] = value
        else:
            channel = int(args[0])
            if channel < 1 or channel > 16:
                print("Channel must be between 1 and 8 or 'all'")
                return
            channels[channel - 1] = value
        self.set_override(channels)


    # osc
    def osc_thrust(self, unused_addr, args):
        cmd_args = [3, args]
        self.cmd_rc(cmd_args)
    def osc_roll(self, unused_addr, args):
        cmd_args = [1, args]
        self.cmd_rc(cmd_args)
    def osc_pitch(self, unused_addr, args):
        cmd_args = [2, args]
        self.cmd_rc(cmd_args)
    def osc_yaw(self, unused_addr, args):
        cmd_args = [4, args]
        self.cmd_rc(cmd_args)
    def print_debug(self, unused_addr, args):
        print("received debug")

    # stop thread




def init(mpstate):
    '''initialise module'''
    return RCModule(mpstate)