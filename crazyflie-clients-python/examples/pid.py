# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.

"""
Simple example that connects to the first Crazyflie found, logs the Stabilizer
and prints it to the console. After 10s the application disconnects and exits.
"""

import sys, os
sys.path.append("../lib")

from threading import Thread

import cflib.crtp

import logging
import time
from threading import Timer

import cflib.crtp
from cfclient.utils.logconfigreader import LogConfig
from cflib.crazyflie import Crazyflie

from collections import deque


from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

class PidExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """
    DATA_NUM = 400

    log_var = [
            ("stabilizer.roll",   "float",   "radius", -10,  10),
            ("stabilizer.pitch",  "float",   "radius", -10,  10),
           # ("stabilizer.yaw",    "float",   "radius", -180, 180),
            ("stabilizer.thrust", "uint16_t", "force",  0, 65535),

            ("actuator.roll",     "int16_t",  "force",  -32768, 32767),
            ("actuator.pitch",    "int16_t",  "force",  -32768, 32767)]
    log_data = []
    log_plot = []

    param_var = [
            ("pid_attitude", "roll_kp"),
            ("pid_attitude", "roll_ki"),
            ("pid_attitude", "roll_kd"),
            ("pid_attitude", "pitch_kp"),
            ("pid_attitude", "pitch_ki"),
            ("pid_attitude", "pitch_kd"),
            ("pid_attitude", "yaw_kp"),
            ("pid_attitude", "yaw_ki"),
            ("pid_attitude", "yaw_kd")
            ]
    param_spins = []


    control_var = [ "thrust", "roll", "pitch", "yawrate" ]
    control_spins = []

    control_data = {}

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        # Create a Crazyflie object without specifying any cache dirs
        self._cf = Crazyflie()

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print "Connecting to %s" % link_uri

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        #self.thrust = 0
        #self.pitch = 0
        #self.roll = 0
        #self.yawrate = 0

        self.control_data['roll'] = 0
        self.control_data['pitch'] = 0
        self.control_data['yawrate'] = 0
        self.control_data['thrust'] = 0

        # setup gui
        self._gui()
        self._gui2()

        for i in range(len(self.log_var)):
            self.log_data.append(deque([0] * self.DATA_NUM))

        self._param_check_list = []
        self._param_groups = []

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print "Connected to %s" % link_uri

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name="Stabilizer", period_in_ms=10)

        for logname, logtype, logunit, y1, y2 in self.log_var:
            self._lg_stab.add_variable(logname, logtype)

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print "Could not start log configuration," \
                  "{} not found in TOC".format(str(e))
        except AttributeError:
            print "Could not add Stabilizer log config, bad configuration."

        for group, name in self.param_var:
            self._cf.param.add_update_callback(group = group, name = name,
                    cb = self._param_callback)
            self._cf.param.request_param_update("{0}.{1}".format(group, name))

        ## Print the param TOC
        #p_toc = self._cf.param.toc.toc
        #for group in sorted(p_toc.keys()):
        #    print "{}".format(group)
        #    for param in sorted(p_toc[group].keys()):
        #        print "\t{}".format(param)
        #        self._param_check_list.append("{0}.{1}".format(group, param))
        #    self._param_groups.append("{}".format(group))
        #    # For every group, register the callback
        #    self._cf.param.add_update_callback(group=group, name=None,
        #                                       cb=self._param_callback)

        #print
        #print "Reading back back all parameter values"
        # Request update for all the parameters using the full name
        # group.name
        #for p in self._param_check_list:
        #    self._cf.param.request_param_update(p)
        

        # Start a timer to disconnect in 10s
        #t = Timer(5, self._cf.close_link)
        #t.start()
        Thread(target=self._ramp_motors).start()

    def _param_callback(self, name, value):
        """Generic callback registered for all the groups"""
        print "{0}: {1}".format(name, value)

        # Remove each parameter from the list and close the link when
        # all are fetched

        #self._param_check_list.remove(name)
        #if len(self._param_check_list) == 0:
        #    print "Have fetched all parameter values."

        #    # First remove all the group callbacks
        #    for g in self._param_groups:
        #        self._cf.param.remove_update_callback(group=g,
        #                                              cb=self._param_callback)

        #    # Create a new random value [0.00,1.00] for pid_attitude.pitch_kd
        #    # and set it
        #    pkd = random.random()
        #    print
        #    print "Write: pid_attitude.pitch_kd={:.2f}".format(pkd)
        #    self._cf.param.add_update_callback(group="pid_attitude",
        #                                       name="pitch_kd",
        #                                       cb=self._a_pitch_kd_callback)
        #    # When setting a value the parameter is automatically read back
        #    # and the registered callbacks will get the updated value
        #    self._cf.param.set_value("pid_attitude.pitch_kd",
        #                             "{:.2f}".format(pkd))

    #def _a_pitch_kd_callback(self, name, value):
    #    """Callback for pid_attitude.pitch_kd"""
    #    print "Readback: {0}={1}".format(name, value)
    #    print

    def _ramp_motors(self):
        time.sleep(1)
        self._cf.param.set_value("pid_attitude.pitch_kd", "0.1")

        #self.pitch = 0 #sum(self.pitch_data)/len(self.pitch_data)
        #self.roll = 0 #sum(self.roll_data)/len(self.roll_data)
        #self.yawrate = 0

        #Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        while True:
            self._cf.commander.send_setpoint(
                    self.control_data['roll'],
                    self.control_data['pitch'], 
                    self.control_data['yawrate'],
                    self.control_data['thrust'])
            time.sleep(0.1)

        self._cf.commander.send_setpoint(0, 0, 0, 0)
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print "Error when logging %s: %s" % (logconf.name, msg)

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        #print "[%d][%s]: %2.4f %2.4f %2.4f" % (timestamp, logconf.name, data["stabilizer.roll"], data["stabilizer.pitch"], data["stabilizer.yaw"])
        
        for i in range(len(self.log_var)):
            self.log_data[i].append(data[self.log_var[i][0]])
            self.log_data[i].popleft()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print "Connection to %s failed: %s" % (link_uri, msg)
        self.is_connected = False
        os._exit(0)

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print "Connection to %s lost: %s" % (link_uri, msg)

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print "Disconnected from %s" % link_uri
        self.is_connected = False

    def _gui(self):
        """Setup a GUI for logging parameters"""
        print "GUI start"
        
        self.app = QtGui.QApplication([])
        self.win = pg.GraphicsWindow(title="Pid tuning.")
        self.win.resize(1000,600)

        # Enable antialiasing for prettier plots
        pg.setConfigOptions(antialias=True)

        for logname, logtype, logunit, y1, y2 in self.log_var:

            pw = self.win.addPlot(title=logname)
            self.win.nextRow()
            self.log_plot.append(pw.plot(pen = (255, 0, 0)))

            pw.showGrid(x = True, y = True)

            pw.setXRange(0, self.DATA_NUM)
            pw.setYRange(y1, y2)

            pw.setLabel('left', 'Value', units=logunit)
            pw.setLabel('bottom', 'Time', units='s')


        self.t = QtCore.QTimer()
        self.t.timeout.connect(self._updateData)
        self.t.start(50)

    def valueChanging(self, sb, value):
        for i in range(len(self.control_spins)):
            if self.control_spins[i] == sb:
                print self.control_var[i]
                self.control_data[self.control_var[i]] = value

        for i in range(len(self.param_spins)):
            if self.param_spins[i] == sb:
                print self.param_var[i]
                self._cf.param.set_value("{0}.{1}".format(self.param_var[i][0], self.param_var[i][1]), "{:.2f}".format(value))

        #self.thrust = value

    def stopClicked(self):
        self.control_data['thrust'] = 0

    def _gui2(self):

        self.win2 = QtGui.QMainWindow()
        self.win2.setWindowTitle('Settings')
        cw = QtGui.QWidget()
        layout = QtGui.QGridLayout()
        cw.setLayout(layout)
        self.win2.setCentralWidget(cw)
        self.win2.show()

# control inputs.
        for name in self.control_var:
            label = QtGui.QLabel(name)
            spin  = pg.SpinBox(value=0, int=True, dec=False, minStep=500, step=500, bounds=[0, None])
            layout.addWidget(label)
            layout.addWidget(spin)
            spin.sigValueChanging.connect(self.valueChanging)

            self.control_spins.append(spin)

        btn = QtGui.QPushButton("Stop")
        btn.clicked.connect(self.stopClicked)
        layout.addWidget(btn)

# params inputs.
        for group, name in self.param_var:
            label = QtGui.QLabel("{0}.{1}".format(group, name))
            spin  = pg.SpinBox(value = 0, bounds=[0, None], step = 0.1, minStep = 0.1)
            layout.addWidget(label)
            layout.addWidget(spin)
            spin.sigValueChanging.connect(self.valueChanging)

            self.param_spins.append(spin)


    def _updateData(self):
        for i in range(len(self.log_data)):
            self.log_plot[i].setData(
                    y = list(self.log_data[i]),
                    x = range(self.DATA_NUM))
        
if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print "Scanning interfaces for Crazyflies..."
    available = cflib.crtp.scan_interfaces()
    print "Crazyflies found:"
    for i in available:
        print i[0]

    if len(available) > 0:
        le = PidExample(available[0][0])
        QtGui.QApplication.instance().exec_()
        #le.app.exec_()
    else:
        print "No Crazyflies found, cannot run example"

    os._exit(0)
