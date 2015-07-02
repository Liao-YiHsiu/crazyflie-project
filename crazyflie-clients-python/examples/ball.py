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
import Traceball

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

class PID:
    kp = 0
    ki = 0
    kd = 0
    sum_error = 0;
    last_error = 0;
    
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
    
    def push(self, error):
        self.sum_error += error
        out = error * self.kp + self.sum_error * self.ki + (error - self.last_error) * self.kd
        self.last_error = error

        return out

    def clear(self):
        self.sum_error = 0
        self.last_error = 0


class PidExample:
    """
    csiot final
    """
    DATA_NUM = 400
    SAFE_ROLL = 15
    SAFE_PITCH = 15
    MAX_THRUST = 65530

    log_var = [
            ("stabilizer.roll",   "float",   "radius", -10,  10),
            ("stabilizer.pitch",  "float",   "radius", -10,  10),
            ("stabilizer.yaw",    "float",   "radius", -180, 180),
            ("stabilizer.thrust", "uint16_t", "force",  0, 65535)

           # ("baro.asl", "float", "pa",  0, 300),
           # ("baro.aslRaw", "float", "pa",  0, 300),

           # ("motor.m1", "int32_t", "power", 0, 65535),
           # ("motor.m2", "int32_t", "power", 0, 65535),
           # ("motor.m3", "int32_t", "power", 0, 65535),
           # ("motor.m4", "int32_t", "power", 0, 65535),

           # ("actuator.roll",     "int16_t",  "force",  -32768, 32767),
           # ("actuator.pitch",    "int16_t",  "force",  -32768, 32767)
            ]

    log_data = []
    log_plot = []

    # tuned parameters
    param_var = [
            ("pid_attitude", "roll_kp",  3.8),
            ("pid_attitude", "roll_ki",  80),
            ("pid_attitude", "roll_kd",  0),
            ("pid_attitude", "pitch_kp", 3.8),
            ("pid_attitude", "pitch_ki", 80),
            ("pid_attitude", "pitch_kd", 0),
            ("pid_attitude", "yaw_kp",   3.5),
            ("pid_attitude", "yaw_ki",   5.0),
            ("pid_attitude", "yaw_kd",   0),
            ]

    stop = True

    def __init__(self, link_uri, tb):
        """ Initialize and run the example with the specified link_uri """

        # Create a Crazyflie object without specifying any cache dirs
        self._cf = Crazyflie()

        self._tb = tb

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print "Connecting to %s" % link_uri

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # setup gui
        self._gui()
        self._gui2()

        for i in range(len(self.log_var)):
            self.log_data.append(deque([0] * self.DATA_NUM))


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

        for group, name, value in self.param_var:
            self._cf.param.add_update_callback(group = group, name = name,
                    cb = self._param_callback)
            # initial values
            self._cf.param.set_value("{0}.{1}".format(group, name), "{:.2f}".format(value))


        Thread(target=self._ramp_motors).start()

    def _param_callback(self, name, value):
        """Generic callback registered for all the groups"""
        print "{0}: {1}".format(name, value)


    def _ramp_motors(self):

        time.sleep(1)

        #Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        thrust  = 0
        roll    = 0
        pitch   = 0
        yawrate = 0

        thrust_pid = PID(10, 1000, 0)

        while True:
            self._cf.commander.send_setpoint(self.roll, self.pitch, self.yawrate, self.thrust)

            if self.stop:
                thrust  = 0
                roll    = 0
                pitch   = 0
                yawrate = 0
                thrust_pid.clear()
                time.sleep(0.1)
                continue;

            # get current (x, y, z)
            (x, y, z) = self.tb.get()

            thrust = thrust_pid.push(self.target_y - y)

            if thrust > self.MAX_THRUST:
                thrust = self.MAX_THRUST
            if thrust < 0:
                thrust = 0

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

        roll   = data['stabilizer.roll']
        pitch  = data['stabilizer.pitch']

        #safe zone testing...
        if abs(roll) > self.SAFE_ROLL or \
                abs(pitch) > self.SAFE_PITCH :
            self.stopClicked()


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


    def _gui2(self):

        self.win2 = QtGui.QMainWindow()
        self.win2.setWindowTitle('Human Control')
        cw = QtGui.QWidget()
        layout = QtGui.QGridLayout()
        cw.setLayout(layout)
        self.win2.setCentralWidget(cw)
        self.win2.show()

        btn = QtGui.QPushButton("Stop")
        btn.clicked.connect(self.stopClicked)
        layout.addWidget(btn)

        btn2 = QtGui.QPushButton("Start")
        btn2.clicked.connect(self.startClicked)
        layout.addWidget(btn2)

    def stopClicked(self):
        self.stop = True

    def startClicked(self):
        (x, y, z) = self.tb.get()
        self.target_y = y
        self.stop = False

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
        tb = Traceball.Traceball(0)
        tb.start()

        le = PidExample(available[0][0], tb)
        QtGui.QApplication.instance().exec_()
        #le.app.exec_()
    else:
        print "No Crazyflies found, cannot run example"

    os._exit(0)
