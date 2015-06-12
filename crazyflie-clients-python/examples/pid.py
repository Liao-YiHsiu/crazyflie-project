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

        self.thrust = 20000
        self.pitch = 0
        self.roll = 0
        self.yawrate = 0

        # setup gui
        self._gui()

        self.roll_data = deque([0] * self.DATA_NUM)
        self.pitch_data = deque([0] * self.DATA_NUM)
        self.yaw_data = deque([0] * self.DATA_NUM)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print "Connected to %s" % link_uri

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name="Stabilizer", period_in_ms=10)
        self._lg_stab.add_variable("stabilizer.roll", "float")
        self._lg_stab.add_variable("stabilizer.pitch", "float")
        self._lg_stab.add_variable("stabilizer.yaw", "float")

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

        # Start a timer to disconnect in 10s
        #t = Timer(5, self._cf.close_link)
        #t.start()
        Thread(target=self._ramp_motors).start()

    def _ramp_motors(self):
        thrust_mult = 1
        thrust_step = 500
        self.thrust = 30000
        self.pitch = 0
        self.roll = 0
        self.yawrate = 0

        time.sleep(1)

        #Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        while self.thrust >= 20000:
            self._cf.commander.send_setpoint(self.roll, self.pitch, self.yawrate, self.thrust)
            time.sleep(0.1)
            if self.thrust >= 45000:
                thrust_mult = -1
            self.thrust += thrust_step * thrust_mult
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print "Error when logging %s: %s" % (logconf.name, msg)

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        #print "[%d][%s]: %s" % (timestamp, logconf.name, data)
        self.roll_data.append(data["stabilizer.roll"])
        self.roll_data.popleft()
        #self.p_roll.setData(y=list(self.roll_data), x=range(self.DATA_NUM));

        self.pitch_data.append(data["stabilizer.pitch"])
        self.pitch_data.popleft()
        #self.p_pitch.setData(y=list(self.pitch_data), x=range(self.DATA_NUM));

        self.yaw_data.append(data["stabilizer.yaw"])
        self.yaw_data.popleft()
        #self.p_yaw.setData(y=list(self.yaw_data), x=range(self.DATA_NUM));


    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print "Connection to %s failed: %s" % (link_uri, msg)
        self.is_connected = False

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
        self.mw = QtGui.QMainWindow()
        self.mw.setWindowTitle('Pid')
        self.mw.resize(800,800)
        cw = QtGui.QWidget()
        self.mw.setCentralWidget(cw)
        l = QtGui.QVBoxLayout()
        cw.setLayout(l)
        
        pw_roll = pg.PlotWidget(name='Roll')
        l.addWidget(pw_roll)

        pw_pitch = pg.PlotWidget(name='Pitch')
        l.addWidget(pw_pitch)

        pw_yaw = pg.PlotWidget(name='Yaw')
        l.addWidget(pw_yaw)

        self.mw.show()

        self.p_roll = pw_roll.plot()
        self.p_roll.setPen((200,200,100))

        line_roll = QtGui.QGraphicsLineItem(QtCore.QLineF(0, 0, self.DATA_NUM, 0))
        line_roll.setPen(QtGui.QPen(QtGui.QColor(100, 200, 100)))

        line_pitch = QtGui.QGraphicsLineItem(QtCore.QLineF(0, 0, self.DATA_NUM, 0))
        line_pitch.setPen(QtGui.QPen(QtGui.QColor(100, 200, 100)))

        line_yaw = QtGui.QGraphicsLineItem(QtCore.QLineF(0, 0, self.DATA_NUM, 0))
        line_yaw.setPen(QtGui.QPen(QtGui.QColor(100, 200, 100)))

        pw_roll.addItem(line_roll)
        pw_pitch.addItem(line_pitch)
        pw_yaw.addItem(line_yaw)

        pw_roll.setLabel('left', 'Value', units='radius')
        pw_roll.setLabel('bottom', 'Time', units='s')
        pw_roll.setXRange(0, self.DATA_NUM)
        pw_roll.setYRange(-180, 180)

        self.p_pitch = pw_pitch.plot()
        self.p_pitch.setPen((200,200,100))

        pw_pitch.setLabel('left', 'Value', units='radius')
        pw_pitch.setLabel('bottom', 'Time', units='s')
        pw_pitch.setXRange(0, self.DATA_NUM)
        pw_pitch.setYRange(-90, 90)

        self.p_yaw = pw_yaw.plot()
        self.p_yaw.setPen((200,200,100))

        pw_yaw.setLabel('left', 'Value', units='radius')
        pw_yaw.setLabel('bottom', 'Time', units='s')
        pw_yaw.setXRange(0, self.DATA_NUM)
        pw_yaw.setYRange(-180, 180)

        self.t = QtCore.QTimer()
        self.t.timeout.connect(self._updateData)
        self.t.start(50)

    def _updateData(self):
        self.p_roll.setData(y=list(self.roll_data), x=range(self.DATA_NUM));
        self.p_pitch.setData(y=list(self.pitch_data), x=range(self.DATA_NUM));
        self.p_yaw.setData(y=list(self.yaw_data), x=range(self.DATA_NUM));

#def rand(n):
#    data = np.random.random(n)
#    data[int(n*0.1):int(n*0.13)] += .5
#    data[int(n*0.18)] += 2
#    data[int(n*0.1):int(n*0.13)] *= 5
#    data[int(n*0.18)] *= 20
#    data *= 1e-12
#    return data, np.arange(n, n+len(data)) / float(n)

#def updateData():
#    yd, xd = rand(10000)
#    p1.setData(y=yd, x=xd)

## Start a timer to rapidly update the plot in pw

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
