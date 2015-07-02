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
    DATA_NUM = 600
    SAFE_ROLL = 15
    SAFE_PITCH = 15

    STB_ROLL  = 2
    STB_PITCH = 2
    STB_COUNT = 30

    stable_counter = 0
    m1_sum = 0.0
    m2_sum = 0.0
    m3_sum = 0.0
    m4_sum = 0.0

    log_var = [
            ("stabilizer.roll",   "float",   "radius", -10,  10),
            ("stabilizer.pitch",  "float",   "radius", -10,  10),
           # ("stabilizer.yaw",    "float",   "radius", -180, 180),
            ("stabilizer.thrust", "uint16_t", "force",  0, 65535),

           # ("baro.asl", "float", "pa",  0, 300),
           # ("baro.aslRaw", "float", "pa",  0, 300),
           # ("baro.aslLong", "float", "pa",  0, 300)]

            ("motor.m1", "int32_t", "power", 0, 65535),
            ("motor.m2", "int32_t", "power", 0, 65535),
            ("motor.m3", "int32_t", "power", 0, 65535),
            ("motor.m4", "int32_t", "power", 0, 65535),

           # ("actuator.roll",     "int16_t",  "force",  -32768, 32767),
           # ("actuator.pitch",    "int16_t",  "force",  -32768, 32767)
            ]

    log_data = []
    log_plot = []

    log_noplot = [
           # ("motor.m1", "int32_t"),
           # ("motor.m2", "int32_t"),
           # ("motor.m3", "int32_t"),
           # ("motor.m4", "int32_t")
            ]
    
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
            ("flightmode", "althold",    0),
            ("motorFactor", "m2", 1),
            ("motorFactor", "m3", 1),
            ("motorFactor", "m4", 1)
            ]
    param_spins = []


    control_var = [ "thrust", "roll", "pitch", "yawrate" ]
    control_spins = []

    control_data = {}
    param_data   = {}

    thrust_counter = 0
    decay = 1

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

        for logname, logtype in self.log_noplot:
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
                    cb = self._init_param_callback)
            #self._cf.param.request_param_update("{0}.{1}".format(group, name))
            # initial values
            self._cf.param.set_value("{0}.{1}".format(group, name), "{:.2f}".format(value))


        # Start a timer to disconnect in 10s
        #t = Timer(5, self._cf.close_link)
        #t.start()
        Thread(target=self._ramp_motors).start()

    def _init_param_callback(self, name, value):
        """Generic callback registered for all the groups"""
        print "init {0}: {1}".format(name, value)
        arr = name.split('.')
        group = arr[0]
        name = arr[1]

        self._cf.param.remove_update_callback(group = group, name = name,
                cb = self._init_param_callback)

        self.param_data[name] = value
        
        if len(self.param_data) == len(self.param_var):
            print "all value recorded"
            QtCore.QTimer.singleShot(100, self._param_update)

        #for i in range(len(self.param_var)):
        #    if group == self.param_var[i][0] and name == self.param_var[i][1]:
        #        QTCore.QTimer.singleShot(0, )
        #        #self.param_spins[i].setValue(value)
        #        break

        self._cf.param.add_update_callback(group = group, name = name,
                cb = self._param_callback)

    def _param_callback(self, name, value):
        """Generic callback registered for all the groups"""
        print "{0}: {1}".format(name, value)

    def _param_update(self):
        print "updating init parameters"
        for i in range(len(self.param_spins)):
            name = "{0}.{1}".format(self.param_var[i][0], self.param_var[i][1])
            self.param_spins[i].setValue(param_data[name])



    def _ramp_motors(self):
        time.sleep(1)

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

        thrust = data['stabilizer.thrust']
        roll   = data['stabilizer.roll']
        pitch  = data['stabilizer.pitch']

        if thrust == 0:
            self.thrust_counter += 1
        else:
            self.thrust_counter = 0
        
        if thrust != 0 or self.thrust_counter < 100 :
        #if True:
            for i in range(len(self.log_var)):
                self.log_data[i].append(data[self.log_var[i][0]])
                self.log_data[i].popleft()

        for name, datatype in self.log_noplot:
            print "[%s]: %2.4f" % (name, data[name])
        
        #safe zone testing...
        if abs(roll) > self.SAFE_ROLL or \
                abs(pitch) > self.SAFE_PITCH :
            #print "Danger!!! %2.4f %2.4f" % (roll, pitch)
            self.decay = 0.98
            #self.control_data['thrust'] = 0

        if abs(roll) > 2*self.SAFE_ROLL or \
                abs(pitch) > 2*self.SAFE_PITCH :
            self.control_data['thrust'] = 0

        if self.control_data['thrust'] <= 100:
            self.decay = 1

        self.control_data['thrust'] *= self.decay

        #logging stablized flight...
        if thrust > 30000 and abs(roll) < self.STB_ROLL and abs(pitch) < self.STB_PITCH:
            self.stable_counter += 1;
            self.m1_sum += data['motor.m1'] 
            self.m2_sum += data['motor.m2'] 
            self.m3_sum += data['motor.m3'] 
            self.m4_sum += data['motor.m4'] 
        else:
            if self.stable_counter >= self.STB_COUNT:
                print "----------------------------------------------------"
                print time.strftime("%I:%M:%S")
                print "stable counter: %d " % (self.stable_counter)
                print "   with average m1 = %5.1f m2 = %5.1f " % \
                        (self.m1_sum / self.stable_counter, \
                        self.m2_sum / self.stable_counter)
                print "                m3 = %5.1f m4 = %5.1f " % \
                        (self.m3_sum / self.stable_counter, \
                        self.m4_sum / self.stable_counter)
                print " ratio: %1.5f %1.5f %1.5f %1.5f" % \
                        (self.m1_sum / float(self.m1_sum), \
                         self.m2_sum / float(self.m1_sum), \
                         self.m3_sum / float(self.m1_sum), \
                         self.m4_sum / float(self.m1_sum))
            self.stable_counter = 0
            self.m1_sum = 0
            self.m2_sum = 0
            self.m3_sum = 0
            self.m4_sum = 0


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
                self.control_data[self.control_var[i]] = value

        for i in range(len(self.param_spins)):
            if self.param_spins[i] == sb:
                self._cf.param.set_value("{0}.{1}".format(self.param_var[i][0], self.param_var[i][1]), "{:.6f}".format(value))

        #self.thrust = value

    def startClicked(self):
        self.control_data['thrust'] = 55000

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

        btn2 = QtGui.QPushButton("Start")
        btn2.clicked.connect(self.startClicked)
        layout.addWidget(btn2)

# params inputs.
        for group, name, value in self.param_var:
            label = QtGui.QLabel("{0}.{1}".format(group, name))
            spin  = pg.SpinBox(value = value, bounds=[0, None], step = 0.1, minStep = 0.1)
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
