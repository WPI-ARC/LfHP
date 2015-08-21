#!/usr/bin/env python
# Rqt plugin that provides GUI to perform trajectory comparison. Two trajectories are
# randomly chosen from the trajectory pool and the pairwise comparison is performed.
# The process of ranking using the merge sort intuition and repeat the comparison 
# procedure until the ranking is generated.
#   Author: Artem Gritsenko

from __future__ import division

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QLabel, QTreeWidget, QTreeWidgetItem, QVBoxLayout, QCheckBox, QWidget, QToolBar, QLineEdit, QPushButton, QSlider
from python_qt_binding.QtCore import Qt, QTimer

import rospy
import roslaunch
from pr2_robot_msgs.msg import *
from std_msgs.msg import *
from pr2_robot_msgs.srv import *

import os
import random

import re
from threading import RLock
import textwrap
import copy

class Top(Plugin):

    def __init__(self, context):
        super(Top, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Top')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())

        rospy.wait_for_service('PlayTrajectoryState')
        self.play_traj = rospy.ServiceProxy('PlayTrajectoryState', PlayTrajectoryState)

        # Create a container widget and give it a layout
        self._container = QWidget()
        self._layout    = QVBoxLayout()
        self._container.setLayout(self._layout)
        context.add_widget(self._container)
        
        # merge sort to pick files generate files to pick

        traj_dir = rospy.get_param('traj_dir')
        self.traj_list = sorted( os.listdir(traj_dir) )
        #self.traj_list = self.traj_list[:5]
        self.length = len( self.traj_list )
        self.work_list = copy.deepcopy(self.traj_list)
        self.merge_width = 1
        self.merge_ind = 0
        
        self._i0 = self.merge_ind
        self._i1 = min(self.merge_ind + self.merge_width, self.length)
        self._j = 0

        self.comp_res = True
        self.valid_comp = False
        self.merge()

        
        #random.seed()
        

        #sys.stdin.readline()
        # use merge sort to sample traj fiels
        #self.merge_traj_files()

        # regenerate the trajectories
        #self.rand_traj_files()
        self.write_to_file = ""
        
        # Add a button to play first trajcetory
        self._play_A_btn = QPushButton('Play Trajectory A')
        self._layout.addWidget(self._play_A_btn)
        self._play_A_btn.clicked.connect(self._play_traj_A)

        # Add a slider to trace the first trajectory
        self._slider_A = QSlider(Qt.Horizontal)
        self._layout.addWidget(self._slider_A)
        self._slider_A.setMinimum(1)
        self._slider_A.setMaximum(100)
        self._slider_A.valueChanged.connect(self._change_slider_A)

        # Add a button to play second trajcetory
        self._play_B_btn = QPushButton('Play Trajectory B')
        self._layout.addWidget(self._play_B_btn)
        self._play_B_btn.clicked.connect(self._play_traj_B)

        # Add a slider to trace the second trajectory
        self._slider_B = QSlider(Qt.Horizontal)
        self._layout.addWidget(self._slider_B)
        self._slider_B.setMinimum(1)
        self._slider_B.setMaximum(100)
        self._slider_B.valueChanged.connect(self._change_slider_B)

        # Add a button to pause current trajectories
        self._pause_btn = QPushButton('Pause current trajectory')
        self._layout.addWidget(self._pause_btn)
        self._pause_btn.clicked.connect(self._pause_traj)
        self.paused = False
        
        # Add a button to pick first trajcetory
        self._pick_A_btn = QPushButton('Pick Trajectory A')
        self._layout.addWidget(self._pick_A_btn)
        self._pick_A_btn.clicked.connect(self._pick_traj_A)

        # Add a button to pick second trajcetory
        self._pick_B_btn = QPushButton('Pick Trajectory B')
        self._layout.addWidget(self._pick_B_btn)
        self._pick_B_btn.clicked.connect(self._pick_traj_B)

        # Add a button to pick neither of trajcetories
        self._pick_none_btn = QPushButton('Pick None of Trajectories')
        self._layout.addWidget(self._pick_none_btn)
        self._pick_none_btn.clicked.connect(self._pick_none)


    def merge_traj_files(self):
        print "___________________________________________"
        if self.merge_width < self.length:
            print "Width = %f" % self.merge_width
            if self.merge_ind < self.length:
                print "Merge index = %f" % self.merge_ind
                # take pair of sublists and merge them
                while True:
                    if self._j < min(self.merge_ind + 2 * self.merge_width, self.length):
                        if (self._i0 < min(self.merge_ind + self.merge_width, self.length)):
                                if self._i1 >= min(self.merge_ind + 2*self.merge_width, self.length):
                                    self.work_list[self._j] = self.traj_list[self._i0]
                                    print "it's here 1"
                                    print "B %f assigned A %f" % (self._j, self._i0)
                                    self._i0 += 1
                                    self.valid_comp = False
                                else:
                                    if self.valid_comp:
                                        if self.comp_res:
                                            self.work_list[self._j] = self.traj_list[self._i0]
                                            print "it's here 2"
                                            print "B %f assigned A %f" % (self._j, self._i0)
                                            self._i0 += 1
                                            self.valid_comp = False

                                        else:
                                            self.work_list[self._j] = self.traj_list[self._i1]
                                            print "it's here 3"
                                            print "B %f assigned A %f" % (self._j, self._i1)
                                            self._i1 += 1
                                            self.valid_comp = False
                                    else:
                                        self.merge()
                                        break
                        else:
                            self.work_list[self._j] = self.traj_list[self._i1]
                            print "it's here 4"
                            print "B %f assigned A %f" % (self._j, self._i1)
                            self._i1 += 1
                            self.valid_comp = False
                        self._j += 1
                    else:
                        self.merge_ind = self.merge_ind + 2 * self.merge_width
                        self._i0 = self.merge_ind
                        self._i1 = min(self.merge_ind + self.merge_width, self.length)
                        self._j = self.merge_ind
                        self.merge()
                        print "lists are merged"
                        print self.traj_list
                        print self.work_list
                        temp = copy.deepcopy(self.traj_list)
                        self.traj_list = copy.deepcopy(self.work_list)
                        #self.work_list = temp
                        #self.merge_traj_files()
                        break
            else:
                # copyArray()
                temp = copy.deepcopy(self.traj_list)
                self.traj_list = copy.deepcopy(self.work_list)
                self.work_list = temp

                self.merge_ind = 0
                self.merge_width = 2 * self.merge_width
                self._i0 = self.merge_ind
                self._i1 = min(self.merge_ind + self.merge_width, self.length)
                self._j = self.merge_ind
                print "ALL LISTS ARE MERGED"
                print self.traj_list
                self.merge_traj_files()
        else:
            print "ALL TRAJECTORIES ARE RANKED"
            f_ = open('ranked_traj.txt','w')
            for item in self.traj_list:
                f_.write("%s\n" % item)
            f_.close()


    def merge(self):
        print self._i0
        print self._i1
        try:
            self.fileA = self.traj_list[self._i0]
            self.fileB = self.traj_list[self._i1]
        except:
            try:
                self.fileA = self.work_list[0]
                self.fileB = self.work_list[min(2 * self.merge_width, self.length)]
            except:
                self.fileA = self.traj_list[0]
                self.fileB = self.traj_list[1]
        print " new files assigned are: %s, %s " % (self.fileA, self.fileB)
        


    def _play_traj_A(self):
        if (self.paused):
            self._pause_btn.setText("Pause current trajectory")
            self.paused = False
        req = UiState(Header(), self.fileA, True, False,-1)
        resp = self.play_traj(req)


    def _change_slider_A(self):
        self._slider_B.setValue(0)
        req = UiState(Header(), self.fileA, False, True, float(self._slider_A.value()))
        resp = self.play_traj(req)


    def _play_traj_B(self):
        if (self.paused):
            self._pause_btn.setText("Pause current trajectory")
            self.paused = False
        req = UiState(Header(), self.fileB, True, False,-1)
        resp = self.play_traj(req)


    def _change_slider_B(self):
        self._slider_A.setValue(0)
        req = UiState(Header(), self.fileB, False, True, float(self._slider_B.value()))
        resp = self.play_traj(req)


    def _pause_traj(self):
        if (self.paused):
            req = UiState(Header(), "", False, False,-1)
            resp = self.play_traj(req)
            self._pause_btn.setText("Pause current trajectory")
            self.paused = False
        else:
            req = UiState(Header(), "", False, True,-1)
            resp = self.play_traj(req)
            self._pause_btn.setText("Continue playing current trajectory")
            self.paused = True


    def _pick_traj_A(self):
        self.comp_res = True
        self.valid_comp = True
        self.merge_traj_files()

    def _pick_traj_B(self):
        self.comp_res = False
        self.valid_comp = True
        self.merge_traj_files()


    def _pick_none(self):
        #reinvoke the traj ranodm generator
        self.rand_traj_files()
    

    # produce two new filenames from the trajectory directory
    def rand_traj_files(self):
        first_to_pick = random.randint(0,len(self.traj_list)-1)
        while (True):
            second_to_pick = random.randint(0,len(self.traj_list)-1)
            if (second_to_pick != first_to_pick):
                break
        self.fileA = self.traj_list[first_to_pick]
        self.fileB = self.traj_list[second_to_pick]

