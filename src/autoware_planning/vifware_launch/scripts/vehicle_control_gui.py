#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

import Tkinter
import tkMessageBox


import threading
from threading import Thread
from time import sleep

import sys
import os
import signal
import subprocess




class VehicleControlGUI:

    def __init__(self):


      self.window = Tkinter.Tk()
      self.window.attributes('-topmost', True)
      self.window.protocol("WM_DELETE_WINDOW", self.close_window)

      self.window.title('Vehicle Control Interface')
      self.pro_motion = None
      self.pro_mission = None


      self.mission_planner_active = False
      self.motion_planner_active = False


      self.button_mission = Tkinter.Button(self.window, text = "Mission Planner", command = self.missionPlannerCallBack, font=("Arial Bold", 15))
      self.button_mission.grid(column=1, row=0)

      self.button_motion = Tkinter.Button(self.window, text = "Motion Planner", command = self.motionPlannerCallBack, font=("Arial Bold", 15))
      self.button_motion.grid(column=2, row=0)


      self.window.mainloop()

    def close_window(self):
       self.window.destroy()
       try:
          os.killpg(os.getpgid(self.pro_motion.pid), signal.SIGKILL)
       except:
         print("")
       try:
          os.killpg(os.getpgid(self.pro_mission.pid), signal.SIGKILL)
       except:
         print("")
       sys.exit()


    def motionPlannerCallBack(self):

        if self.motion_planner_active:
          self.motion_planner_active = False
          os.killpg(os.getpgid(self.pro_motion.pid), signal.SIGTERM)
          self.button_motion.configure(bg="lightgrey")
        else:
          self.motion_planner_active = True
          self.thread_playback = Thread(target=self.thread_motion_planner);
          self.thread_playback.start()
          self.button_motion.configure(bg="grey")

    def missionPlannerCallBack(self):

        if self.mission_planner_active:
          self.mission_planner_active = False
          os.killpg(os.getpgid(self.pro_mission.pid), signal.SIGTERM)
          self.button_mission.configure(bg="lightgrey")
        else:
          self.mission_planner_active = True
          self.thread_playback = Thread(target=self.thread_mission_planner);
          self.thread_playback.start()
          self.button_mission.configure(bg="grey")


    def thread_motion_planner(self):
        self.pro_motion = subprocess.Popen(['roslaunch', 'vifware_launch', 'vifware_motion_planning.launch'])


    def thread_mission_planner(self):
        self.pro_mission = subprocess.Popen(['roslaunch', 'vifware_launch', 'vifware_mission_planning.launch'])




if __name__ == '__main__':
    try:
        rospy.init_node('goal_publisher') # Initialize ROS
        VehicleControlGUI()
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Unexpected error:", sys.exc_info()[0]
        rospy.logfatal("Unexpected error:", sys.exc_info()[0])
