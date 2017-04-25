#! /usr/bin/env python
## %Tag(FULLTEXT)%

import rospy
import math


class Object():

    def __init__(self, _name, _pose = None, _bboxes = None):
        self.name = _name # object name
        self.category = "unknown" #  options : "drinks","food","snacks","cleaningstuff","containers"
        self.posestamped = _pose # center pose
        self.bboxes = _bboxes # bboxes 2d
        self.location = "unknown"
        self.color = "unknown" # dominant color, options: red, blue, yellow, green, black, white
        self.form = "unknown" # form, options : box, sphere, cylinder
        self.hight = 0 # hight, information 3d 
        self.width = 0 # width, information 3d 
        self.long = 0 # long, information 3d 

        #Load object information
        self.getClass()
        self.getClassLocation()

    def getClass(self):
        class_mapping = rospy.get_param('~/classes')
        unknown_class = rospy.get_param('~/unknown_class')

        for object_class in class_mapping:

            objects = rospy.get_param('~/' + object_class)

            if self.name in objects:
                self.category = object_class
                rospy.loginfo("class found: " + object_class + ", for item: " + self.name)
                return 

        rospy.logwarn("class not found, for item: " + self.name)
        self.category = unknown_class


    def getClassLocation(self):
        unknown_class = rospy.get_param('~/unknown_class')
        # check unknown
        if unknown_class == self.category:
            if rospy.has_param('~/' + unknown_class + '_location'):

                location = rospy.get_param('~/' + unknown_class + '_location')
                rospy.loginfo("class location found: " + location + ", for class: " + self.category)
                self.location = location
                return

        # check other classes
        if rospy.has_param('~/' + self.category + '_location'):

            location = rospy.get_param('~/' + self.category + '_location')
            rospy.loginfo("class location found: " + location + ", for class: " + self.category)
            self.location = location
            return

        rospy.logwarn("location not found, for class: " + self.category)

        

class Face():
    def __init__(self, _pose, _bboxes):
        self.name = "unknown" # object name
        self.pose = _pose # center pose
        self.bboxes = _bboxes # bboxes 2d
        self.emotion = "unknown" # dominant emotion, options: sad, happy, angry...
