#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
from robot import Robot
from robot_skill import RobotSkill

from termcolor import cprint
import inspect
import importlib
import pkgutil


# bender dependencies
if "ROBOT" not in os.environ:
    print "ROBOT env variable is not defined!"
    import sys
    sys.exit(1)

robot = os.environ["ROBOT"]
if robot not in ["bender", "maqui"]:
    print "ROBOT env variable is set to an unknown robot: {}".format(robot)
    import sys
    sys.exit(1)


def onerror(name):
    cprint("Error importing module {}!!".format(name), "red", "on_white")
    print_tb(traceback)

def get_classes(package_name, class_type):
    """
    Get specific classes that lies in a package.
    
    Args:
        package_name (str): Package name.
        class_type (class): Class type.
    """
    package = importlib.import_module(package_name)
    class_list = list()
    for importer, modname, ispkg in pkgutil.walk_packages(path=package.__path__,
                                                          prefix=package.__name__+'.',
                                                          onerror=onerror):
        try:
            module = importlib.import_module(modname)
        except ImportError as e:
            msg = 'Error at import {}: {}'.format(modname, e)
            cprint(msg, "red", "on_white")
            continue

        for name, obj in inspect.getmembers(module):
            if inspect.isclass(obj) and issubclass(obj, RobotSkill):
                class_list.append(obj)
    return class_list


def get_skill_dict(packages=list()):
    """
    Get skill dict with {skill._type, skill} entry.
    
    Args:
        packages (list of str): Packages that contains RobotSkill.
    """
    skill_dict = dict()
    for package_name in packages:
        try:
            class_list = get_classes(package_name, RobotSkill)
        except:
            continue
        for skill_class in class_list:
            # Check for name error
            if skill_class._type in skill_dict and skill_class != skill_dict[skill_class._type]:
                msg = 'Skill type {}({}) already used by {}({})'.format(skill_class.__name__, skill_class, 
                    skill_dict[skill_class._type].__name__, skill_dict[skill_class._type])
                cprint(msg, "red", "on_white")
            skill_dict.update({skill_class._type : skill_class})
    return skill_dict

def build_robot(skills, check=True, setup=True):
    """
    Build a robot object based on a skill list. By default build
    a robot using all core skills.

    Args:
        skills (list of str): Skill list.

    Raises:
        TypeError: If `skills` is not a list.
    """
    rospy.loginfo("factory: building robot ... ")
    robot = Robot("bender")

    # Check arg
    if not isinstance(skills, list):
        raise TypeError("skills must be a string list")

    # Add skill instance to robot    
    for skill_name in skills:
        if skill_name in _str_to_skill:
            robot.set(_str_to_skill[skill_name].get_instance())
            # Skills shortcuts
            if skill_name == 'tts':
                robot.say = robot.tts.say
        else:
            rospy.logerr("Skill '{0}' is not registered".format(skill_name))

    rospy.loginfo("factory: the robot is built")
    # Robot check
    if check:
        if not robot.check():
            raise RuntimeError("Required skills don't available")
    # Robot setup
    if setup:
        if not robot.setup():
            raise RuntimeError("Robot setup failed")
    # Return robot
    return robot


_str_to_skill = get_skill_dict(['robot_skills', robot + '_skills'])