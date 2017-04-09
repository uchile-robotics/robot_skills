#!/usr/bin/env python
# -*- coding: utf-8 -*-

from robot import Robot
from robot_skill import RobotSkill

from termcolor import cprint
import inspect
import importlib
import pkgutil

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
