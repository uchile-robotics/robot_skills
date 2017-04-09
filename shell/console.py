#!/usr/bin/ipython -i

"""
robot-console
    This runs a ipython console where you can
    directly interact with the robot skills.

    The interpreter provides the "robot" variable.
    The skills this robot contains depends on the parameters
    you used to call this console.

    TODO: In the future, this is suposed to let you call
    smach states.

Usage:
  robot-console (--only <skill>... | [<skill>...])

Options:
  -h --help  Show this screen.
  --only     Robot is built only with the selected skills.

Examples:
 robot-console
    It builds a robot with the default skills, i.e: a corebot.

 robot-console navigation perception
    It builds a corebot, but augmented with the required skills.

 robot-console --only tts knowledge
    It builds a robot from scratch, using the selected skills only.

"""

import os
import rospy
from docopt import docopt, DocoptExit
from termcolor import cprint, colored

# bender dependencies
if "ROBOT" not in os.environ:
    print "ROBOT env variable is not defined!"
    import sys
    sys.exit(1)

if os.environ["ROBOT"] == "bender":
    from bender_skills import robot_factory as _robot_factory
elif os.environ["ROBOT"] == "maqui":
    from maqui_skills import robot_factory as _robot_factory
else:
    print "ROBOT env variable is set to an unknown robot: {}".format(os.environ["ROBOT"])
    import sys
    sys.exit(1)    


def colored_header(text):
    return colored(text, 'green', 'on_grey', attrs=['bold'])


def print_process(text):
    return cprint(text, 'cyan', attrs=[])


def print_ready(text):
    return cprint(text, 'yellow', 'on_grey', attrs=['bold'])


def print_warning(text):
    return cprint(text, 'yellow', attrs=[])


if __name__ == '__main__':
    try:
        args = docopt(__doc__)
        _selected_skills = args["<skill>"]
        _use_only_required = args["--only"]

        rospy.init_node("robot_console", anonymous=True)

        print(
            colored_header("\n") +
            colored_header("\t ----------------------------------- \n") +
            colored_header("\t|   ") +
            colored("\033[9mTU/e\033[0m\033[95m", 'green', 'on_grey', attrs=['bold']) +
            colored_header(" Robot Console     |\n") +
            colored_header("\t ----------------------------------- \n")
        )
        print_ready(
            "WORK IN PROGRESS...\nMeanwhile, this can only provide you a robot " +
            "with core skills. If you need to use high level skills, then you will have " +
            "to import and load them into the robot."
        )
        
        # create core robot
        print_process('1.- Building your new robot')
        if not _selected_skills:
            # default robot
            print_process('... using the default skills: "core robot"')
            robot = _robot_factory.build()

        elif _use_only_required:
            # build only with the required skills
            print_process('... building only with your required skills')
            # 
            # TODO: factory para skills de alto nivel
            high_skills = set(['navigation', 'manipulation', 'perception', 'audition'])
            requested_high_level_skills = len(high_skills.intersection(set(_selected_skills))) > 0

            if not requested_high_level_skills:
                robot = _robot_factory.build(_selected_skills)
            else:
                print_warning("... sorry, at the moment i am only enabled to setup core skills.")
                raise DocoptExit("")


        else:
            # append new skills to the default robot
            print_process('... using the default skills: "core robot"')
            robot = _robot_factory.build()

            print_process('... adding the skills you requested')
            high_skills = ['navigation', 'manipulation', 'perception', 'audition']
            filtered_skills = []
            for skill in _selected_skills:
                if skill in high_skills:
                    print_warning(".. omiting the '" + skill +  "' skill. I can only setup core skills at the moment.")
                # print "setting the '" + skill + "' skill"
                # robot.set(NavigationSkill.get_instance())

        print_process('... Your robot is built\n')


        # checking robot
        print_process('2.- Checking your new robot')
        if not robot.check():
            raise DocoptExit("ERROR: The created robot has unmet runtime dependencies")
        print_process('... Your robot is built\n')

        # robot setup
        print_process('3.- Preparing your robot')
        if not robot.setup():
            raise DocoptExit("ERROR: There was a problem while trying to setup the robot.")
        print_process('... Your robot is ready to work\n')

        # done
        print_ready(
            "\n" +
            "\tYou can use the \"robot\" now. Try:\n" +
            "\t> robot.name\n"
            "\t> robot.tts.say(\"hello lab\")\n" + 
            "\t> robot.say(\"hello lab\")\n" + 
            "\t> robot.[TAB]\n"
        )

    except DocoptExit as e:
        cprint(str(e), 'red')
        cprint("Killing console ...", 'red')
        os.kill(os.getpid(), 9)
