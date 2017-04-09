#!/bin/bash

robot-console () {
    #depcheck ipython
    PKG_DIR=$(rospack find robot_skills)
    #ROBOT_PARTS_DIR=$(rospack find robot_skills)/src/robot_skills
    echo -e "launching robot console...\n"
    ipython -i --no-banner --no-confirm-exit --autocall 2 "${PKG_DIR}/shell/console.py" -- "$@"
}



###################################
# COMPLETION: bash/zsh only
###################################
# prevent failure
if ! _bender_check_if_bash_or_zsh ; then
    return 1
fi


_robotcomplete_robot-console()
{
    local cur PKG_DIR rskills core skills opts first

    opts="-h --help --only"

    # get list of core skills from the robot_skills package
    PKG_DIR="$(rospack find robot_skills)"
    rskills=
    if [ -d "$PKG_DIR"/src/robot_skills/core ]; then
        cd "$PKG_DIR"/src/robot_skills/core
        rskills="$(find . -maxdepth 3 -type f -regex ".*.py" ! -name '__init__.py'  -printf '%f\n' | sed 's/.py//g')"
    fi

    # get list of core skills from the *_skills package
    PKG_DIR="$(rospack find ${ROBOT}_skills)"
    cd "$PKG_DIR"/src/${ROBOT}_skills/core
    core="$(find . -maxdepth 5 -type f -regex ".*.py" ! -name '__init__.py'  -printf '%f\n' | sed 's/.py//g')"

    # get list of high skills from the bender_skills package
    cd "$PKG_DIR"/src/${ROBOT}_skills/capabilities
    skills="$(find . -maxdepth 5 -type f -regex ".*.py" ! -name '__init__.py'  -printf '%f\n' | sed 's/.py//g')"

    if _bender_check_if_bash ; then

        # current word
        cur="${COMP_WORDS[COMP_CWORD]}"
        # only complete one option
        if [[ $COMP_CWORD == 1 ]] ; then
            COMPREPLY=( $(compgen -W "${opts} ${skills}" -- "${cur}") )
            return 0
        else

            first="${COMP_WORDS[1]}"

            # return on help
            if [ "$first" == "-h" ] || [ "$first" == "--help" ]; then
                return 0
            fi

            # show core skills only when the "--only" option is selected 
            if [ "$first" == "--only" ]; then
                COMPREPLY=( $(compgen -W "$rskills $core $skills" -- "${cur}") )
                return 0
            fi

            COMPREPLY=( $(compgen -W "$skills" -- "${cur}") )
            return 0
        fi

    else

        # zsh - compctl
        reply=()

        if [[ ${CURRENT} == 2 ]]; then
            reply=(${=opts} ${=skills})

        else
            first="${=${(s: :)words}[2]}"

            # return on help
            if [[ "$first" == "-h" ]] || [[ "$first" == "--help" ]]; then
                return 0
            fi

            # show core skills only when the "--only" option is selected 
            if [[ "$first" == "--only" ]]; then
                reply=(${=rskills} ${=core} ${=skills})
                return 0
            fi

            reply=(${=skills})
            return 0  

        fi

    fi

}


# prevent failure
if _bender_check_if_bash ; then
    complete -F _robotcomplete_robot-console robot-console
else
    compctl -K "_robotcomplete_robot-console" "robot-console"
fi
