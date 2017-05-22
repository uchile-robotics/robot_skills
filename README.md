# robot_skills [deprecated]

## Overview

This package manages the ROS interface to the robot low and mid level software. It is written in python and its goal is to simplify the robot usage when writting states or behaviours. Each software piece should have a `robot_skill` module implementing the `RobotSkill` class.

It aims to be robot independent.

## Arquitectura

## API

### Clase `robot`

Documentado en el mismo archivo: `src/robot_skills/robot.py`

### Clase `robot_skill`

Documentado en el mismo archivo: `src/robot_skills/robot_skill.py`


## Creando nuevos skills

Cada skill debe heredar de la clase `RobotSkill` e implementar al menos los métodos `start()`, `check()`, `setup()`, `pause()` y `shutdown()`. Es decisión del implementador la funcionalidad que tendrán, por ejemplo, no tiene sentido hacer shutdown o pausa del joystick, pues es imperativo que esté funcionando en todo momento.

Además de los métodos obligatorios, se espera que cada implementador agrege más funcionalidades a la clase. Por ejemplo, se espera que la HeadSkill provea una función cómo `head.set_emotion("happy")`.

En la carpeta `src/robot_skills/core/` se presentan implementaciones de cada componente del core. Se recomienda utilizarlas como base para programar nuevas funcionalidades.


## Ejemplos

La carpeta `robot_skills/samples` contiene nodos de ROS con ejemplos de diversas versiones de un robot, construido a partir de los skills de ejemplo. Pueden ser ejecutados con los siguientes comandos: 

```bash
rosrun robot_skills testbot.py
```
