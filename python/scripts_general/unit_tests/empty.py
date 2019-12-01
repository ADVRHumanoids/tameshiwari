# EMPTY THIS SCRIPT AT THE END OF USAGE

import centauro_config as config

joints = config.JointNames('arm1')
joints.printName()
joints.addJoints('torso')
joints.printName()