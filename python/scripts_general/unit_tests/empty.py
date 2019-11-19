# EMPTY THIS SCRIPT AT THE END OF USAGE

import numpy as np
import random
import centauro_config as config

init_pose = 'home'
centauro = config.HomePose(pose=init_pose)
joints_all = centauro.getName()
q_all = centauro.getValue()
q_all = np.asarray(q_all)
print type(q_all)
current_str = ['j_arm1_1', 'hip_pitch_1', 'j_arm1_3', 'j_arm1_4', 'j_arm1_5', 'j_arm1_6']
random.shuffle(current_str)

index = [i for i, x in enumerate(joints_all) if x in current_str]
print index
joints_remain = [i for j, i in enumerate(joints_all) if j not in index]
print joints_remain
q_remain = np.delete(q_all,index).tolist()
print q_remain