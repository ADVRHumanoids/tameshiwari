# -*- coding: utf-8 -*-

# import numpy as np
# import matplotlib.pyplot as plt


# # Example data
# t = np.arange(0.0, 1.0 + 0.01, 0.01)
# s = np.cos(4 * np.pi * t) + 2

# plt.rc('text', usetex=True)
# plt.rc('font', family='serif')
# plt.plot(t, s)

# plt.xlabel(r'\textbf{time} (s)')
# plt.ylabel(r'\textit{voltage} (mV)',fontsize=16)
# # plt.title(r"\TeX\ is Number "
# #           r"$\displaystyle\sum_{n=1}^\infty\frac{-e^{i\pi}}{2^n}$!",
# #           fontsize=16, color='gray')

# # Make room for the ridiculously large title.
# # plt.subplots_adjust(top=0.8)

# # plt.savefig('tex_demo')
# plt.show()

import matplotlib.pyplot as plt
import numpy as np
import numpy.matlib
import os
from datetime import datetime
from cycler import cycler

# plt.style.use("ggplot")

# prop_cycle = plt.rcParams
# print prop_cycle
# prop_cycle = plt.rcParams['axes.prop_cycle']
# print prop_cycle
# colors = prop_cycle.by_key()['color']
# print colors
# print type(colors)
# colors.extend(colors)
# print colors
# plt.rc('axes',prop_cycle=(cycler(color=colors)))
# prop_cycle = plt.rcParams['axes.prop_cycle']
# print prop_cycle


# Time  = np.linspace(0,10,11)
# print Time
# print Time[[0,-1]]

vec = []
g = "g"
h = "h"

vec += g
vec += h 
print vec