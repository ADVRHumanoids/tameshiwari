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

# plt.style.use("ggplot")

# t = np.arange(0.0, 2.0, 0.1)
# t = np.matlib.linspace(0.0,2.0,101)
# s = np.sin(2 * np.pi * t)
# s2 = np.cos(2 * np.pi * t)
# plt.plot(t, s, "o-", lw=4.1)
# plt.plot(t, s2, "o-", lw=4.1)
# plt.xlabel("time (s)")
# plt.ylabel("Voltage (mV)")
# plt.title("Simple plot $\\frac{\\alpha}{2}$")
# plt.grid(True)
# plt.show()
# import matplotlib2tikz
# str_time = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
# matplotlib2tikz.save("test__" + str_time + ".tex")

test123 = ['q_J00','q_J01']
print type(test123)