# EMPTY THIS SCRIPT AT THE END OF USAGE

from casadi import *

p_end = [2]



p_end = MX(p_end)
print p_end 
print type(p_end)
print p_end.size()
print exp