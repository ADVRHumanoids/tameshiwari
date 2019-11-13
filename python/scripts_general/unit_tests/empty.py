# EMPTY THIS SCRIPT AT THE END OF USAGE

import centauro_functions as cfn

# path = cfn.realpath()
# print path
# abspath = cfn.abspath(path)
# print abspath


from string import digits

s = 'ddf12939dkdfjidi3119;94ddd'
res = s.translate(None,digits)
print res