import os
import sys

# print sys.version

# print os.getcwd()

# print os.path.abspath
# print os.path.realpath

def realpath():
    return os.path.realpath(sys.argv[0])

def abspath(path):
    return os.path.abspath(path)








if __name__ == '__main__':
    path = realpath()
    abspath = abspath(path)
    print path
    print abspath
    