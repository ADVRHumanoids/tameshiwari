from xml.dom import minidom
import xml.etree.ElementTree as ET
import numpy as np
import re

filename_srdf = 'centauro_srdf.xacro'
filename_limits = 'joint_limits.xacro'

tree = ET.parse(filename_limits)
root = tree.getroot()

enum = 1
joint_str = []
q_lb = []
q_ub = []

arm_1 = True
arm_ind = 1 if arm_1 else 2

for child in root:
    if 'j_arm_' in child.attrib['name'] and 'lower' in child.attrib['name']:
        joint_str += ['j_arm' + str(arm_ind) + '_' + str(enum)]
        string = child.attrib['value']
        row = re.findall("[-+]?\d+\.\d+", string)
        q_lb += [row[arm_ind-1]]
        enum += 1
    if 'j_arm_' in child.attrib['name'] and 'upper' in child.attrib['name']:
        string = child.attrib['value']
        row = re.findall("[-+]?\d+\.\d+", string)
        q_ub += [row[arm_ind-1]]
        enum += 1

# print joint_str
# print q_lb
# print q_ub

class HomePose():
    def __init__(self,pose=None):
        tree = ET.parse(filename_srdf)
        root = tree.getroot()
        self.pose = pose or 'home'
        self.name = []
        self.value = []
        for child in root:
            if child.tag == 'group_state' and child.attrib['name'] == 'home':
                for subchild in child:
                    self.name += [subchild.attrib['name']]
                    self.value += [float(subchild.attrib['value'])]
    
    def getName(self):
        return self.name

    def getNumJoints(self):
        return len(self.value)

    def printName(self):
        print self.name

    def getValue(self):
        return self.value

    def printValue(self):
        print self.value
    
    def getPose(self):
        return self.pose
    
    def printPose(self):
        print self.pose

class JointNames():
    def __init__(self,keyword=None):
        # Use Keyword to extract certain joints, for example using "arm" to extract all arm joints
        tree = ET.parse(filename_srdf)
        root = tree.getroot()
        self.name = []
        key = keyword or ''
        for child in root:
            if child.tag == 'group' and child.attrib['name'] == 'joint_names':
                for subchild in child:
                    if key in subchild.attrib['name']:
                        self.name += [subchild.attrib['name']]

    def getName(self):
        return self.name

    def printName(self):
        print self.name

    def getNumJoints():
        return len(self.name)

class JointBounds():
    def __init__(self,name=None):
        self.name = name or JointNames().getName()
        tree = ET.parse(filename_limits)
        root = tree.getroot()


    def printName(self):
        print self.name

if __name__ == "__main__":
    # home = HomePose()
    # home.printName()
    # home.printPose()
    # home.printValue()

    left_arm = JointNames('arm1')
    # left_arm_bounds = JointBounds
    # names = name_list.getName()
    # print names

    # JointNames().printName()

    # bounds = JointBounds()
    # bounds.printName()