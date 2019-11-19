from xml.dom import minidom
import xml.etree.ElementTree as ET
import numpy as np
import re
import os, sys
from string import digits
import traceback

def getPath():
    return os.path.split(traceback.extract_stack()[-1][0])[0]
filename_srdf = getPath() + '/centauro_srdf.xacro'
filename_limits = getPath() + '/joint_limits.xacro'

class HomePose():
    def __init__(self,pose=None,name=None):
        self.name = name or JointNames().getName()
        tree = ET.parse(filename_srdf)
        root = tree.getroot()
        if pose is None:
            self.pose = 'home'
        else:
            self.pose = pose
        # self.pose = pose or 'home'
        # print pose
        # self.name = []
        self.value = []
        for child in root:
            if child.tag == 'group_state' and child.attrib['name'] == pose:
                for subchild in child:
                    if subchild.attrib['name'] in self.name:
                        # self.name += [subchild.attrib['name']]
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
        # print self.name
        tree = ET.parse(filename_limits)
        root = tree.getroot()

        joint_str = []
        motor_str = []
        self.lower = []
        self.upper = []
        self.velocity = []
        self.torque = []

        for joint in self.name:
            try:
                i = [x.isdigit() for x in joint].index(True)
                num = int(joint[i])
                string = joint[:i]+joint[i+1:]
                # print string
                joint_str += [string]
            except ValueError:
                string = joint
                # print string
                joint_str += [string]
            if "arm" in string:
                motor = string
                # print motor
                motor_str += [motor]
            else:
                for child in root:
                    if child.tag == 'motor' and child.attrib['name'] in string:
                        motor = child.attrib['value']
                        # print motor
                        motor_str += [motor]

            for child in root:
                # print child.tag
                name = child.attrib['name']
                if string in name and 'lower' in name:
                    row = re.findall("[-+]?\d+\.\d+", child.attrib['value'])
                    self.lower += [float(row[num-1])]
                if string in name and 'upper' in name:
                    row = re.findall("[-+]?\d+\.\d+", child.attrib['value'])
                    self.upper += [float(row[num-1])]
                if motor in name and "velocity" in name:
                    if 'motor' in name:
                        self.velocity += [float(child.attrib['value'])]
                    else: 
                        row = re.findall("[-+]?\d+\.\d+", child.attrib['value'])
                        self.velocity += [float(row[num-1])]
                    # print self.velocity[-1]
                if motor in name and ("effort" in name or "torque" in name):
                    if 'motor' in name:
                        self.torque += [float(child.attrib['value'])]
                    else: 
                        row = re.findall("[-+]?\d+\.\d+", child.attrib['value'])
                        self.torque += [float(row[num-1])]
                    # print self.torque[-1]

    def printName(self):
        print self.name

    def getName(self):
        return self.name

    def getLowerBound(self):
        return self.lower
    
    def getUpperBound(self):
        return self.upper

    def getVelocityBound(self):
        return self.velocity

    def getTorqueBound(self):
        return self.torque

if __name__ == "__main__":
    # DEBUGGING

    # Getting home vectors for full centauro
    centauro = JointNames()
    centauro.printName()
    HomePose('Navvab_Parking_3',centauro.getName()).printValue()

    leftarm = JointNames('arm1')
    leftarm.printName()

    # left_arm = JointNames('arm1')
    # bounds = JointBounds(left_arm.getName())
    # print bounds.getName()
    # print bounds.getLowerBound()
    # print bounds.getUpperBound()
    # print bounds.getVelocityBound()
    # print bounds.getTorqueBound()
    # print len(bounds.getTorqueBound())

    # home = HomePose()
    # home.printName()
    # home.printPose()
    # home.printValue()

    # left_arm = JointNames('arm1')
    # # left_arm.printName()
    # # home = HomePose(name=left_arm.getName())
    # # home.printName()
    # # # home.printPose()
    # # home.printValue()
    # left_arm_bounds = JointBounds(left_arm.getName())
    # print left_arm_bounds.getLowerBound()
    # print left_arm_bounds.getUpperBound()
    # left_arm_bounds = JointBounds()
    # names = name_list.getName()

    # JointNames().printName()

    # bounds = JointBounds()
    # bounds.printName()