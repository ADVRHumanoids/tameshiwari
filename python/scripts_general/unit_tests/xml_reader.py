from xml.dom import minidom
import xml.etree.ElementTree as ET
import numpy as np

tree = ET.parse('centauro_srdf.xml')
root = tree.getroot()

joint_str = []
joint_init = []
counter = 0
for child in root:
    if child.tag == 'group_state' and child.attrib['name'] == 'home':
        # print child.tag, child.attrib
        # print child.attrib['name']
        for subchild in child:
            joint_str += [subchild.attrib['name']]
            if "j_arm1" in subchild.attrib['name']:
                joint_init += [float(subchild.attrib['value'])]
            elif "j_arm2" in subchild.attrib['name']:
                joint_init += [-1*float(subchild.attrib['value'])]
            else:
                joint_init += [float(subchild.attrib['value'])]
            # print subchild.attrib
            
           
print joint_str
print len(joint_str)
print joint_init + np.zeros(8).tolist()

# print np.zeros(32).tolist()
# mydoc = minidom.parse('centauro_srdf.xml')

# items = mydoc.getElementsByTagName('group_state')
# for elem in items:
#     print(elem.attributes['name'].value)

# for joint in root.iter('joint'):
#     print(root.attrib)