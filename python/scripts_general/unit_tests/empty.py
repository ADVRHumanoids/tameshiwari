# EMPTY THIS SCRIPT AT THE END OF USAGE

import subprocess
import os

subprocess.call(["rosservice", "call", "/xbotcore/HomingExample_switch", "1"])
subprocess.call(["rosservice", "call", "/xbotcore/HomingExample_switch", "0"])
subprocess.call(["rosservice", "call", "/xbotcore/XBotCommunicationPlugin_switch", "1"])
subprocess.call(["rosservice", "call", "/xbotcore/set_filter_profile_safe"])
subprocess.call(["rosservice", "call", "/xbotcore/set_filter_profile_fast"])


os.system("ls -l")

# print os.getcwd()