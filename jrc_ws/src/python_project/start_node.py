import os, os.path, sys
import time

node_name = 'jrc_main_node'
val = os.popen('ps -ef|grep ' + node_name + '|grep -v grep').read()
if (val != ""):
    # print(val)
    items = val.split(" ")
    pid = ""
    i = 0
    for item in items:
        if i is 0 or item is '':
            i = i + 1
            continue
        else:
            pid = item
            break

    print(pid)
    os.system("rosnode kill /" + node_name)
    time.sleep(0.1)
    os.system("kill " + pid)
else:
    print("Cannot Find Process: " + node_name)