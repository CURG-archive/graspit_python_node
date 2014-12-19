#!/usr/bin/python
import subprocess
import sys
import rospy
if __name__=="__main__":
    rospy.init_node('graspit_remote')
    graspit_ssh_url = rospy.get_param('~graspit_ssh_url', 'jweisz@192.168.11.63')
    graspit_display = rospy.get_param('~graspit_display',':1.0')
    spargs = ['ssh',graspit_ssh_url, '-t', "pkill graspit;export DISPLAY=%s;bash -ic graspit"%(graspit_display)]
    p = subprocess.Popen(spargs)

    while not rospy.is_shutdown():
        pass
    p.kill()

        
