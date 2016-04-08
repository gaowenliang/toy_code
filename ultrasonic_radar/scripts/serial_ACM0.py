#!/usr/bin/env python

__author__ = "mferguson@willowgarage.com (Michael Ferguson)"

import rospy
from rosserial_python import SerialClient, RosSerialServer
import multiprocessing

import sys
   
if __name__=="__main__":

    rospy.init_node("serial_node")
    rospy.loginfo("ROS Serial Python Node")

    port_name = rospy.get_param('~port','/dev/ttyACM0')
    baud = int(rospy.get_param('~baud','57600'))

    # TODO: should these really be global?
    tcp_portnum = int(rospy.get_param('/rosserial_embeddedlinux/tcp_port', '11411'))
    fork_server = rospy.get_param('/rosserial_embeddedlinux/fork_server', False)

    # TODO: do we really want command line params in addition to parameter server params?
    sys.argv = rospy.myargv(argv=sys.argv)
    if len(sys.argv) == 2 :
        port_name  = sys.argv[1]
    if len(sys.argv) == 3 :
        tcp_portnum = int(sys.argv[2])
    
    if port_name == "tcp" :
        server = RosSerialServer(tcp_portnum, fork_server)
        rospy.loginfo("Waiting for socket connections on port %d" % tcp_portnum)
        try:
            server.listen()
        except KeyboardInterrupt:
            rospy.loginfo("got keyboard interrupt")
        finally:
            rospy.loginfo("Shutting down")
            for process in multiprocessing.active_children():
                rospy.loginfo("Shutting down process %r", process)
                process.terminate()
                process.join()
            rospy.loginfo("All done")

    else :          # Use serial port 
        rospy.loginfo("Connecting to %s at %d baud" % (port_name,baud) )
        client = SerialClient(port_name, baud)
        try:
            client.run()
        except KeyboardInterrupt:
            pass

