#!/usr/bin/env python3
import rospy
from networktables import NetworkTables
import logging

if __name__ == '__main__':
    rospy.init_node("nt_sim")
    rospy.loginfo("Started NetworkTables simulator")
    logging.basicConfig(level=logging.DEBUG)
    NetworkTables.initialize()
    t1 = NetworkTables.getTable("table1")
    t2 = NetworkTables.getTable("table2")
    t1.getEntry("test1").setString("this is t1")
    t2.getEntry("test2").setString("this is t2")
    rospy.spin()