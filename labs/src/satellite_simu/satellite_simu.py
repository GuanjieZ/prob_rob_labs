#!/usr/bin/env python

import rospy


def main():
    rospy.init_node('satellite_simu')
    rospy.loginfo('starting satellite_simu')
    rospy.spin()
    rospy.loginfo('done')

if __name__=='__main__':
    main()
