#!/usr/bin/env python3
import rospy
import message_filters

from hbba_lite.msg import Int32Stamped

import hbba_lite


def callback(data1, data2):
    rospy.loginfo('Data received : {} {}'.format(data1.data, data2.data))


def main():
    rospy.init_node('goto_filter_state')
    _ = hbba_lite.OnOffHbbaFilterState('goto/FilterState')
    _2 = hbba_lite.OnOffHbbaFilterState('talk/FilterState')
    _3 = hbba_lite.OnOffHbbaFilterState('take/FilterState')
    _4 = hbba_lite.OnOffHbbaFilterState('drop/FilterState')
    _5 = hbba_lite.OnOffHbbaFilterState('discuss/FilterState')
    _6 = hbba_lite.OnOffHbbaFilterState('explore/FilterState')
    rospy.spin()


if __name__ == '__main__':
    main()
