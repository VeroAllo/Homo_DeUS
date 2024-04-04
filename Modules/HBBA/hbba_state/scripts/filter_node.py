#!/usr/bin/env python3
import rospy
import hbba_lite

def main():
    rospy.init_node('goto_filter_state')
    _1 = hbba_lite.OnOffHbbaFilterState('goto/FilterState')
    _2 = hbba_lite.OnOffHbbaFilterState('talk/FilterState')
    _3 = hbba_lite.OnOffHbbaFilterState('take/FilterState')
    _4 = hbba_lite.OnOffHbbaFilterState('drop/FilterState')
    _5 = hbba_lite.OnOffHbbaFilterState('discuss/FilterState')
    _6 = hbba_lite.OnOffHbbaFilterState('explore/FilterState')
    rospy.spin()


if __name__ == '__main__':
    main()
