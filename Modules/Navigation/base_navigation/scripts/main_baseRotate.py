#!/usr/bin/env python3


# from homodeus_library.homodeus_precomp import *
from base_navigation.BaseRotate import BaseRotate


def main() -> None:
  rotato = BaseRotate()
  rospy.spin()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass