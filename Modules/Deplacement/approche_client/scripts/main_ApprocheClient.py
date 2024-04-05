#!/usr/bin/env python3


from homodeus_library.homodeus_precomp import *
from approche_client.approche_client import ApproachClient


def main() -> None:
  approcheClient = ApproachClient()
  rospy.spin()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass