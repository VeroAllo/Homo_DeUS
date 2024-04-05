#!/usr/bin/env python3
# $HOME/tiago_public_ws/src/pmb2_simulation/pmb2_2dnav_gazebo/script/main_navSelector.py

from homodeus_library.homodeus_precomp import *
from NavSelector.NavSelector import NavSelector
from time import sleep


# TODO
#   Atteindre son but avec une orientation ish cree un decalage entre carte et pose du robot
#   Les objets fantomes restent une coupe de secondes (5 secs) ce qui peut causer un comportement non pres du robot (contour un osbtacle fantome)
#     ~planner_patience (double, default: 5.0)
#       How long the planner will wait in seconds in an attempt to find a valid plan before space-clearing operations are performed. 
#     ~controller_patience (double, default: 15.0)
#       How long the controller will wait in seconds without receiving a valid control before space-clearing operations are performed. 
#     ~conservative_reset_dist (double, default: 3.0)
#       The distance away from the robot in meters beyond which obstacles will be cleared from the costmap when attempting to clear space in the map. Note, this parameter is only used when the default recovery behaviors are used for move_base. 


sendSecondGoal = False
filename : str = "/home/urobot/tiago_public_ws/src/zhomodeus/NavSelector/scripts/predefNavGoal.json"

def f(a) -> None: #As we see, we can get events from the NavSelector in the controller
  print(f"Voici ce qu'on recoit comme résultat du goal : {convGoalStatus(a)}")
  global sendSecondGoal
  sendSecondGoal = True

def wait_for_goal_end() :
  global sendSecondGoal
  while(sendSecondGoal == False) :
    pass
  sendSecondGoal = False

def __node_shutdown():
  nav = NavSelector()
  nav.CancelAllGoals()

def main() -> None:
  
  global sendSecondGoal
  initRosNode("nav_selector")
  rospy.on_shutdown(__node_shutdown)
  hdInfo("Waiting for system init and arm to tuck, waiting 5 seconds")
  sleep(5)

  nav = NavSelector()
  nav.SetFilename(filename=filename)
  nav.ConnectCallBack(f)

  str_input : str = input("start recover (o/n) | 2D pose estimate: ")
  # if str_input != 'o':
  #   print('exit recover')
  #   return
  # nav.RelocateItselfInMap()
  
  str_input : str = input("start clear map (o/n) : ")
  if str_input != 'o':
    print('exit recover')
    return
  nav.ClearMap()
  
  for iter_circuit in range(5):
    str_input : str = input("start circuit " + str(iter_circuit) + " (o/n) : ")
    if str_input != 'o':
      print('exit laps circuit')
      break

    nav.LoadPreDefNavGoal()
    nav.SetIndexCurrentGoal(0)
    nav.UnblockAllGoals()

    for index_goal in range(5):
      str_input : str = input("send goal " + str(index_goal) + " (o/n) : ")
      if str_input != 'o':
        print('exit goal')
        break

      nav.SendGoal()
      wait_for_goal_end()


if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass