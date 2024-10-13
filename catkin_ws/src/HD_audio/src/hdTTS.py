#!/usr/bin/env python


import rospy
from actionlib import SimpleActionClient
from pal_interaction_msgs.msg import TtsAction, TtsGoal


ACTION_NAMESPACE = '/tts'
ACTION_NAME = TtsAction
TIMEOUT_SRV = 5
TIMEOUT_EXECUTE_GOAL = 30


class hdTTS():


  def __init__(self) -> None:
    try:
      self.__tts_client = SimpleActionClient(ACTION_NAMESPACE, ACTION_NAME)
      self.__tts_client.wait_for_server(timeout=rospy.Duration(TIMEOUT_SRV))
    except rospy.ServiceException as src_exc:
      print(f"Le service {ACTION_NAMESPACE} ne répond pas pcq {src_exc}")

    self.__goal = None


  def get_goal(self) -> TtsGoal:
    return self.__goal


  def set_goal(self, text: str, lang: str):
    # Create a goal to say our sentence
    self.__goal = TtsGoal()
    self.__goal.rawtext.text = text
    self.__goal.rawtext.lang_id = lang   # RFC 3006 code


  def talk_blocking(self):
    # Send the goal and wait
    self.__tts_client.send_goal_and_wait(self.__goal, 
                                         execute_timeout=rospy.Duration(TIMEOUT_EXECUTE_GOAL),
                                         preempt_timeout=rospy.Duration(1))


  def talk_unblocking(self):
    # Send the goal without wait
    self.__tts_client.send_goal(self.__goal)

