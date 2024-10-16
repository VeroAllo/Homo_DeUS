#!/usr/bin/env python

import rospy
from rospy import Publisher, Subscriber
from homodeus_msgs.msg import HDResponse, HDTextToTalk, HDStatus
from talk import AudioTalk


class AudioRos():
    _TALK_REQUEST_TOPIC = "/Homodeus/Behaviour/Talk/Request"
    _TALK_RESPONSE_TOPIC = "/Homodeus/Behaviour/Talk/Response"
    _TALK_STATUS_TOPIC = "/Homodeus/Behaviour/Talk/Status"
    

    def __init__(self, node_name: str = 'base_rotate') -> None:

        # Initialise the node
        rospy.init_node(node_name, anonymous=True)

        # Initialise parameters
        self.__talker: AudioTalk = AudioTalk()
        self.__desireID: int        = 0

        # Subscriber
        self.__talk_request_sub: Subscriber = Subscriber(self._TALK_REQUEST_TOPIC, HDTextToTalk, self.__talk_request_subscriber_callback)
              
        # Publisher
        self.__talk_status_pub: Publisher = Publisher(self._TALK_STATUS_TOPIC, HDStatus, queue_size=1)
        self.__talk_response_pub: Publisher = Publisher(self._TALK_RESPONSE_TOPIC, HDResponse, queue_size=1)

        rospy.on_shutdown(self.__close_connection_node)
        rospy.loginfo("Behavior Talk initialized")
    

    def __talk_request_subscriber_callback(self, msg: HDTextToTalk):
        # Tu pourrais recuperer le texte msg et appeler __talker.talk
        #  en dehors de la fct cb si tu ne veux pas "bloquer" la fct cb ...
        # Quoique la fct cb est aperiodique ...
        self.__desireID = msg.id.desire_id
        rospy.loginfo("Request to talk received")
        self.__talker.talk(msg.message.data)
        rospy.loginfo("Request to talk finished")
        response = HDResponse()
        response.id.desire_id = self.__desireID
        response.result = True
        self.__talk_response_pub.publish(response)
        rospy.loginfo("Response to talk sent")

    
    def __close_connection_node(self):
        self.__talk_request_sub.unregister()
        self.__talk_status_pub.unregister()
        self.__talk_response_pub.unregister()
        rospy.loginfo("Closing Behavior Talk node")


if __name__ == '__main__':
    try:
        audio_ros = AudioRos()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass