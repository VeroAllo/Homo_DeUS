#!/usr/bin/env python3

import rospy
from rospy import Publisher, Subscriber
from homodeus_msgs.msg import HDResponse, HDDiscussionStarted, HDStatus
from discuss import AudioHandler

class AudioRosDiscuss():
    _DISCUSS_REQUEST_TOPIC = "/Homodeus/Behaviour/Discuss/Request"
    _DISCUSS_RESPONSE_TOPIC = "/Homodeus/Behaviour/Discuss/Response"
    _DISCUSS_STATUS_TOPIC = "/Homodeus/Behaviour/Discuss/Status"
    
    def __init__(self, node_name: str = 'base_rotate') -> None:
        # Initialise the node
        rospy.init_node(node_name, anonymous=True)

        # Initialise parameters
        self.__discusser: AudioHandler = AudioHandler()
        self.__desireID: int = 0

        # Subscriber
        self.__discuss_request_sub: Subscriber = Subscriber(self._DISCUSS_REQUEST_TOPIC, HDDiscussionStarted, self.__discuss_request_subscriber_callback)
              
        # Publisher
        self.__discuss_status_pub: Publisher = Publisher(self._DISCUSS_STATUS_TOPIC, HDStatus, queue_size=1)
        self.__discuss_response_pub: Publisher = Publisher(self._DISCUSS_RESPONSE_TOPIC, HDResponse, queue_size=1)

        rospy.on_shutdown(self.__close_connection_node)
        rospy.loginfo("Behavior Discuss initialized")
    
    def __discuss_request_subscriber_callback(self, msg: HDDiscussionStarted):
        self.__desireID = msg.id.desire_id
        rospy.loginfo("Request to discuss received")
        # self.__discusser.discuss(msg.message.data)
        self.__discusser.start()
        rospy.loginfo("Request to discuss finished")
        
        # Récupérer l'item sélectionné
        selected_item = self.__discusser.get_selected_item()
        
        # Publier l'item sélectionné sur le topic response
        response = HDResponse()
        response.id.desire_id = self.__desireID
        response.result = True
        response.selected_item = selected_item  # Assurez-vous que HDResponse a un champ pour l'item sélectionné
        self.__discuss_response_pub.publish(response)
        rospy.loginfo("Response to discuss sent with selected item: %s", selected_item)
    
    def __close_connection_node(self):
        self.__discuss_request_sub.unregister()
        self.__discuss_status_pub.unregister()
        self.__discuss_response_pub.unregister()
        rospy.loginfo("Closing Behavior Discuss node")