from gtts import gTTS
import os
from typing import Any, Text, Dict, List
from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.types import DomainDict
from rasa_sdk.events import SlotSet

class ActionOrderDrink(Action):

    def name(self) -> Text:
        return "action_order_drink"

    def text_to_speech(self, text: str):
        tts = gTTS(text=text, lang='en')  # Change 'fr' to 'en' for English
        tts.save("message.mp3")
        os.system("mpg321 message.mp3")

    def run(self, dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict) -> List[Dict[Text, Any]]:

        ordered_drink = tracker.get_slot("ordered_drink")
    
        if ordered_drink is not None:
            message = f"Your order of {ordered_drink} has been successfully placed."
            print(f"Sending message: {message}")  # Debug print statement
            dispatcher.utter_message(text=message)
            self.text_to_speech(message)
            return [SlotSet("ordered_drink", ordered_drink)]
        else:
            message = "I'm sorry, I didn't understand. Could you please rephrase?"
            print(f"Sending message: {message}")  # Debug print statement
            dispatcher.utter_message(text=message)
            self.text_to_speech(message)
            return []

class ActionConfirmOrder(Action):

    def name(self) -> Text:
        return "action_confirm_order"

    def text_to_speech(self, text: str):
        tts = gTTS(text=text, lang='en')  # Change 'fr' to 'en' for English
        tts.save("message.mp3")
        os.system("mpg321 message.mp3")

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: DomainDict) -> List[Dict[Text, Any]]:

        ordered_drink = tracker.get_slot("ordered_drink")

        if ordered_drink is not None:
            message = f"Your order of {ordered_drink} has been confirmed. Thank you!"
            dispatcher.utter_message(text=message)
            self.text_to_speech(message)
        else:
            message = "I'm sorry, I didn't understand. Could you please rephrase?"
            dispatcher.utter_message(text=message)
            self.text_to_speech(message)

        return []