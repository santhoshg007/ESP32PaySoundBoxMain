from pubnub.pnconfiguration import PNConfiguration
from pubnub.pubnub import PubNub
from pubnub.callbacks import SubscribeCallback  # Import the SubscribeCallback class
import uuid
import time

# Initialize the Pubnub Keys
# Replace them with your keysets
pub_key = "pub-c-8d7a0bbd-5ee4-4148-a1b1-da94fa53ff2f"
sub_key = "sub-c-42eb7c66-8e90-4ec6-a939-e5967dc2cfff"

def init():  # initialize the pubnub keys and start subscribing
    global pubnub  # Pubnub Initialization

    pnconfig = PNConfiguration()  # Create PNConfiguration object
    pnconfig.publish_key = pub_key
    pnconfig.subscribe_key = sub_key
    pnconfig.uuid = str(uuid.uuid4())  # Generate a UUID

    pubnub = PubNub(pnconfig)  # Pass PNConfiguration to PubNub constructor

    pubnub.add_listener(PubNubListener())  # Add listener (make sure to define this class)
    pubnub.subscribe().channels('paymentTrigger').with_presence().execute()

def get_message(controlCommand):
    if "trigger" in controlCommand:
        if controlCommand["trigger"] == "anything" and controlCommand["status"] == 1:
            print("payment received")
        else:
            pass
    else:
        pass

def callback(envelope, status):  # callback function now takes two arguments
    if status:
        print(status)
    print(envelope)
    if "requester" in envelope:
        get_message(envelope)
    else:
        pass

def presence_callback(envelope, status):  # presence callback takes two arguments
    if status:
        print(status)
    print(envelope)

def status_callback(envelope, status):  # status callback takes two arguments
    if status:
        print(status)
    if envelope.status_code == 200:
        print("PubNub Connected")
    else:
        print(f"PubNub Error: {envelope.status_code}")

class PubNubListener(SubscribeCallback):  # Inherit from SubscribeCallback
    def status(self, pubnub, status):  # Change here: accept pubnub instance as the first argument
        print("Status callback")
        print(status)

    def message(self, pubnub, message):
        print("Message callback")
        print(pubnub)
        print(message)

    def presence(self, pubnub, presence):
        print("Presence callback")
        print(presence)


if __name__ == '__main__':
    init()  # Initialize the Script

    try:
        while True:
            time.sleep(1)  # Keep the script running
    except KeyboardInterrupt:
        print("Stopping PubNub...")
        pubnub.stop()
