import paho.mqtt.client as mqtt
import uuid
import json
import time
import os

# Adafruit IO MQTT Settings
broker = "io.adafruit.com"
port = 1883
#username = ""  # Replace with your Adafruit IO username
aio_key = os.getenv("AIO_KEY")
feed = "paymenttrigger"  # Replace with the feed you're interested in

# Initialize MQTT client
client = mqtt.Client(client_id=str(uuid.uuid4()))  # Generate a unique UUID for the client

# Set username and password for authentication (AIO key as password)
client.username_pw_set(username, aio_key)

# Define the callback functions for MQTT
def on_connect(client, userdata, flags, rc):
    """ Callback for when the client connects to the MQTT broker """
    print(f"Connected with result code {rc}")
    # Subscribe to a feed (i.e., topic)
    client.subscribe(f"{username}/feeds/{feed}")  # Subscribe to the feed on Adafruit IO
    print(f"Subscribed to {username}/feeds/{feed}")

def on_message(client, userdata, msg):
    """ Callback for when a message is received on the subscribed topic """
    print(f"Message received on topic {msg.topic}: {msg.payload.decode()}")

    # Process the message content
    try:
        controlCommand = json.loads(msg.payload.decode())
        get_message(controlCommand)
    except json.JSONDecodeError:
        print("Received message is not a valid JSON")

def on_disconnect(client, userdata, rc):
    """ Callback for when the client disconnects """
    print(f"Disconnected with result code {rc}")

def get_message(controlCommand):
    """ Process control commands """
    print("Processing message:", controlCommand)  # Print the entire message for debugging

    # Ensure that the message contains the required keys
    if "trigger" in controlCommand and "status" in controlCommand:
        trigger = controlCommand["trigger"]
        status = controlCommand["status"]

        # Example condition: Check if payment was completed
        if trigger == "PAYMENT.ORDER.CREATED" and status == "completed":
            print("Payment successfully completed!")
        else:
            print(f"Received event: {trigger} with status: {status}")
    else:
        print("Message is missing required keys: 'trigger' and/or 'status'")

def init():  # Initialize MQTT client and connect to Adafruit IO
    # Set the callback functions
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect

    # Connect to the MQTT broker (Adafruit IO)
    client.connect(broker, port, 60)
    
    # Start the MQTT loop in a non-blocking way
    client.loop_start()

if __name__ == '__main__':
    init()  # Initialize the MQTT connection

    try:
        # Run the script and keep the connection active to receive messages
        while True:
            time.sleep(1)  # Keep the loop running to receive messages
    except KeyboardInterrupt:
        print("Stopping Adafruit IO MQTT...")
        client.loop_stop()  # Stop the MQTT client loop
        client.disconnect()  # Disconnect from the broker
