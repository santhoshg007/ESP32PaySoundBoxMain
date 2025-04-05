from flask import Flask, request, jsonify
import paho.mqtt.client as mqtt
import json
import os

app = Flask(__name__)

# Adafruit IO MQTT Settings
broker = "io.adafruit.com"
port = 1883
username = "SanthoshG"  # Replace with your Adafruit IO username
aio_key = ""
feed = "paymentTrigger"  # Replace with the feed you're interested in

# Initialize MQTT client for publishing to Adafruit IO
client = mqtt.Client()
client.username_pw_set(username, aio_key)
client.connect(broker, port, 60)

def on_connect(client, userdata, flags, rc):
    """Called when the MQTT client connects."""
    print(f"Connected with result code {rc}")

def on_disconnect(client, userdata, rc):
    """Called when the MQTT client disconnects."""
    print(f"Disconnected with result code {rc}")

client.on_connect = on_connect
client.on_disconnect = on_disconnect

# MQTT client loop
client.loop_start()

@app.route('/webhook', methods=['POST'])
def handle_webhook():
    """Handle the incoming PayPal webhook data."""
    try:
        # Get the JSON payload from PayPal webhook
        webhook_data = request.get_json()

        # Check if the necessary fields are present in the webhook data
        event_type = webhook_data.get("event_type", "")
        state = webhook_data.get("resource", {}).get("state", "")
        
        if not event_type or not state:
            return jsonify({"status": "error", "message": "Missing event_type or state"}), 400

        # Format the data for Adafruit IO (e.g., trigger and status)
        formatted_data = {
            "trigger": event_type,  # Use event_type as the trigger
            "status": state         # Use state (completed, etc.) as the status
        }
        
        # Publish the formatted data to Adafruit IO
        payload = json.dumps(formatted_data)
        client.publish(f"{username}/feeds/{feed}", payload)
        print(f"Published to Adafruit IO: {payload}")

        # Return a success response to PayPal
        return jsonify({"status": "success"}), 200

    except Exception as e:
        print(f"Error processing webhook: {str(e)}")
        return jsonify({"status": "error", "message": "Failed to process webhook"}), 500

if __name__ == '__main__':
    # Start Flask app
    app.run(debug=True, host='0.0.0.0', port=5000)  # Flask app running on port 5000
