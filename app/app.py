from flask import Flask, render_template
from flask_socketio import SocketIO
import websocket
import threading
import base64
import json

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, cors_allowed_origins="*")

# WebSocket connection to the C++ Seasocks server
ws = None

def on_message(ws, message):
    # Assuming the message is binary image data
    base64_image = base64.b64encode(message).decode('utf-8')
    socketio.emit('image', base64_image)

def on_error(ws, error):
    print(f"WebSocket error: {error}")

def on_close(ws, close_status_code, close_msg):
    print(f"WebSocket connection closed: {close_status_code} - {close_msg}")

def on_open(ws):
    print("WebSocket connection opened")

def connect_to_cpp_server():
    global ws
    ws = websocket.WebSocketApp("ws://localhost:8080/ws",
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close,
                                on_open=on_open)
    ws.run_forever()

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('connect')
def handle_connect():
    print('Browser client connected')

@socketio.on('disconnect')
def handle_disconnect():
    print('Browser client disconnected')

@socketio.on('control')
def handle_control(data):
    print(f"Received control message: {data}")
    if ws and ws.sock and ws.sock.connected:
        ws.send(json.dumps(data))
    else:
        print("WebSocket to C++ server is not connected")

if __name__ == '__main__':
    # Start WebSocket client in a separate thread
    threading.Thread(target=connect_to_cpp_server, daemon=True).start()
    
    # Run Flask-SocketIO app
    socketio.run(app, debug=True, host='0.0.0.0', port=5000)