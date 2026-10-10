from flask import Flask, render_template_string, request
from flask_socketio import SocketIO
import serial
import time

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)

@socketio.on("move")
def move(data):
    cmd = data["cmd"]
    print("SEND:", cmd)
    arduino.write((cmd + "\n").encode())

@app.route("/")
def index():
    host = request.host.split(":")[0]

    return render_template_string(f"""
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<script src="https://cdn.socket.io/4.5.4/socket.io.min.js"></script>

<style>
body {{
  background:#111;
  color:white;
  text-align:center;
  font-family:Arial;
}}
iframe {{
  border:none;
  margin-top:15px;
  border-radius:10px;
}}
</style>
</head>

<body>

<h2>Rover Control</h2>

<iframe src="http://{host}:8889/mystream"
width="640"
height="360"></iframe>

<p>Use W A S D keys</p>

<script>
const socket = io();

let keys = {{}};
let lastCmd = "";

// Track keys
document.addEventListener("keydown", (e) => {{
  keys[e.key.toLowerCase()] = true;
}});

document.addEventListener("keyup", (e) => {{
  keys[e.key.toLowerCase()] = false;
}});

// ? Control loop
setInterval(() => {{
  let left = 0;
  let right = 0;

  if (keys["w"]) {{
    left += 150;
    right += 150;
  }}
  if (keys["s"]) {{
    left -= 150;
    right -= 150;
  }}
  if (keys["a"]) {{
    left -= 100;
    right += 100;
  }}
  if (keys["d"]) {{
    left += 100;
    right -= 100;
  }}

  let cmd = (left === 0 && right === 0)
    ? "S"
    : "L" + left + ",R" + right;

  // ? only send if changed
  if (cmd !== lastCmd) {{
    socket.emit("move", {{cmd: cmd}});
    lastCmd = cmd;
    console.log("SEND:", cmd);
  }}

}}, 100);
</script>

</body>
</html>
""")

if __name__ == "__main__":
    socketio.run(app, host="0.0.0.0", port=5001)
