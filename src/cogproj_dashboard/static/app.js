/**
 * CogProj Operator Console - Frontend Application Logic
 * Coordinates WebSockets telemetry, Canvas Map rendering, and Safe Teleop.
 */

// Global State
let ws = null;
let currentMode = "STOPPED";
let robotPose = { x: 0, y: 0, yaw: 0, valid: false };
let latestMap = null;
let scanPoints = [];
let spatialOres = [];
let selectedGoal = null;
const activeKeys = new Set();
let teleopTimer = null;

// Teleop Speed Ceilings
const MAX_LINEAR_VEL = 0.10;   // m/s
const MAX_ANGULAR_VEL = 0.25;  // rad/s

// DOM Elements
const connStatus = document.getElementById("conn-status");
const activeModePill = document.getElementById("active-mode-pill");
const btnEstop = document.getElementById("btn-estop");

const cameraStream = document.getElementById("camera-stream");
const cameraBanner = document.getElementById("camera-overlay-banner");
const camFps = document.getElementById("cam-fps");
const detLatency = document.getElementById("det-latency");
const detModeBadge = document.getElementById("det-mode-badge");

const mapCanvas = document.getElementById("map-canvas");
const mapCtx = mapCanvas.getContext("2d");
const mapBanner = document.getElementById("map-overlay-banner");
const slamStatusBadge = document.getElementById("slam-status-badge");
const btnFitMap = document.getElementById("btn-fit-map");

const countTotal = document.getElementById("count-total");
const countResource = document.getElementById("count-resource");
const countRock = document.getElementById("count-rock");
const countLocalized = document.getElementById("count-localized");

const sensorCam = document.getElementById("sensor-cam");
const sensorLidar = document.getElementById("sensor-lidar");
const sensorCalib = document.getElementById("sensor-calib");
const sensorArmed = document.getElementById("sensor-armed");
const sensorPose = document.getElementById("sensor-pose");
const sensorNav = document.getElementById("sensor-nav");
const tracksTableBody = document.getElementById("tracks-table-body");

const btnModeStop = document.getElementById("btn-mode-stop");
const btnModeManual = document.getElementById("btn-mode-manual");
const btnModeAuto = document.getElementById("btn-mode-auto");
const manualControls = document.getElementById("manual-controls");
const autoControls = document.getElementById("auto-controls");
const deadmanState = document.getElementById("deadman-state");

const selectedGoalLbl = document.getElementById("selected-goal-lbl");
const btnSendGoal = document.getElementById("btn-send-goal");
const btnCancelGoal = document.getElementById("btn-cancel-goal");
const navStateLbl = document.getElementById("nav-state-lbl");

// Canvas Coordinate Transform
let mapScale = 20; // pixels per meter
let mapOffsetX = 0;
let mapOffsetY = 0;

// Initialize WebSocket
function connectWebSocket() {
  const loc = window.location;
  const protocol = loc.protocol === "https:" ? "wss:" : "ws:";
  const wsUrl = `${protocol}//${loc.host}/ws`;

  connStatus.textContent = "Connecting...";
  connStatus.className = "status-pill offline";

  ws = new WebSocket(wsUrl);

  ws.onopen = () => {
    connStatus.textContent = "ONLINE (WS Connected)";
    connStatus.className = "status-pill online";
  };

  ws.onclose = () => {
    connStatus.textContent = "OFFLINE (Disconnected)";
    connStatus.className = "status-pill offline";
    cameraBanner.classList.remove("hidden");
    stopTeleop();
    setTimeout(connectWebSocket, 1500);
  };

  ws.onerror = () => {
    ws.close();
  };

  ws.onmessage = (event) => {
    try {
      const msg = JSON.parse(event.data);
      handleSocketMessage(msg);
    } catch (e) {
      console.error("Malformed WS message:", e);
    }
  };
}

function handleSocketMessage(msg) {
  if (msg.type === "telemetry" && msg.state) {
    updateTelemetry(msg.state);
  } else if (msg.type === "image" && msg.data) {
    cameraStream.src = `data:image/jpeg;base64,${msg.data}`;
    cameraBanner.classList.add("hidden");
  } else if (msg.type === "map_update" && msg.map) {
    latestMap = msg.map;
    slamStatusBadge.textContent = "MAPPING ACTIVE";
    slamStatusBadge.className = "badge badge-real";
    mapBanner.classList.add("hidden");
    drawMap();
  } else if (msg.type === "mode_response") {
    updateModeUI(msg.mode);
  }
}

function updateTelemetry(s) {
  // Mode & Motion
  if (s.mode !== currentMode) {
    updateModeUI(s.mode);
  }

  // Camera & Detection
  camFps.textContent = `${s.camera_fps} FPS`;
  detLatency.textContent = `${Math.round(s.detector_latency_ms)} ms`;
  detModeBadge.textContent = s.detector_mode === "REAL" ? "REAL ONNX" : "MOCK SYNTHETIC";
  detModeBadge.className = s.detector_mode === "REAL" ? "badge badge-real" : "badge badge-mock";

  // Ore Counts
  if (s.counts) {
    countTotal.textContent = s.counts.total;
    countResource.textContent = s.counts.resource;
    countRock.textContent = s.counts.host_rock;
    countLocalized.textContent = s.counts.localized;
  }

  // Sensors & Health
  if (s.sensors) {
    sensorCam.textContent = s.sensors.camera_online ? "ONLINE (30 FPS)" : "OFFLINE";
    sensorCam.style.color = s.sensors.camera_online ? "var(--success)" : "var(--danger)";

    sensorLidar.textContent = s.sensors.lidar_online ? `ONLINE (${s.sensors.lidar_rate_hz} Hz)` : "OFFLINE";
    sensorLidar.style.color = s.sensors.lidar_online ? "var(--success)" : "var(--danger)";

    sensorCalib.textContent = s.sensors.calibration;
    sensorCalib.style.color = s.sensors.calibration === "CALIBRATED" ? "var(--success)" : "var(--warning)";

    sensorArmed.textContent = s.sensors.robot_armed ? "ARMED" : "DISARMED";
    sensorArmed.style.color = s.sensors.robot_armed ? "var(--success)" : "var(--danger)";
  }

  // Robot Pose
  if (s.robot_pose && s.robot_pose.valid) {
    robotPose = s.robot_pose;
    const deg = Math.round(robotPose.yaw * (180 / Math.PI));
    sensorPose.textContent = `(${robotPose.x}, ${robotPose.y}, ${deg}°) [${robotPose.frame}]`;
  }

  // LaserScan & Ore Points
  if (s.scan_points) scanPoints = s.scan_points;
  if (s.spatial_ores) spatialOres = s.spatial_ores;

  // Nav2 State
  if (s.nav_state) {
    sensorNav.textContent = s.nav_state;
    navStateLbl.textContent = s.nav_state;
    btnCancelGoal.disabled = (s.nav_state === "IDLE" || s.nav_state === "GOAL_REACHED" || s.nav_state === "GOAL_CANCELED");
  }

  // Active Tracks Table
  renderTracksTable(s.tracks || []);

  // Redraw Canvas
  drawMap();
}

function renderTracksTable(tracks) {
  if (tracks.length === 0) {
    tracksTableBody.innerHTML = '<tr><td colspan="5" class="empty-row">No active ore tracks detected</td></tr>';
    return;
  }

  let html = "";
  for (const t of tracks) {
    html += `<tr>
      <td><strong>#${t.id}</strong></td>
      <td><span class="badge ${t.class === 'resource' ? 'badge-real' : 'badge-mock'}">${t.class}</span></td>
      <td>${Math.round(t.confidence * 100)}%</td>
      <td>${t.state}</td>
      <td>${t.pos}</td>
    </tr>`;
  }
  tracksTableBody.innerHTML = html;
}

function updateModeUI(mode) {
  currentMode = mode;
  activeModePill.textContent = `MODE: ${mode}`;
  activeModePill.className = `status-pill mode-${mode.toLowerCase()}`;

  btnModeStop.classList.toggle("active-mode", mode === "STOPPED");
  btnModeManual.classList.toggle("active-mode", mode === "MANUAL");
  btnModeAuto.classList.toggle("active-mode", mode === "AUTONOMOUS");

  manualControls.classList.toggle("disabled", mode !== "MANUAL");
  autoControls.classList.toggle("disabled", mode !== "AUTONOMOUS");

  if (mode !== "MANUAL") {
    stopTeleop();
  }
}

// Map Canvas Drawing
function resizeCanvas() {
  const rect = mapCanvas.getBoundingClientRect();
  mapCanvas.width = rect.width;
  mapCanvas.height = rect.height;
  mapOffsetX = mapCanvas.width / 2;
  mapOffsetY = mapCanvas.height / 2;
  drawMap();
}

function drawMap() {
  mapCtx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);

  // 1. Draw Grid background
  mapCtx.strokeStyle = "#1e293b";
  mapCtx.lineWidth = 1;
  const gridSize = 1.0 * mapScale; // 1 meter grid
  for (let x = mapOffsetX % gridSize; x < mapCanvas.width; x += gridSize) {
    mapCtx.beginPath();
    mapCtx.moveTo(x, 0);
    mapCtx.lineTo(x, mapCanvas.height);
    mapCtx.stroke();
  }
  for (let y = mapOffsetY % gridSize; y < mapCanvas.height; y += gridSize) {
    mapCtx.beginPath();
    mapCtx.moveTo(0, y);
    mapCtx.lineTo(mapCanvas.width, y);
    mapCtx.stroke();
  }

  // 2. Draw Occupancy Grid (if present)
  if (latestMap) {
    const res = latestMap.resolution;
    const w = latestMap.width;
    const h = latestMap.height;
    const ox = latestMap.origin_x;
    const oy = latestMap.origin_y;
    const cells = latestMap.cells;

    for (let gy = 0; gy < h; gy++) {
      for (let gx = 0; gx < w; gx++) {
        const val = cells[gy * w + gx];
        if (val === -1) continue; // unknown

        const wx = ox + gx * res;
        const wy = oy + gy * res;
        const cx = mapOffsetX + wx * mapScale;
        const cy = mapOffsetY - wy * mapScale; // invert Y for screen coords

        if (val > 50) {
          mapCtx.fillStyle = "#0f172a"; // occupied (black)
        } else {
          mapCtx.fillStyle = "#f8fafc"; // free (white)
        }
        mapCtx.fillRect(cx, cy, res * mapScale + 1, res * mapScale + 1);
      }
    }
  }

  // 3. Draw LaserScan Points (Red dots)
  if (scanPoints.length > 0) {
    mapCtx.fillStyle = "#ef4444";
    for (const pt of scanPoints) {
      // transform scan point relative to robot pose
      const cosA = Math.cos(robotPose.yaw);
      const sinA = Math.sin(robotPose.yaw);
      const wx = robotPose.x + (pt[0] * cosA - pt[1] * sinA);
      const wy = robotPose.y + (pt[0] * sinA + pt[1] * cosA);

      const cx = mapOffsetX + wx * mapScale;
      const cy = mapOffsetY - wy * mapScale;

      mapCtx.beginPath();
      mapCtx.arc(cx, cy, 2, 0, Math.PI * 2);
      mapCtx.fill();
    }
  }

  // 4. Draw Spatial Ore Markers (Amber dots)
  if (spatialOres.length > 0) {
    for (const ore of spatialOres) {
      const cx = mapOffsetX + ore.x * mapScale;
      const cy = mapOffsetY - ore.y * mapScale;
      mapCtx.fillStyle = ore.class === "resource" ? "#fbbf24" : "#94a3b8";
      mapCtx.beginPath();
      mapCtx.arc(cx, cy, 6, 0, Math.PI * 2);
      mapCtx.fill();
      mapCtx.strokeStyle = "#fff";
      mapCtx.stroke();

      mapCtx.fillStyle = "#fff";
      mapCtx.font = "10px sans-serif";
      mapCtx.fillText(`Ore #${ore.track_id}`, cx + 8, cy + 3);
    }
  }

  // 5. Draw Robot (Blue circle with heading indicator)
  const rcx = mapOffsetX + robotPose.x * mapScale;
  const rcy = mapOffsetY - robotPose.y * mapScale;

  mapCtx.fillStyle = "#38bdf8";
  mapCtx.beginPath();
  mapCtx.arc(rcx, rcy, 8, 0, Math.PI * 2);
  mapCtx.fill();
  mapCtx.strokeStyle = "#0284c7";
  mapCtx.lineWidth = 2;
  mapCtx.stroke();

  // Heading arrow
  const arrowLen = 16;
  const hx = rcx + arrowLen * Math.cos(robotPose.yaw);
  const hy = rcy - arrowLen * Math.sin(robotPose.yaw);
  mapCtx.strokeStyle = "#38bdf8";
  mapCtx.beginPath();
  mapCtx.moveTo(rcx, rcy);
  mapCtx.lineTo(hx, hy);
  mapCtx.stroke();

  // 6. Draw Navigation Target Goal (if selected)
  if (selectedGoal) {
    const gcx = mapOffsetX + selectedGoal.x * mapScale;
    const gcy = mapOffsetY - selectedGoal.y * mapScale;

    mapCtx.fillStyle = "#22c55e";
    mapCtx.beginPath();
    mapCtx.arc(gcx, gcy, 6, 0, Math.PI * 2);
    mapCtx.fill();
    mapCtx.strokeStyle = "#fff";
    mapCtx.stroke();
    mapCtx.fillText("🎯 Goal", gcx + 8, gcy + 3);
  }
}

// Map Click -> Select Navigation Goal
mapCanvas.addEventListener("click", (e) => {
  if (currentMode !== "AUTONOMOUS") return;

  const rect = mapCanvas.getBoundingClientRect();
  const cx = e.clientX - rect.left;
  const cy = e.clientY - rect.top;

  const wx = (cx - mapOffsetX) / mapScale;
  const wy = -(cy - mapOffsetY) / mapScale;

  selectedGoal = { x: parseFloat(wx.toFixed(2)), y: parseFloat(wy.toFixed(2)), yaw: 0.0 };
  selectedGoalLbl.textContent = `X: ${selectedGoal.x} m, Y: ${selectedGoal.y} m, θ: 0°`;
  btnSendGoal.disabled = false;
  drawMap();
});

btnFitMap.addEventListener("click", () => {
  mapOffsetX = mapCanvas.width / 2 - robotPose.x * mapScale;
  mapOffsetY = mapCanvas.height / 2 + robotPose.y * mapScale;
  drawMap();
});

// Mode Buttons
btnModeStop.addEventListener("click", () => setMode("STOPPED"));
btnModeManual.addEventListener("click", () => setMode("MANUAL"));
btnModeAuto.addEventListener("click", () => setMode("AUTONOMOUS"));
btnEstop.addEventListener("click", () => {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: "estop" }));
  }
  updateModeUI("STOPPED");
});

function setMode(mode) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: "set_mode", mode }));
  }
}

// Autonomous Goal Actions
btnSendGoal.addEventListener("click", () => {
  if (!selectedGoal || currentMode !== "AUTONOMOUS") return;
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({
      type: "nav_goal",
      x: selectedGoal.x,
      y: selectedGoal.y,
      yaw: selectedGoal.yaw
    }));
  }
});

btnCancelGoal.addEventListener("click", () => {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: "cancel_nav" }));
  }
});

// Teleoperation Keyboard Controls
window.addEventListener("keydown", (e) => {
  if (currentMode !== "MANUAL") return;
  const key = e.key.toLowerCase();

  if (["w", "a", "s", "d", " "].includes(key)) {
    e.preventDefault();
    if (!activeKeys.has(key)) {
      activeKeys.add(key);
      highlightKey(key, true);
      sendTeleopFromKeys();
    }
  }
});

window.addEventListener("keyup", (e) => {
  if (currentMode !== "MANUAL") return;
  const key = e.key.toLowerCase();

  if (["w", "a", "s", "d", " "].includes(key)) {
    activeKeys.delete(key);
    highlightKey(key, false);
    if (activeKeys.size === 0) {
      stopTeleop();
    } else {
      sendTeleopFromKeys();
    }
  }
});

// Automatic Safety Stop on Focus Loss or Tab Hide
window.addEventListener("blur", () => {
  if (currentMode === "MANUAL") stopTeleop();
});
document.addEventListener("visibilitychange", () => {
  if (document.hidden && currentMode === "MANUAL") stopTeleop();
});

function sendTeleopFromKeys() {
  if (currentMode !== "MANUAL") return;

  if (activeKeys.has(" ")) {
    stopTeleop();
    return;
  }

  let lin = 0.0;
  let ang = 0.0;

  if (activeKeys.has("w")) lin += MAX_LINEAR_VEL;
  if (activeKeys.has("s")) lin -= MAX_LINEAR_VEL;
  if (activeKeys.has("a")) ang += MAX_ANGULAR_VEL;
  if (activeKeys.has("d")) ang -= MAX_ANGULAR_VEL;

  const deadman = (lin !== 0.0 || ang !== 0.0);

  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({
      type: "teleop",
      linear: lin,
      angular: ang,
      deadman: deadman
    }));
  }

  deadmanState.textContent = deadman ? `ACTIVE (Lin: ${lin.toFixed(2)} m/s, Ang: ${ang.toFixed(2)} rad/s)` : "IDLE (0.0 m/s)";
  deadmanState.style.color = deadman ? "var(--success)" : "var(--text-muted)";
}

function stopTeleop() {
  activeKeys.clear();
  document.querySelectorAll(".btn-key").forEach(b => b.classList.remove("active-key"));

  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({
      type: "teleop",
      linear: 0.0,
      angular: 0.0,
      deadman: false
    }));
  }

  deadmanState.textContent = "IDLE (0.0 m/s)";
  deadmanState.style.color = "var(--text-muted)";
}

function highlightKey(key, active) {
  const btn = document.querySelector(`.btn-key[data-key="${key}"]`);
  if (btn) {
    btn.classList.toggle("active-key", active);
  }
}

// On-screen Button Pointer Events
document.querySelectorAll(".btn-key").forEach(btn => {
  const key = btn.getAttribute("data-key");
  btn.addEventListener("mousedown", () => {
    if (currentMode === "MANUAL") {
      activeKeys.add(key);
      highlightKey(key, true);
      sendTeleopFromKeys();
    }
  });
  btn.addEventListener("mouseup", () => {
    if (currentMode === "MANUAL") {
      activeKeys.delete(key);
      highlightKey(key, false);
      if (activeKeys.size === 0) stopTeleop();
    }
  });
});

// Initialization
window.addEventListener("resize", resizeCanvas);
window.addEventListener("DOMContentLoaded", () => {
  resizeCanvas();
  connectWebSocket();
});
