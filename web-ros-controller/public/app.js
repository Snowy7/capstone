/* global ROSLIB, nipplejs */
let ros = null;
let connected = false;
let cmdVel = null;
let mapInfo = null;
let robotPose = { x: 0, y: 0, theta: 0 };
let nav2Ready = false;
let currentMode = "mapping";
let currentMapName = "";
let currentParkingSpots = [];
let heartbeatTimer = null;
let emergencyStopActive = false;

const DEFAULT_ROS_URL = (() => {
  const host = location.hostname || "localhost";
  return `ws://${host}:9090`;
})();

const ROS_CONFIG = window.ROS_CONFIG || { url: DEFAULT_ROS_URL };

function $(id) { return document.getElementById(id); }
function setText(id, val) { const el = $(id); if (el) el.textContent = val; }

function updateConnUI(isConnected) {
  document.querySelectorAll('[data-role="conn-pill"]').forEach(p => {
    p.classList.toggle("connected", isConnected);
    const dot = p.querySelector(".dot");
    if (dot) dot.style.background = isConnected ? "#67e8a6" : "#ff6b6b";
    const label = p.querySelector(".label");
    if (label) label.textContent = isConnected ? "Connected" : "Disconnected";
  });
}

// Theme handling
(() => {
  // Force dark theme for now as per new design
  document.documentElement.setAttribute("data-theme", "dark");
  
  // Sidebar Toggle
  const burger = $("burgerToggle");
  const sidebar = $("sidebar");
  const scrim = $("sidebarScrim");
  
  function toggleSidebar() {
    sidebar.classList.toggle("open");
  }
  
  if (burger) burger.onclick = toggleSidebar;
  if (scrim) scrim.onclick = toggleSidebar;
})();

function connectROS() {
  try {
    ros = new ROSLIB.Ros({ url: ROS_CONFIG.url || DEFAULT_ROS_URL, timeout: 20000 });
  } catch (e) {
    console.error("ROSLIB init failed", e);
    return;
  }

  ros.on("connection", () => {
    connected = true;
    updateConnUI(true);
    cmdVel = new ROSLIB.Topic({
      ros, name: "/cmd_vel", messageType: "geometry_msgs/msg/Twist",
    });
    subAll();
    setAlert("info", "Robot connected");

    // Initial data fetches
  });

  ros.on("close", () => {
    connected = false;
    updateConnUI(false);
    unsubAll();
    setAlert("warn", "Disconnected. Retrying…");
    setTimeout(connectROS, 2000);
  });

  ros.on("error", () => setAlert("warn", "ROS bridge error"));
}

function setAlert(level, text) {
  const ts = new Date().toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" });
  const logEntry = { level, text, ts, id: Date.now() };
  
  // Save to session storage
  try {
    const logs = JSON.parse(sessionStorage.getItem("appLogs") || "[]");
    logs.unshift(logEntry);
    if (logs.length > 50) logs.pop(); // Keep last 50 logs
    sessionStorage.setItem("appLogs", JSON.stringify(logs));
  } catch (e) {
    console.error("Failed to save log:", e);
  }

  // Update UI if list exists
  addLogToUI(logEntry);
  
  // Always show toast notification
  showToast(level, text);
}

function addLogToUI(log) {
  const list = $("alertList");
  if (!list) return;
  
  const empty = $("alertsEmpty");
  if (empty) empty.style.display = "none";
  
  // Check if already exists (for restoration)
  if (document.getElementById(`log-${log.id}`)) return;

  const li = document.createElement("li");
  li.id = `log-${log.id}`;
  li.className = `alert-item ${log.level}`;
  li.innerHTML = `
    <i class="fa-solid ${
      log.level === "critical" ? "fa-circle-exclamation" :
      log.level === "warn" ? "fa-triangle-exclamation" : "fa-circle-info"
    }"></i>
    <div>
      <div>${log.text}</div>
      <div class="alert-meta">${log.ts}</div>
    </div>
  `;
  list.prepend(li);
  
  // Limit UI items
  while (list.children.length > 20) list.removeChild(list.lastChild);
}

// Restore logs on load
document.addEventListener("DOMContentLoaded", () => {
  try {
    const logs = JSON.parse(sessionStorage.getItem("appLogs") || "[]");
    // Add in reverse order so they end up in correct order when prepended
    logs.slice().reverse().forEach(addLogToUI);
  } catch (e) {
    console.error("Failed to restore logs:", e);
  }
});

function showToast(level, text) {
  let container = $("toastContainer");
  if (!container) {
    container = document.createElement("div");
    container.id = "toastContainer";
    container.style.cssText = `
      position: fixed;
      top: 20px;
      right: 20px;
      z-index: 10000;
      display: flex;
      flex-direction: column;
      gap: 10px;
      pointer-events: none;
    `;
    document.body.appendChild(container);
  }
  
  const toast = document.createElement("div");
  // Map level to new CSS variables
  const colors = {
    critical: "#ef4444",
    warn: "#f59e0b",
    info: "#3b82f6",
    success: "#10b981"
  };
  const color = colors[level] || colors.info;
  
  toast.style.cssText = `
    background: rgba(24, 27, 33, 0.95);
    border: 1px solid ${color};
    border-left: 4px solid ${color};
    color: #f3f4f6;
    padding: 12px 20px;
    border-radius: 8px;
    box-shadow: 0 10px 25px -5px rgba(0,0,0,0.5);
    display: flex;
    align-items: center;
    gap: 12px;
    min-width: 280px;
    max-width: 400px;
    pointer-events: auto;
    animation: slideIn 0.3s cubic-bezier(0.4, 0, 0.2, 1);
    font-size: 14px;
    font-weight: 500;
    backdrop-filter: blur(10px);
  `;
  
  const icon = level === "critical" ? "fa-circle-exclamation" :
               level === "warn" ? "fa-triangle-exclamation" : 
               level === "success" ? "fa-check-circle" : "fa-circle-info";
  
  toast.innerHTML = `
    <i class="fa-solid ${icon}" style="color: ${color}; font-size: 1.1em;"></i>
    <span>${text}</span>
  `;
  
  container.appendChild(toast);
  
  setTimeout(() => {
    toast.style.animation = "slideOut 0.3s cubic-bezier(0.4, 0, 0.2, 1) forwards";
    setTimeout(() => {
      if (toast.parentElement) toast.parentElement.removeChild(toast);
    }, 300);
  }, 4000);
}

// Add CSS animations for toasts
if (!document.getElementById("toastStyles")) {
  const style = document.createElement("style");
  style.id = "toastStyles";
  style.textContent = `
    @keyframes slideIn {
      from {
        transform: translateX(400px);
        opacity: 0;
      }
      to {
        transform: translateX(0);
        opacity: 1;
      }
    }
    @keyframes slideOut {
      from {
        transform: translateX(0);
        opacity: 1;
      }
      to {
        transform: translateX(400px);
        opacity: 0;
      }
    }
  `;
  document.head.appendChild(style);
}

let topics = {};
function subAll() {
  topics.odom = new ROSLIB.Topic({
    ros, name: "/odom", messageType: "nav_msgs/msg/Odometry", throttle_rate: 120,
  });
  topics.odom.subscribe(onOdom);

  topics.batt = new ROSLIB.Topic({
    ros, name: "/battery_state", messageType: "sensor_msgs/msg/BatteryState",
    throttle_rate: 1000,
  });
  topics.batt.subscribe(onBattery);

  // Subscribe to unified robot state (includes mode, map, nav2, pose)
  topics.state = new ROSLIB.Topic({
    ros, name: "/robot_state", messageType: "std_msgs/msg/String",
    throttle_rate: 100,
  });
  topics.state.subscribe(onRobotState);

  topics.map = new ROSLIB.Topic({
    ros, name: "/map", messageType: "nav_msgs/msg/OccupancyGrid",
    throttle_rate: 500,
  });
  topics.map.subscribe(onMap);

  if ($("mapLoading")) $("mapLoading").hidden = false;

  // Camera & Docking Subscriptions
  topics.camera = new ROSLIB.Topic({
    ros, name: "/docking/debug_image/compressed", messageType: "sensor_msgs/msg/CompressedImage",
    throttle_rate: 50, // 20Hz
  });
  topics.camera.subscribe(onCameraImage);

  topics.dockStatus = new ROSLIB.Topic({
    ros, name: "/docking/status", messageType: "std_msgs/msg/String",
    throttle_rate: 100,
  });
  topics.dockStatus.subscribe(onDockStatus);

  // System Stats Subscription
  topics.sysStats = new ROSLIB.Topic({
    ros, name: "/system_stats", messageType: "std_msgs/msg/String",
    throttle_rate: 1000,
  });
  topics.sysStats.subscribe(onSystemStats);

  topics.estopState = new ROSLIB.Topic({
    ros, name: "/emergency_stop_state", messageType: "std_msgs/msg/Bool",
    throttle_rate: 100,
  });
  topics.estopState.subscribe(onEmergencyStopState);

  restartHeartbeat();
}

function unsubAll() {
  Object.values(topics).forEach(t => { try { t.unsubscribe(); } catch {} });
  topics = {};
  if (heartbeatTimer) clearTimeout(heartbeatTimer);
  heartbeatTimer = null;
}

function onOdom(m) {
  const v = m.twist.twist.linear;
  const speed = Math.hypot(v.x, v.y);
  setText("rangeEstimate", `${(speed * 2).toFixed(1)} km`); // rough
  setText("lastUpdate", "just now");
  restartHeartbeat();
}

function onBattery(m) {
  const pct = (typeof m.percentage === "number")
    ? (m.percentage > 1 ? m.percentage : m.percentage * 100) : null;
  if (pct != null) setText("batteryLevel", `${pct.toFixed(0)}%`);
  if (typeof m.voltage === "number")
    setText("batteryVoltage", `${m.voltage.toFixed(1)} V`);
}

function onRobotState(msg) {
  try {
    const state = JSON.parse(msg.data);
    
    // Update robot pose
    robotPose.x = state.pose.x;
    robotPose.y = state.pose.y;
    robotPose.theta = state.pose.theta;
    setText("robotPos", `x: ${robotPose.x.toFixed(2)}, y: ${robotPose.y.toFixed(2)}`);
    setText("robotPos-setup", `x: ${robotPose.x.toFixed(2)}, y: ${robotPose.y.toFixed(2)}`);
    
    // Update mode
    if (state.mode !== currentMode) {
      currentMode = state.mode;
    }
    
    // Update current map
    const newMapName = state.map_name !== "none" ? state.map_name : "";
    if (newMapName !== currentMapName) {
      currentMapName = newMapName;
      // Refresh parking spots when map changes
      if (currentMapName && currentMode === "navigation") {
        refreshParking(currentMapName);
      }
    }
    updateMode(currentMode);
    
    // Update Nav2 status
    setNav2(state.nav2_ready ? "ready" : "stopped");
    
    // Trigger map redraw if following robot
    Object.values(mapViewports).forEach(vp => {
      if (vp.followRobot) {
        vp.needsRedraw = true;
      }
    });
    if (!animationFrameId && Object.values(mapViewports).some(v => v.followRobot)) {
      renderMaps();
    }
    
    restartHeartbeat();
  } catch (e) {
    console.error("Failed to parse robot state:", e);
  }
}

function onMap(msg) {
  mapInfo = msg.info;
  drawMap("map-dashboard", "mapSize-dash", "mapHint", msg);
  drawMap("map-setup", "mapSize-setup", "mapHint-setup", msg);
}

function onCameraImage(msg) {
  const img = $("cameraFeed");
  if (img) {
    img.src = "data:image/jpeg;base64," + msg.data;
  }
}

function onDockStatus(msg) {
  try {
    const status = JSON.parse(msg.data);
    setText("tagDetected", status.detected ? "YES" : "NO");
    setText("targetId", status.target_id !== -1 ? status.target_id : "--");
    setText("tagDistance", status.detected ? status.distance.toFixed(3) + " m" : "--");
    
    const overlay = $("dockingOverlay");
    if (overlay) {
      overlay.hidden = !status.is_docking;
    }
    
    const detEl = $("tagDetected");
    if (detEl) {
      detEl.style.color = status.detected ? "#67e8a6" : "inherit";
      detEl.style.fontWeight = status.detected ? "bold" : "normal";
    }
  } catch (e) {
    console.error("Error parsing dock status:", e);
  }
}

function onEmergencyStopState(msg) {
  emergencyStopActive = msg.data;
  updateEmergencyStopUI();
}

function onSystemStats(msg) {
  try {
    const stats = JSON.parse(msg.data);
    setText("cpuUsage", `${stats.cpu_percent.toFixed(1)}%`);
    setText("ramUsage", `${stats.ram_percent.toFixed(1)}%`);
    setText("diskUsage", `${stats.disk_percent.toFixed(1)}%`);
    
    // Power Stats
    if (stats.power) {
      setText("voltage", stats.power.voltage);
      const voltEl = $("voltage");
      if (voltEl) {
        // Highlight if undervoltage
        voltEl.style.color = stats.power.undervoltage_now ? "var(--danger)" : "var(--text-main)";
      }
      
      const throttleEl = $("throttleStatus");
      if (throttleEl) {
        if (stats.power.undervoltage_now) {
          throttleEl.textContent = "UNDERVOLTAGE";
          throttleEl.className = "badge status critical";
        } else if (stats.power.throttled_now) {
          throttleEl.textContent = "THROTTLED";
          throttleEl.className = "badge status warn";
        } else if (stats.power.undervoltage_occurred) {
          throttleEl.textContent = "Past Undervolt";
          throttleEl.className = "badge status warn";
        } else {
          throttleEl.textContent = "Normal";
          throttleEl.className = "badge status success";
        }
      }
    }

    const nodeList = $("nodeList");
    if (nodeList && stats.nodes) {
      nodeList.innerHTML = stats.nodes.map(n => `<li>${n}</li>`).join("");
    }
    
    // Kernel Logs
    const kLogList = $("kernelLogList");
    if (kLogList && stats.logs) {
      // Only update if different to avoid flickering/scroll jump
      const newLogs = stats.logs.join("\n");
      if (kLogList.dataset.lastLogs !== newLogs) {
        kLogList.innerHTML = stats.logs.map(l => `<div class="log-line">${l}</div>`).join("");
        kLogList.dataset.lastLogs = newLogs;
        // Auto scroll to bottom
        kLogList.scrollTop = kLogList.scrollHeight;
      }
    }
  } catch (e) {
    console.error("Error parsing system stats:", e);
  }
}

function toggleEmergencyStop() {
  const newState = !emergencyStopActive;
  
  callService("/emergency_stop", "interfaces/srv/EmergencyStop",
    { enable: newState },
    (res) => {
      if (res.success) {
        emergencyStopActive = res.is_stopped;
        updateEmergencyStopUI();
        setAlert(newState ? "critical" : "info", res.message);
      } else {
        setAlert("warn", "Failed: " + res.message);
      }
    },
    (err) => {
      setAlert("warn", "Emergency stop service error: " + err);
    });
}

function updateEmergencyStopUI() {
  const btns = document.querySelectorAll('[id*="mergency"]');
  btns.forEach(btn => {
    if (!btn) return;
    
    if (emergencyStopActive) {
      btn.classList.add("active");
      btn.classList.add("estop-active");
      btn.title = "RESET EMERGENCY STOP";
      const icon = btn.querySelector("i");
      if (icon) {
        icon.className = "fa-solid fa-rotate-right";
      }
    } else {
      btn.classList.remove("active");
      btn.classList.remove("estop-active");
      btn.title = "EMERGENCY STOP";
      const icon = btn.querySelector("i");
      if (icon) {
        icon.className = "fa-solid fa-hand";
      }
    }
  });
}

// Bind emergency button globally
document.addEventListener("DOMContentLoaded", () => {
  const eBtn = $("globalEmergencyBtn");
  if (eBtn) eBtn.onclick = toggleEmergencyStop;
});

// Map viewport state per canvas
let mapViewports = {};
let mapCache = {};
let animationFrameId = null;

function getViewport(canvasId) {
  if (!mapViewports[canvasId]) {
    mapViewports[canvasId] = {
      offsetX: 0,
      offsetY: 0,
      scale: 1,
      isDragging: false,
      startX: 0,
      startY: 0,
      followRobot: true,
      needsRedraw: true
    };
  }
  return mapViewports[canvasId];
}

function drawMap(canvasId, sizeId, hintId, msg) {
  const c = $(canvasId);
  if (!c) return;
  
  const { width, height, resolution } = msg.info;
  const data = msg.data;
  
  // Cache the map data if changed
  const cacheKey = `${width}_${height}_${data.length}`;
  if (!mapCache[canvasId] || mapCache[canvasId].key !== cacheKey) {
    mapCache[canvasId] = {
      key: cacheKey,
      width,
      height,
      resolution,
      data,
      offscreenCanvas: null
    };
  }
  
  const viewport = getViewport(canvasId);
  viewport.needsRedraw = true;
  
  // Initial scale to fit map in canvas
  if (viewport.scale === 1 && width > 0 && height > 0) {
    const containerWidth = c.parentElement.clientWidth;
    const containerHeight = c.parentElement.clientHeight;
    viewport.scale = Math.min(containerWidth / width, containerHeight / height) * 0.9;
  }
  
  const hint = $(hintId);
  if (hint) {
    const ts = new Date().toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" });
    hint.textContent = `Updated ${ts}`;
    hint.className = "badge status success";
  }
  const sizeEl = $(sizeId);
  if (sizeEl) sizeEl.textContent = `${width} \u00d7 ${height}`;
  
  // Hide loading overlay - use both hidden attr and display style for reliability
  const overlay = $(canvasId === "map-dashboard" ? "mapLoading" : "mapLoading-setup");
  if (overlay) {
    overlay.hidden = true;
    overlay.style.display = "none";
  }
  
  // Start render loop if not already running
  if (!animationFrameId) {
    renderMaps();
  }
}

function renderMaps() {
  let anyNeedsRedraw = false;
  
  Object.keys(mapCache).forEach(canvasId => {
    const c = $(canvasId);
    if (!c) return;
    
    const viewport = getViewport(canvasId);
    // Always redraw if following robot to update robot position
    if (viewport.followRobot) {
      viewport.needsRedraw = true;
    }
    if (!viewport.needsRedraw) return;
    
    anyNeedsRedraw = true;
    const cache = mapCache[canvasId];
    const { width, height, resolution, data } = cache;
    
    // Get container size (fixed, not growing)
    const container = c.parentElement;
    const containerWidth = container.clientWidth;
    const containerHeight = container.clientHeight;
    
    // Only resize canvas if container size changed
    const dpr = window.devicePixelRatio || 1;
    const targetWidth = Math.floor(containerWidth * dpr);
    const targetHeight = Math.floor(containerHeight * dpr);
    
    if (c.width !== targetWidth || c.height !== targetHeight) {
      c.width = targetWidth;
      c.height = targetHeight;
    }
    
    const ctx = c.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    
    const canvasWidth = containerWidth;
    const canvasHeight = containerHeight;
    
    // Fill with dark background
    ctx.fillStyle = "#0f1115";
    ctx.fillRect(0, 0, canvasWidth, canvasHeight);
    
    const scale = viewport.scale;
    
    // If following robot, center on robot position
    if (viewport.followRobot && mapInfo) {
      const mx = (robotPose.x - mapInfo.origin.position.x) / mapInfo.resolution;
      const my = (robotPose.y - mapInfo.origin.position.y) / mapInfo.resolution;
      viewport.offsetX = canvasWidth / 2 - mx * scale;
      viewport.offsetY = canvasHeight / 2 - (height - my - 1) * scale;
    }
    
    ctx.save();
    ctx.translate(viewport.offsetX, viewport.offsetY);
    
    // Draw map cells (optimized)
    const minX = Math.max(0, Math.floor(-viewport.offsetX / scale));
    const minY = Math.max(0, Math.floor(-viewport.offsetY / scale));
    const maxX = Math.min(width, Math.ceil((canvasWidth - viewport.offsetX) / scale));
    const maxY = Math.min(height, Math.ceil((canvasHeight - viewport.offsetY) / scale));
    
    for (let y = minY; y < maxY; y++) {
      for (let x = minX; x < maxX; x++) {
        const i = x + (height - y - 1) * width;
        const v = data[i];
        if (v === -1) continue; 
        
        let color;
        if (v === 100) color = "#3b82f6"; // Obstacle: Blue
        else if (v === 0) color = "rgba(255,255,255,0.08)"; // Free: Very faint white
        else color = "rgba(59, 130, 246, 0.3)"; // Unknown/Other
        
        ctx.fillStyle = color;
        ctx.fillRect(x * scale, y * scale, scale, scale);
      }
    }

    // Robot overlay - stylized robot icon
    if (mapInfo) {
      const mx = (robotPose.x - mapInfo.origin.position.x) / mapInfo.resolution;
      const my = (robotPose.y - mapInfo.origin.position.y) / mapInfo.resolution;
      const cx = mx * scale;
      const cy = (height - my - 1) * scale;

      const robotSize = Math.max(8, (0.25 / mapInfo.resolution) * scale);

      ctx.save();
      ctx.translate(cx, cy);
      ctx.rotate(-robotPose.theta);
      
      // Robot body (rounded rectangle)
      const bodyWidth = robotSize * 1.6;
      const bodyHeight = robotSize * 1.2;
      ctx.beginPath();
      ctx.roundRect(-bodyWidth/2, -bodyHeight/2, bodyWidth, bodyHeight, robotSize * 0.3);
      
      // Gradient fill for depth
      const gradient = ctx.createLinearGradient(0, -bodyHeight/2, 0, bodyHeight/2);
      gradient.addColorStop(0, "rgba(77, 163, 255, 0.95)");
      gradient.addColorStop(1, "rgba(77, 163, 255, 0.75)");
      ctx.fillStyle = gradient;
      ctx.fill();
      
      // Border
      ctx.strokeStyle = "rgba(140, 200, 255, 1)";
      ctx.lineWidth = Math.max(1.5, robotSize * 0.15);
      ctx.stroke();
      
      // Direction indicator (front chevron)
      ctx.beginPath();
      ctx.moveTo(bodyWidth/2 - robotSize * 0.2, -robotSize * 0.35);
      ctx.lineTo(bodyWidth/2 + robotSize * 0.3, 0);
      ctx.lineTo(bodyWidth/2 - robotSize * 0.2, robotSize * 0.35);
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = Math.max(2, robotSize * 0.2);
      ctx.lineCap = "round";
      ctx.lineJoin = "round";
      ctx.stroke();
      
      // Inner detail lines
      ctx.strokeStyle = "rgba(140, 200, 255, 0.5)";
      ctx.lineWidth = Math.max(1, robotSize * 0.08);
      ctx.beginPath();
      ctx.moveTo(-bodyWidth/2 + robotSize * 0.3, -bodyHeight/2 + robotSize * 0.3);
      ctx.lineTo(-bodyWidth/2 + robotSize * 0.3, bodyHeight/2 - robotSize * 0.3);
      ctx.stroke();
      
      ctx.restore();
      
      // Position trail/glow effect
      ctx.save();
      ctx.translate(cx, cy);
      ctx.beginPath();
      ctx.arc(0, 0, robotSize * 1.8, 0, Math.PI * 2);
      const glowGradient = ctx.createRadialGradient(0, 0, 0, 0, 0, robotSize * 1.8);
      glowGradient.addColorStop(0, "rgba(77, 163, 255, 0.15)");
      glowGradient.addColorStop(1, "rgba(77, 163, 255, 0)");
      ctx.fillStyle = glowGradient;
      ctx.fill();
      ctx.restore();
    }
    
    // Draw parking spots
    if (mapInfo && currentParkingSpots.length > 0) {
      currentParkingSpots.forEach(spot => {
        const sx = (spot.x - mapInfo.origin.position.x) / mapInfo.resolution;
        const sy = (spot.y - mapInfo.origin.position.y) / mapInfo.resolution;
        const spotX = sx * scale;
        const spotY = (height - sy - 1) * scale;
        
        const spotSize = Math.max(6, (0.15 / mapInfo.resolution) * scale);
        
        ctx.save();
        ctx.translate(spotX, spotY);
        
        // Outer glow
        ctx.beginPath();
        ctx.arc(0, 0, spotSize * 1.5, 0, Math.PI * 2);
        const spotGlow = ctx.createRadialGradient(0, 0, 0, 0, 0, spotSize * 1.5);
        spotGlow.addColorStop(0, "rgba(255, 193, 7, 0.3)");
        spotGlow.addColorStop(1, "rgba(255, 193, 7, 0)");
        ctx.fillStyle = spotGlow;
        ctx.fill();
        
        // Parking spot marker (P icon style)
        ctx.beginPath();
        ctx.arc(0, 0, spotSize, 0, Math.PI * 2);
        ctx.fillStyle = "rgba(255, 193, 7, 0.9)";
        ctx.fill();
        ctx.strokeStyle = "rgba(255, 220, 100, 1)";
        ctx.lineWidth = Math.max(1.5, spotSize * 0.2);
        ctx.stroke();
        
        // P letter
        ctx.fillStyle = "#ffffff";
        ctx.font = `bold ${spotSize * 1.2}px Arial`;
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText("P", 0, 0);
        
        ctx.restore();
      });
    }
    
    ctx.restore();
    viewport.needsRedraw = false;
  });
  
  if (anyNeedsRedraw || Object.values(mapViewports).some(v => v.followRobot)) {
    animationFrameId = requestAnimationFrame(renderMaps);
  } else {
    animationFrameId = null;
  }
}

function updateMode(mode) {
  currentMode = mode.toLowerCase();
  const badge = $("modeBadge");
  if (badge) {
    let text = currentMode === "navigation" ? "Navigation" : "Mapping";
    if (currentMode === "navigation" && currentMapName) {
      text += ` (${currentMapName})`;
    }
    badge.textContent = text;
  }
  document.querySelectorAll("#btn-mode-mapping,#btn-mode-nav").forEach(el => {
    if (!el) return;
    const isMapping = el.id === "btn-mode-mapping";
    el.classList.toggle("active",
      (isMapping && currentMode === "mapping") ||
      (!isMapping && currentMode === "navigation"));
  });
}

function setNav2(state) {
  nav2Ready = (state === "ready");
  const el = $("nav2Status");
  if (!el) return;
  el.textContent = nav2Ready ? "Nav2 Ready" : (state === "starting" ? "Nav2 Starting…" : "Nav2 Stopped");
  el.className = `badge ${nav2Ready ? "success" : "muted"}`;
}

function restartHeartbeat() {
  if (heartbeatTimer) clearTimeout(heartbeatTimer);
  setText("lastUpdate", "just now");
  heartbeatTimer = setTimeout(() => setText("lastUpdate", "stale"), 5000);
}

// Services
function callService(name, type, req, onOk, onErr) {
  const svc = new ROSLIB.Service({ ros, name, serviceType: type });
  const request = new ROSLIB.ServiceRequest(req || {});
  svc.callService(request, (res) => onOk && onOk(res),
    (err) => onErr ? onErr(err) : setAlert("warn", String(err)));
}

function setMode(mode, mapName="") {
  console.log("Setting mode:", mode, "Map:", mapName);
  
  // Show immediate feedback
  setAlert("info", `Switching to ${mode} mode...`);
  
  callService("/set_mode", "interfaces/srv/SetMode",
    { mode, map_name: mapName },
    (res) => {
      if (res.success) {
        // Don't update mode here - let the /robot_mode topic handle it
        setAlert("info", res.message);
        // Refresh parking spots if switching to navigation with a map
        if (mode === "navigation" && mapName) {
          setTimeout(() => refreshParking(mapName), 2000);
        }
      } else {
        setAlert("warn", res.message);
      }
    },
    (err) => {
      // Even on timeout, the mode switch may still succeed
      setAlert("info", "Mode switch initiated (checking status...)");
    });
}

function listMaps() {
  callService("/list_maps", "interfaces/srv/ListMaps", {},
    (res) => {
      showMapSelectionModal(res.maps);
    });
}

function showMapSelectionModal(maps) {
  // Create modal
  const modal = document.createElement("div");
  modal.className = "modal-overlay";
  modal.innerHTML = `
    <div class="modal-content">
      <h2><i class="fa-solid fa-map"></i> Select Map for Navigation</h2>
      <p>Choose a map to load for navigation mode</p>
      <ul class="map-list" id="modalMapsList"></ul>
      <button class="btn-cancel" id="cancelMapSelection">
        <i class="fa-solid fa-times"></i> Cancel
      </button>
    </div>
  `;
  document.body.appendChild(modal);
  
  const ul = modal.querySelector("#modalMapsList");
  maps.forEach(name => {
    const li = document.createElement("li");
    li.className = "map-item";
    li.innerHTML = `<i class="fa-solid fa-map-location-dot"></i><span class="map-name">${name}</span>`;
    li.onclick = () => {
      document.body.removeChild(modal);
      switchToNavigationMode(name);
    };
    ul.appendChild(li);
  });
  
  $("cancelMapSelection").onclick = () => {
    document.body.removeChild(modal);
  };
  
  modal.onclick = (e) => {
    if (e.target === modal) {
      document.body.removeChild(modal);
    }
  };
}

function switchToNavigationMode(mapName) {
  currentMapName = mapName;
  setAlert("info", `Switching to navigation with ${mapName}...`);
  setMode("navigation", mapName);
}



function saveMap(name) {
  callService("/save_map", "interfaces/srv/SaveMap",
    { map_name: name || "" },
    (res) => {
      const el = $("saveMapStatus");
      if (!el) return;
      if (res.success) {
        el.className = "save-status success";
        el.textContent = `Saved → ${res.file_path}`;
      } else {
        el.className = "save-status error";
        el.textContent = res.message;
      }
    });
}

function navigateTo(x, y, theta, label="", spot_id="") {
  if (!nav2Ready) { setAlert("warn", "Nav2 not ready"); return; }
  callService("/navigate_to_goal", "interfaces/srv/NavigateToGoal",
    { x, y, theta, label, spot_id },
    (res) => setAlert(res.success ? "info" : "warn",
      res.success ? "Goal accepted" : res.message));
}

function saveParking(label) {
  const mapName = currentMapName;
  console.log("Map name for saving parking spot:", mapName);
  if (!mapName) { 
    setAlert("warn", "No map loaded. Switch to navigation mode first."); 
    return; 
  }
  console.log("Saving parking spot:", label);
  callService("/save_parking_spot", "interfaces/srv/SaveParkingSpot",
    { spot_id: "", label, map_name: mapName,
      x: robotPose.x, y: robotPose.y, theta: robotPose.theta },
    (res) => {
      if (res.success) {
        setAlert("info", `Saved parking (${label})`);
        refreshParking(mapName);
      } else setAlert("warn", res.message);
    });
}

function refreshParking(mapName) {
  callService("/list_parking_spots", "interfaces/srv/ListParkingSpots",
    { map_name: mapName },
    (res) => {
      const ul1 = $("parkingSpotsListDash");
      if (ul1) ul1.innerHTML = "";
      const total = res.spot_ids.length;
      setText("parkingSummary", `${total} saved`);
      
      // Store parking spots globally for map rendering
      currentParkingSpots = [];
      for (let i = 0; i < total; i++) {
        const spot = {
          spot_id: res.spot_ids[i],
          label: res.labels[i],
          x: res.x[i], y: res.y[i], theta: res.theta[i],
        };
        currentParkingSpots.push(spot);
        
        if (ul1) {
          const li = document.createElement("li");
          li.className = "parking-spot-item";
          li.innerHTML = `
            <div class="spot-info">
              <span class="spot-label">${spot.label}</span>
              <span class="spot-coords">(${spot.x.toFixed(2)}, ${spot.y.toFixed(2)})</span>
            </div>
            <div class="spot-actions">
              <button class="btn" data-id="${spot.spot_id}">
                <i class="fa-solid fa-location-arrow"></i>
              </button>
              <button class="btn danger" data-del="${spot.spot_id}">
                <i class="fa-solid fa-trash"></i>
              </button>
            </div>`;
          ul1.appendChild(li);
        }
      }
      
      // Trigger map redraw to show parking spots
      Object.values(mapViewports).forEach(vp => {
        vp.needsRedraw = true;
      });
      if (!animationFrameId) renderMaps();
      
      document.querySelectorAll('[data-id]').forEach(b => {
        b.onclick = () => {
          const id = b.getAttribute("data-id");
          const idx = res.spot_ids.indexOf(id);
          navigateTo(res.x[idx], res.y[idx], res.theta[idx], res.labels[idx], res.spot_ids[idx]);
        };
      });
      document.querySelectorAll('[data-del]').forEach(b => {
        b.onclick = () => {
          const id = b.getAttribute("data-del");
          callService("/delete_parking_spot",
            "interfaces/srv/DeleteParkingSpot", { spot_id: id },
            () => refreshParking(mapName));
        };
      });
    });
}

// Manual controls
let currentLinear = { x: 0, y: 0 };
let currentAngular = 0;
let controlTimer = null;

function sendTwist(lx, ly, az) {
  if (!connected || !cmdVel) return;
  cmdVel.publish(new ROSLIB.Message({
    linear: { x: lx, y: ly, z: 0 }, angular: { x: 0, y: 0, z: az },
  }));
}

function updateControlLoop() {
  sendTwist(currentLinear.x, currentLinear.y, currentAngular);
}

function startControlLoop() {
  if (controlTimer) return;
  controlTimer = setInterval(updateControlLoop, 50);
}

function stopControlLoop() {
  if (controlTimer) {
    clearInterval(controlTimer);
    controlTimer = null;
  }
  currentLinear = { x: 0, y: 0 };
  currentAngular = 0;
  sendTwist(0, 0, 0);
  updateJoystickDisplay();
}

function updateJoystickDisplay() {
  const linearSpeed = Math.hypot(currentLinear.x, currentLinear.y);
  const maxSpeed = 1.0;
  const percentage = Math.min((linearSpeed / maxSpeed) * 100, 100);
  
  setText("joystickLinear", `${linearSpeed.toFixed(2)} m/s`);
  setText("joystickAngular", `${Math.abs(currentAngular).toFixed(2)} rad/s`);
  
  const speedBar = $("speedBar");
  if (speedBar) {
    speedBar.style.width = `${percentage}%`;
  }
}

// Joystick setup
function setupJoystick() {
  const zone = $("joystickZone");
  if (!zone || typeof nipplejs === "undefined") return;

  // Use dynamic mode - joystick appears where you touch
  const joystick = nipplejs.create({
    zone: zone,
    mode: "dynamic",
    color: "#4da3ff",
    size: 150,
    restOpacity: 0.5,
  });

  joystick.on("start", () => {
    zone.classList.add("active");
    startControlLoop();
  });

  joystick.on("move", (evt, data) => {
    if (!data.vector) return;
    
    const maxSpeed = 1.0; // m/s
    
    // data.vector gives normalized values from -1 to 1
    // vector.y: positive = up (forward), negative = down (backward)
    // vector.x: positive = right, negative = left
    // ROS: x = forward/back, y = left/right (positive y = left)
    currentLinear.x = data.vector.y * maxSpeed;   // Forward/backward
    currentLinear.y = -data.vector.x * maxSpeed;  // Left/right (invert for ROS)
    
    updateJoystickDisplay();
  });

  joystick.on("end", () => {
    zone.classList.remove("active");
    currentLinear = { x: 0, y: 0 };
    updateJoystickDisplay();
    if (currentAngular === 0) {
      stopControlLoop();
    }
  });
}

// Rotation controls
function setupRotationControls() {
  const btnRotLeft = $("btn-rot-left");
  const btnRotRight = $("btn-rot-right");
  
  if (!btnRotLeft || !btnRotRight) return;

  const rotSpeed = 0.8; // rad/s

  // Left rotation
  const startRotLeft = () => {
    currentAngular = rotSpeed;
    startControlLoop();
    updateJoystickDisplay();
  };

  const stopRotLeft = () => {
    currentAngular = 0;
    updateJoystickDisplay();
    if (currentLinear.x === 0 && currentLinear.y === 0) {
      stopControlLoop();
    }
  };

  btnRotLeft.addEventListener("mousedown", startRotLeft);
  btnRotLeft.addEventListener("mouseup", stopRotLeft);
  btnRotLeft.addEventListener("mouseleave", stopRotLeft);
  btnRotLeft.addEventListener("touchstart", (e) => {
    e.preventDefault();
    startRotLeft();
  });
  btnRotLeft.addEventListener("touchend", (e) => {
    e.preventDefault();
    stopRotLeft();
  });

  // Right rotation
  const startRotRight = () => {
    currentAngular = -rotSpeed;
    startControlLoop();
    updateJoystickDisplay();
  };

  const stopRotRight = () => {
    currentAngular = 0;
    updateJoystickDisplay();
    if (currentLinear.x === 0 && currentLinear.y === 0) {
      stopControlLoop();
    }
  };

  btnRotRight.addEventListener("mousedown", startRotRight);
  btnRotRight.addEventListener("mouseup", stopRotRight);
  btnRotRight.addEventListener("mouseleave", stopRotRight);
  btnRotRight.addEventListener("touchstart", (e) => {
    e.preventDefault();
    startRotRight();
  });
  btnRotRight.addEventListener("touchend", (e) => {
    e.preventDefault();
    stopRotRight();
  });
}

// Map controls
function setupMapControls() {
  const canvases = ["map-setup", "map-dashboard"];
  
  canvases.forEach(canvasId => {
    const canvas = $(canvasId);
    if (!canvas) return;
    
    const viewport = getViewport(canvasId);
    
    // Mouse drag
    canvas.addEventListener("mousedown", (e) => {
      viewport.isDragging = true;
      viewport.followRobot = false;
      viewport.startX = e.clientX - viewport.offsetX;
      viewport.startY = e.clientY - viewport.offsetY;
      canvas.style.cursor = "grabbing";
      updateFollowButtons();
    });
    
    canvas.addEventListener("mousemove", (e) => {
      if (!viewport.isDragging) return;
      viewport.offsetX = e.clientX - viewport.startX;
      viewport.offsetY = e.clientY - viewport.startY;
      viewport.needsRedraw = true;
      if (!animationFrameId) renderMaps();
    });
    
    canvas.addEventListener("mouseup", () => {
      viewport.isDragging = false;
      canvas.style.cursor = "grab";
    });
    
    canvas.addEventListener("mouseleave", () => {
      viewport.isDragging = false;
      canvas.style.cursor = "grab";
    });
    
    // Touch drag
    canvas.addEventListener("touchstart", (e) => {
      if (e.touches.length === 1) {
        e.preventDefault();
        viewport.isDragging = true;
        viewport.followRobot = false;
        const touch = e.touches[0];
        viewport.startX = touch.clientX - viewport.offsetX;
        viewport.startY = touch.clientY - viewport.offsetY;
        updateFollowButtons();
      }
    });
    
    canvas.addEventListener("touchmove", (e) => {
      if (!viewport.isDragging || e.touches.length !== 1) return;
      e.preventDefault();
      const touch = e.touches[0];
      viewport.offsetX = touch.clientX - viewport.startX;
      viewport.offsetY = touch.clientY - viewport.startY;
      viewport.needsRedraw = true;
      if (!animationFrameId) renderMaps();
    });
    
    canvas.addEventListener("touchend", () => {
      viewport.isDragging = false;
    });
    
    // Wheel zoom
    canvas.addEventListener("wheel", (e) => {
      e.preventDefault();
      viewport.followRobot = false;
      const delta = e.deltaY > 0 ? -0.1 : 0.1;
      viewport.scale = Math.max(0.5, Math.min(10, viewport.scale + delta));
      viewport.needsRedraw = true;
      if (!animationFrameId) renderMaps();
      updateFollowButtons();
    });
  });
  
  // Control buttons - setup page
  const btnFollow = $("btn-follow-robot");
  if (btnFollow) {
    btnFollow.onclick = () => {
      const vp = getViewport("map-setup");
      vp.followRobot = !vp.followRobot;
      vp.needsRedraw = true;
      if (!animationFrameId) renderMaps();
      updateFollowButtons();
    };
  }
  
  const btnZoomIn = $("btn-zoom-in");
  if (btnZoomIn) {
    btnZoomIn.onclick = () => {
      const vp = getViewport("map-setup");
      vp.scale = Math.min(10, vp.scale + 0.3);
      vp.needsRedraw = true;
      if (!animationFrameId) renderMaps();
    };
  }
  
  const btnZoomOut = $("btn-zoom-out");
  if (btnZoomOut) {
    btnZoomOut.onclick = () => {
      const vp = getViewport("map-setup");
      vp.scale = Math.max(0.5, vp.scale - 0.3);
      vp.needsRedraw = true;
      if (!animationFrameId) renderMaps();
    };
  }
  
  // Control buttons - dashboard page
  const btnFollowDash = $("btn-follow-robot-dash");
  if (btnFollowDash) {
    btnFollowDash.onclick = () => {
      const vp = getViewport("map-dashboard");
      vp.followRobot = !vp.followRobot;
      vp.needsRedraw = true;
      if (!animationFrameId) renderMaps();
      updateFollowButtons();
    };
  }
  
  const btnZoomInDash = $("btn-zoom-in-dash");
  if (btnZoomInDash) {
    btnZoomInDash.onclick = () => {
      const vp = getViewport("map-dashboard");
      vp.scale = Math.min(10, vp.scale + 0.3);
      vp.needsRedraw = true;
      if (!animationFrameId) renderMaps();
    };
  }
  
  const btnZoomOutDash = $("btn-zoom-out-dash");
  if (btnZoomOutDash) {
    btnZoomOutDash.onclick = () => {
      const vp = getViewport("map-dashboard");
      vp.scale = Math.max(0.5, vp.scale - 0.3);
      vp.needsRedraw = true;
      if (!animationFrameId) renderMaps();
    };
  }
}

function updateFollowButtons() {
  const btnFollow = $("btn-follow-robot");
  const btnFollowDash = $("btn-follow-robot-dash");
  
  if (btnFollow) {
    const vp = getViewport("map-setup");
    btnFollow.classList.toggle("active", vp.followRobot);
  }
  
  if (btnFollowDash) {
    const vp = getViewport("map-dashboard");
    btnFollowDash.classList.toggle("active", vp.followRobot);
  }
}

// Sidebar toggle
function setupSidebarToggle() {
  const burger = $("burgerToggle");
  const sidebar = document.querySelector(".sidebar");
  const scrim = $("sidebarScrim");

  if (!burger || !sidebar || !scrim) return;

  function openSidebar() {
    sidebar.classList.add("open");
    scrim.classList.add("open");
    document.body.style.overflow = "hidden";
  }

  function closeSidebar() {
    sidebar.classList.remove("open");
    scrim.classList.remove("open");
    document.body.style.overflow = "";
  }

  burger.onclick = () => {
    if (sidebar.classList.contains("open")) {
      closeSidebar();
    } else {
      openSidebar();
    }
  };

  scrim.onclick = closeSidebar;

  // Close sidebar when clicking nav links on mobile
  document.querySelectorAll(".nav a").forEach(link => {
    link.addEventListener("click", () => {
      if (window.innerWidth <= 980) {
        closeSidebar();
      }
    });
  });
}

// UI hooks
window.addEventListener("load", () => {
  connectROS();
  setupSidebarToggle();
  setupMapControls();
  
  // Setup joystick and rotation controls
  setupJoystick();
  setupRotationControls();

  // Emergency stop button
  const btnEmergencyStop = $("btn-emergency-stop");
  if (btnEmergencyStop) {
    btnEmergencyStop.onclick = toggleEmergencyStop;
  }

  const mMap = $("btn-mode-mapping");
  const mNav = $("btn-mode-nav");
  if (mMap) mMap.onclick = () => setMode("mapping");
  if (mNav) mNav.onclick = () => listMaps();

  const btnSaveMap = $("btn-save-map");
  if (btnSaveMap) btnSaveMap.onclick = () => {
    saveMap($("mapName")?.value?.trim());
  };

  const btnSaveSpot = $("btn-save-parking-spot");
  if (btnSaveSpot) btnSaveSpot.onclick = () => {
    const label = $("parkingSpotLabel")?.value?.trim();
    console.log("Saving parking spot:", label);
    if (!label) return setAlert("warn", "Enter a label");
    saveParking(label);
  };
});