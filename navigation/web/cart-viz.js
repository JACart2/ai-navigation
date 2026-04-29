const VEHICLE = {
  width: 1.1938,
  length: 2.4003,
  wheelBase: 2.4003,
  frontAxleTrack: 0.9017,
  rearAxleTrack: 0.9652,
  tireWidth: 0.2159,
  safeObstacleDist: 6 * 1.25,
  safeObstacleTime: 2 * 1.25,
};

const params = new URLSearchParams(window.location.search);
const ROSBRIDGE_HOST = params.get("rosbridgeHost") || window.location.hostname || "127.0.0.1";
const ROSBRIDGE_PORT = params.get("rosbridgePort") || "9090";

const canvas = document.getElementById("scene");
const ctx = canvas.getContext("2d");

const ui = {
  connection: document.getElementById("connection-status"),
  frame: document.getElementById("frame-status"),
  mode: document.getElementById("mode-status"),
  speed: document.getElementById("speed-value"),
  speedSecondary: document.getElementById("speed-value-secondary"),
  targetSpeed: document.getElementById("target-speed-value"),
  targetSpeedSecondary: document.getElementById("target-speed-secondary"),
  steering: document.getElementById("steering-value"),
  obstacleCount: document.getElementById("obstacle-count"),
  obstacleCountSub: document.getElementById("obstacle-count-sub"),
  criticalCount: document.getElementById("critical-count"),
  followableCount: document.getElementById("followable-count"),
  pathCount: document.getElementById("path-count"),
  targetStatus: document.getElementById("target-status"),
  stopSummary: document.getElementById("stop-summary"),
};

const state = {
  connection: "connecting",
  obstacleFrame: "base_link",
  steerDeg: 0,
  targetSpeedMps: 0,
  currentSpeedMps: 0,
  pose: null,
  yaw: 0,
  pathPoints: [],
  targetPoint: null,
  obstacles: [],
  projectedLines: new Map(),
  stopStatus: "No active stop request.",
  vehicleState: {
    isNavigating: false,
    reachedDestination: false,
    stopped: false,
  },
};

let socket = null;

function formatSpeed(mps) {
  return `${(mps * 3.6).toFixed(1)} km/h`;
}

function formatMps(mps) {
  return `${mps.toFixed(1)} m/s`;
}

function yawFromQuaternion(quat) {
  const siny = 2 * ((quat.w * quat.z) + (quat.x * quat.y));
  const cosy = 1 - 2 * ((quat.y * quat.y) + (quat.z * quat.z));
  return Math.atan2(siny, cosy);
}

function mapToLocal(point) {
  if (!state.pose) {
    return null;
  }

  const dx = point.x - state.pose.x;
  const dy = point.y - state.pose.y;
  const cosYaw = Math.cos(state.yaw);
  const sinYaw = Math.sin(state.yaw);

  return {
    x: (cosYaw * dx) + (sinYaw * dy),
    y: (-sinYaw * dx) + (cosYaw * dy),
  };
}

function connectRosbridge() {
  const url = `ws://${ROSBRIDGE_HOST}:${ROSBRIDGE_PORT}`;
  socket = new WebSocket(url);

  socket.addEventListener("open", () => {
    state.connection = "live";
    subscribeTopics();
    updateHud([]);
  });

  socket.addEventListener("close", () => {
    state.connection = "reconnecting";
    updateHud([]);
    setTimeout(connectRosbridge, 1500);
  });

  socket.addEventListener("error", () => {
    state.connection = "offline";
    updateHud([]);
  });

  socket.addEventListener("message", (event) => {
    const payload = JSON.parse(event.data);
    if (payload.op !== "publish") {
      return;
    }

    switch (payload.topic) {
      case "/obstacles":
        state.obstacleFrame = payload.msg.header?.frame_id || "unknown";
        state.obstacles = (payload.msg.obstacles || []).map((obstacle, index) => ({
          id: index,
          x: obstacle.pos?.point?.x ?? 0,
          y: obstacle.pos?.point?.y ?? 0,
          radius: obstacle.radius ?? 0.35,
          followable: Boolean(obstacle.followable),
        }));
        break;
      case "/nav_cmd":
        state.targetSpeedMps = payload.msg.vel ?? 0;
        state.steerDeg = payload.msg.angle ?? 0;
        break;
      case "/estimate_twist":
        state.currentSpeedMps = payload.msg.twist?.linear?.x ?? 0;
        break;
      case "/pcl_pose": {
        const pose = payload.msg.pose?.pose;
        if (!pose) {
          break;
        }

        state.pose = {
          x: pose.position?.x ?? 0,
          y: pose.position?.y ?? 0,
        };
        state.yaw = yawFromQuaternion(pose.orientation || { x: 0, y: 0, z: 0, w: 1 });
        break;
      }
      case "/path":
        state.pathPoints = (payload.msg.poses || []).map((entry) => ({
          x: entry.pose?.position?.x ?? 0,
          y: entry.pose?.position?.y ?? 0,
        }));
        break;
      case "/target_point":
        state.targetPoint = {
          x: payload.msg.pose?.position?.x ?? 0,
          y: payload.msg.pose?.position?.y ?? 0,
        };
        break;
      case "/projected_path":
        state.projectedLines.set(payload.msg.id ?? 0, payload.msg);
        break;
      case "/stop": {
        const sender = payload.msg.sender_id?.data || "unknown";
        const distance = payload.msg.distance;
        state.stopStatus = payload.msg.stop
          ? `${sender} requested stop${distance > 0 ? ` at ${distance.toFixed(1)} m` : ""}.`
          : `${sender} cleared stop request.`;
        break;
      }
      case "/vehicle_state":
        state.vehicleState = {
          isNavigating: Boolean(payload.msg.is_navigating),
          reachedDestination: Boolean(payload.msg.reached_destination),
          stopped: Boolean(payload.msg.stopped),
        };
        break;
      default:
        break;
    }
  });
}

function subscribeTopics() {
  [
    "/obstacles",
    "/nav_cmd",
    "/estimate_twist",
    "/pcl_pose",
    "/path",
    "/target_point",
    "/projected_path",
    "/stop",
    "/vehicle_state",
  ].forEach((topic) => {
    socket.send(JSON.stringify({ op: "subscribe", topic }));
  });
}

function resizeCanvas() {
  const dpr = window.devicePixelRatio || 1;
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  canvas.width = Math.round(width * dpr);
  canvas.height = Math.round(height * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

function getViewportMetrics() {
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  return {
    width,
    height,
    originX: width * 0.5,
    originY: height * 0.78,
    pixelsPerMeter: Math.min(width * 0.04, height * 0.06),
  };
}

function toScreen(localX, localY, metrics) {
  return {
    x: metrics.originX - (localY * metrics.pixelsPerMeter),
    y: metrics.originY - (localX * metrics.pixelsPerMeter),
  };
}

function computeSweepGeometry(steerDeg) {
  const frontAxleCenter = { x: VEHICLE.wheelBase / 2, y: 0 };
  if (Math.abs(steerDeg) < 1.0) {
    return {
      mode: "straight",
      frontAxleCenter,
      previewLength: 12,
      halfWidth: (VEHICLE.width / 2) + 0.35,
    };
  }

  const steeringAngle = (steerDeg * Math.PI) / 180;
  const innerRadius = VEHICLE.wheelBase / Math.tan(steeringAngle);
  const rearLeftCorner = {
    x: -(VEHICLE.wheelBase / 2),
    y: (VEHICLE.frontAxleTrack / 2) + (VEHICLE.tireWidth / 2),
  };
  const rearRightCorner = {
    x: -(VEHICLE.wheelBase / 2),
    y: -((VEHICLE.frontAxleTrack / 2) + (VEHICLE.tireWidth / 2)),
  };

  if (steerDeg > 0) {
    return {
      mode: "arc",
      rightTurn: false,
      frontAxleCenter,
      innerRadius,
      outerRadius: innerRadius + VEHICLE.width,
      circleCenter: {
        x: rearLeftCorner.x,
        y: rearLeftCorner.y + innerRadius,
      },
    };
  }

  return {
    mode: "arc",
    rightTurn: true,
    frontAxleCenter,
    innerRadius,
    outerRadius: innerRadius - VEHICLE.width,
    circleCenter: {
      x: rearRightCorner.x,
      y: rearRightCorner.y + innerRadius,
    },
  };
}

function pointOnArc(circleCenter, radius, angle, rightTurn) {
  if (!rightTurn) {
    return {
      x: circleCenter.x + (radius * Math.cos(angle)),
      y: circleCenter.y + (radius * Math.sin(angle)),
    };
  }

  return {
    x: circleCenter.x - (radius * Math.cos(angle)),
    y: circleCenter.y - (radius * Math.sin(angle)),
  };
}

function sampleSweepArea(geometry) {
  if (geometry.mode === "straight") {
    return [
      { x: geometry.frontAxleCenter.x, y: -geometry.halfWidth },
      { x: geometry.frontAxleCenter.x + geometry.previewLength, y: -geometry.halfWidth },
      { x: geometry.frontAxleCenter.x + geometry.previewLength, y: geometry.halfWidth },
      { x: geometry.frontAxleCenter.x, y: geometry.halfWidth },
    ];
  }

  const previewLength = 9;
  const radiusAbs = Math.abs(geometry.innerRadius);
  const sweepAngle = previewLength / Math.max(radiusAbs, 1e-6);
  const startAngle = geometry.rightTurn ? ((Math.PI / 2) - sweepAngle) : (-(Math.PI / 2));
  const endAngle = geometry.rightTurn ? (Math.PI / 2) : ((-(Math.PI / 2)) + sweepAngle);
  const points = [];
  const steps = 42;

  for (let i = 0; i <= steps; i += 1) {
    const angle = startAngle + (((endAngle - startAngle) * i) / steps);
    points.push(pointOnArc(geometry.circleCenter, geometry.outerRadius, angle, geometry.rightTurn));
  }
  for (let i = steps; i >= 0; i -= 1) {
    const angle = startAngle + (((endAngle - startAngle) * i) / steps);
    points.push(pointOnArc(geometry.circleCenter, geometry.innerRadius, angle, geometry.rightTurn));
  }

  return points;
}

function classifyObstacle(obstacle, geometry) {
  const currentSpeed = Math.max(state.currentSpeedMps, 0.1);
  const frontAxleCenter = geometry.frontAxleCenter;
  const dx = obstacle.x - frontAxleCenter.x;
  const dy = obstacle.y - frontAxleCenter.y;
  const distance = Math.hypot(dx, dy);
  const impactTime = distance / currentSpeed;

  let inSweep = false;

  if (geometry.mode === "straight") {
    const withinX =
      obstacle.x + obstacle.radius >= frontAxleCenter.x &&
      obstacle.x - obstacle.radius <= frontAxleCenter.x + geometry.previewLength;
    const withinY = Math.abs(obstacle.y) - obstacle.radius <= geometry.halfWidth;
    inSweep = withinX && withinY;
  } else {
    const obstacleSize = 2 * obstacle.radius;
    const circleDist = Math.hypot(
      obstacle.x - geometry.circleCenter.x,
      obstacle.y - geometry.circleCenter.y,
    );

    inSweep =
      (Math.abs(geometry.innerRadius) - obstacleSize) < circleDist &&
      circleDist < (Math.abs(geometry.outerRadius) + obstacleSize);
  }

  return {
    ...obstacle,
    distance,
    impactTime,
    inSweep,
    critical:
      inSweep &&
      (
        distance < (VEHICLE.safeObstacleDist / 3) ||
        impactTime < (VEHICLE.safeObstacleTime / 3)
      ),
  };
}

function drawBackground(metrics, t) {
  ctx.clearRect(0, 0, metrics.width, metrics.height);

  const bgGradient = ctx.createLinearGradient(0, 0, 0, metrics.height);
  bgGradient.addColorStop(0, "#0b1a28");
  bgGradient.addColorStop(1, "#02060a");
  ctx.fillStyle = bgGradient;
  ctx.fillRect(0, 0, metrics.width, metrics.height);

  const glow = ctx.createRadialGradient(
    metrics.originX,
    metrics.originY - (metrics.height * 0.2),
    0,
    metrics.originX,
    metrics.originY - (metrics.height * 0.2),
    metrics.width * 0.42,
  );
  glow.addColorStop(0, "rgba(73, 144, 181, 0.18)");
  glow.addColorStop(1, "rgba(73, 144, 181, 0)");
  ctx.fillStyle = glow;
  ctx.fillRect(0, 0, metrics.width, metrics.height);

  ctx.save();
  ctx.translate(metrics.originX, metrics.originY);

  for (let i = -10; i <= 10; i += 1) {
    const alpha = i === 0 ? 0.35 : 0.12;
    ctx.strokeStyle = `rgba(90, 154, 193, ${alpha})`;
    ctx.lineWidth = i === 0 ? 1.8 : 1;
    ctx.beginPath();
    ctx.moveTo(i * metrics.pixelsPerMeter, -metrics.height);
    ctx.lineTo(i * metrics.pixelsPerMeter, metrics.height);
    ctx.stroke();
  }

  for (let i = 0; i < 12; i += 1) {
    const y = -(i * 2 * metrics.pixelsPerMeter);
    const alpha = Math.max(0.04, 0.28 - (i * 0.018));
    ctx.strokeStyle = `rgba(255, 255, 255, ${alpha})`;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(-metrics.width, y);
    ctx.lineTo(metrics.width, y);
    ctx.stroke();
  }

  ctx.strokeStyle = `rgba(255, 194, 71, ${0.1 + (0.05 * Math.sin(t / 750))})`;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(-metrics.width, -(12 * metrics.pixelsPerMeter));
  ctx.lineTo(metrics.width, -(12 * metrics.pixelsPerMeter));
  ctx.stroke();

  ctx.restore();
}

function drawPath(metrics) {
  if (!state.pose || state.pathPoints.length < 2) {
    return;
  }

  const localPoints = state.pathPoints
    .map(mapToLocal)
    .filter((point) => point && point.x > -4 && point.x < 24 && Math.abs(point.y) < 14);

  if (localPoints.length < 2) {
    return;
  }

  ctx.save();
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  ctx.shadowBlur = 20;
  ctx.shadowColor = "rgba(89, 216, 255, 0.65)";
  ctx.strokeStyle = "rgba(89, 216, 255, 0.92)";
  ctx.lineWidth = 4;
  ctx.beginPath();

  localPoints.forEach((point, index) => {
    const screen = toScreen(point.x, point.y, metrics);
    if (index === 0) {
      ctx.moveTo(screen.x, screen.y);
    } else {
      ctx.lineTo(screen.x, screen.y);
    }
  });

  ctx.stroke();
  ctx.restore();
}

function drawProjectedLines(metrics) {
  state.projectedLines.forEach((marker) => {
    if (!marker.points?.length) {
      return;
    }

    const baseX = marker.pose?.position?.x ?? 0;
    const baseY = marker.pose?.position?.y ?? 0;
    ctx.save();
    ctx.strokeStyle = "rgba(77, 228, 184, 0.7)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    marker.points.forEach((point, index) => {
      const local = mapToLocal({ x: baseX + point.x, y: baseY + point.y });
      if (!local) {
        return;
      }
      const screen = toScreen(local.x, local.y, metrics);
      if (index === 0) {
        ctx.moveTo(screen.x, screen.y);
      } else {
        ctx.lineTo(screen.x, screen.y);
      }
    });
    ctx.stroke();
    ctx.restore();
  });
}

function drawSweepZone(metrics, geometry, t) {
  const area = sampleSweepArea(geometry);
  if (!area.length) {
    return;
  }

  ctx.save();
  ctx.beginPath();
  area.forEach((point, index) => {
    const screen = toScreen(point.x, point.y, metrics);
    if (index === 0) {
      ctx.moveTo(screen.x, screen.y);
    } else {
      ctx.lineTo(screen.x, screen.y);
    }
  });
  ctx.closePath();

  const fill = ctx.createLinearGradient(0, metrics.originY, 0, metrics.originY - (10 * metrics.pixelsPerMeter));
  fill.addColorStop(0, "rgba(255, 194, 71, 0.08)");
  fill.addColorStop(1, "rgba(255, 194, 71, 0.22)");
  ctx.fillStyle = fill;
  ctx.fill();

  ctx.strokeStyle = `rgba(255, 194, 71, ${0.55 + (0.12 * Math.sin(t / 500))})`;
  ctx.lineWidth = 2.5;
  ctx.shadowBlur = 24;
  ctx.shadowColor = "rgba(255, 194, 71, 0.45)";
  ctx.stroke();
  ctx.restore();
}

function drawCart(metrics, t) {
  const bodyCenter = toScreen(0, 0, metrics);
  const widthPx = VEHICLE.width * metrics.pixelsPerMeter;
  const lengthPx = VEHICLE.length * metrics.pixelsPerMeter;

  ctx.save();
  ctx.translate(bodyCenter.x, bodyCenter.y);

  ctx.shadowBlur = 30;
  ctx.shadowColor = "rgba(0, 0, 0, 0.35)";
  const bodyGradient = ctx.createLinearGradient(0, lengthPx / 2, 0, -lengthPx / 2);
  bodyGradient.addColorStop(0, "rgba(238, 244, 247, 0.92)");
  bodyGradient.addColorStop(1, "rgba(146, 171, 189, 0.95)");
  ctx.fillStyle = bodyGradient;
  ctx.strokeStyle = "rgba(255, 255, 255, 0.8)";
  ctx.lineWidth = 2;

  ctx.beginPath();
  ctx.roundRect(-widthPx / 2, -lengthPx / 2, widthPx, lengthPx, 20);
  ctx.fill();
  ctx.stroke();

  ctx.fillStyle = "rgba(16, 28, 40, 0.85)";
  ctx.beginPath();
  ctx.roundRect(-(widthPx * 0.28), -(lengthPx * 0.3), widthPx * 0.56, lengthPx * 0.32, 14);
  ctx.fill();

  drawWheel(metrics, -VEHICLE.rearAxleTrack / 2, -(VEHICLE.wheelBase / 2));
  drawWheel(metrics, VEHICLE.rearAxleTrack / 2, -(VEHICLE.wheelBase / 2));
  drawWheel(metrics, -VEHICLE.frontAxleTrack / 2, VEHICLE.wheelBase / 2);
  drawWheel(metrics, VEHICLE.frontAxleTrack / 2, VEHICLE.wheelBase / 2);

  ctx.fillStyle = `rgba(255, 194, 71, ${0.35 + (0.15 * Math.sin(t / 650))})`;
  ctx.beginPath();
  ctx.arc(0, -(lengthPx * 0.35), 7, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
}

function drawWheel(metrics, lateralOffset, longitudinalOffset) {
  const center = toScreen(longitudinalOffset, lateralOffset, metrics);
  const widthPx = VEHICLE.tireWidth * metrics.pixelsPerMeter;
  const heightPx = 0.5 * metrics.pixelsPerMeter;
  ctx.fillStyle = "rgba(19, 26, 31, 0.95)";
  ctx.beginPath();
  ctx.roundRect(center.x - (widthPx / 2), center.y - (heightPx / 2), widthPx, heightPx, 4);
  ctx.fill();
}

function drawTarget(metrics, t) {
  if (!state.targetPoint || !state.pose) {
    return;
  }

  const local = mapToLocal(state.targetPoint);
  if (!local) {
    return;
  }

  const screen = toScreen(local.x, local.y, metrics);
  const pulse = 1 + (0.25 * Math.sin(t / 450));

  ctx.save();
  ctx.strokeStyle = "rgba(255, 194, 71, 0.95)";
  ctx.lineWidth = 3;
  ctx.shadowBlur = 24;
  ctx.shadowColor = "rgba(255, 194, 71, 0.7)";
  ctx.beginPath();
  ctx.arc(screen.x, screen.y, 12 * pulse, 0, Math.PI * 2);
  ctx.stroke();

  ctx.fillStyle = "rgba(255, 194, 71, 0.95)";
  ctx.beginPath();
  ctx.arc(screen.x, screen.y, 4.5, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
}

function drawObstacles(metrics, classified, t) {
  classified.forEach((obstacle) => {
    const screen = toScreen(obstacle.x, obstacle.y, metrics);
    const radius = Math.max(obstacle.radius * metrics.pixelsPerMeter, 8);

    let fill = "rgba(118, 184, 255, 0.82)";
    let glow = "rgba(118, 184, 255, 0.6)";
    if (obstacle.inSweep) {
      fill = "rgba(255, 194, 71, 0.88)";
      glow = "rgba(255, 194, 71, 0.65)";
    }
    if (obstacle.critical) {
      fill = `rgba(255, 95, 87, ${0.78 + (0.18 * Math.sin(t / 220))})`;
      glow = "rgba(255, 95, 87, 0.8)";
    }

    ctx.save();
    ctx.shadowBlur = 30;
    ctx.shadowColor = glow;
    ctx.fillStyle = fill;
    ctx.beginPath();
    ctx.arc(screen.x, screen.y, radius, 0, Math.PI * 2);
    ctx.fill();

    ctx.lineWidth = obstacle.followable ? 2.5 : 1.5;
    ctx.strokeStyle = "rgba(255, 255, 255, 0.9)";
    if (obstacle.followable) {
      ctx.setLineDash([6, 5]);
    }
    ctx.beginPath();
    ctx.arc(screen.x, screen.y, radius + 4, 0, Math.PI * 2);
    ctx.stroke();
    ctx.restore();

    ctx.save();
    ctx.font = '12px "SFMono-Regular", Menlo, monospace';
    ctx.fillStyle = "rgba(245, 244, 236, 0.92)";
    ctx.fillText(`${obstacle.distance.toFixed(1)}m`, screen.x + radius + 8, screen.y - 6);
    ctx.restore();
  });
}

function updateHud(classified) {
  const inSweep = classified.filter((obstacle) => obstacle.inSweep).length;
  const critical = classified.filter((obstacle) => obstacle.critical).length;
  const followable = classified.filter((obstacle) => obstacle.followable).length;

  ui.connection.textContent = state.connection;
  ui.frame.textContent = state.obstacleFrame || "unknown";
  ui.mode.textContent = state.vehicleState.reachedDestination
    ? "Arrived"
    : state.vehicleState.stopped
      ? "Stopped"
      : state.vehicleState.isNavigating
        ? "Navigating"
        : "Idle";

  ui.speed.textContent = formatSpeed(Math.max(state.currentSpeedMps, 0));
  ui.speedSecondary.textContent = formatMps(Math.max(state.currentSpeedMps, 0));
  ui.targetSpeed.textContent = formatSpeed(Math.max(state.targetSpeedMps, 0));
  ui.targetSpeedSecondary.textContent = formatMps(Math.max(state.targetSpeedMps, 0));
  ui.steering.textContent = `${state.steerDeg.toFixed(1)}°`;
  ui.obstacleCount.textContent = String(state.obstacles.length);
  ui.obstacleCountSub.textContent = `${inSweep} in sweep zone`;
  ui.criticalCount.textContent = String(critical);
  ui.followableCount.textContent = String(followable);
  ui.pathCount.textContent = String(state.pathPoints.length);
  ui.targetStatus.textContent = state.targetPoint ? "Locked" : "Unavailable";
  ui.stopSummary.textContent = state.stopStatus;
}

function drawScene(t) {
  const metrics = getViewportMetrics();
  drawBackground(metrics, t);
  drawPath(metrics);
  drawProjectedLines(metrics);

  const geometry = computeSweepGeometry(state.steerDeg);
  const classified = state.obstacles.map((obstacle) => classifyObstacle(obstacle, geometry));

  drawSweepZone(metrics, geometry, t);
  drawTarget(metrics, t);
  drawObstacles(metrics, classified, t);
  drawCart(metrics, t);
  updateHud(classified);
  requestAnimationFrame(drawScene);
}

window.addEventListener("resize", resizeCanvas);
resizeCanvas();
connectRosbridge();
requestAnimationFrame(drawScene);
