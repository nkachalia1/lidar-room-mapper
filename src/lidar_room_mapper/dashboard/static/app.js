const canvas = document.getElementById("mapCanvas");
const ctx = canvas.getContext("2d");
const scans = document.getElementById("scans");
const occupied = document.getElementById("occupied");
const free = document.getElementById("free");
const uptime = document.getElementById("uptime");
const source = document.getElementById("source");
const points = document.getElementById("points");
const error = document.getElementById("error");
const cameraViewport = document.getElementById("cameraViewport");
const cameraImage = document.getElementById("cameraImage");
const cameraOverlay = document.getElementById("cameraOverlay");
const cameraOverlayContext = cameraOverlay.getContext("2d");
const cameraCaption = document.getElementById("cameraCaption");
const overlayToggle = document.getElementById("overlayToggle");
const resetButton = document.getElementById("resetButton");
const pauseButton = document.getElementById("pauseButton");

let paused = false;
let lastCameraTimestamp = null;
let latestState = null;

resetButton.addEventListener("click", async () => {
  await fetch("/api/reset");
});

pauseButton.addEventListener("click", () => {
  paused = !paused;
  pauseButton.textContent = paused ? "Resume" : "Pause";
});

overlayToggle.addEventListener("change", () => {
  if (latestState) {
    updateCamera(latestState);
  }
});

async function tick() {
  if (!paused) {
    try {
      const response = await fetch("/api/state");
      const state = await response.json();
      latestState = state;
      updateMetrics(state);
      drawMap(state);
      updateCamera(state);
    } catch (err) {
      error.textContent = String(err);
    }
  }
  window.setTimeout(tick, 500);
}

function updateMetrics(state) {
  const stats = state.stats || {};
  const latest = state.latest_scan || {};
  const runtime = state.runtime || {};
  scans.textContent = stats.scans_integrated || 0;
  occupied.textContent = stats.occupied_cells || 0;
  free.textContent = stats.free_cells || 0;
  uptime.textContent = `${Math.round(runtime.uptime_s || 0)}s`;
  source.textContent = latest.source || "waiting";
  points.textContent = `${latest.points || 0} points`;
  error.textContent = runtime.error || "";
}

function drawMap(state) {
  const grid = state.grid || [];
  const width = state.width || 1;
  const height = state.height || 1;
  const cellW = canvas.width / width;
  const cellH = canvas.height / height;

  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#171a21";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  for (let y = 0; y < height; y += 1) {
    const row = grid[y] || [];
    for (let x = 0; x < width; x += 1) {
      const p = row[x] ?? 50;
      if (p > 58) {
        const strength = Math.min(1, (p - 50) / 50);
        ctx.fillStyle = `rgba(242, 245, 244, ${0.26 + strength * 0.72})`;
      } else if (p < 42) {
        const strength = Math.min(1, (50 - p) / 50);
        ctx.fillStyle = `rgba(69, 196, 160, ${0.08 + strength * 0.34})`;
      } else {
        continue;
      }
      ctx.fillRect(x * cellW, canvas.height - (y + 1) * cellH, cellW + 0.5, cellH + 0.5);
    }
  }

  drawRobot();
}

function drawRobot() {
  const cx = canvas.width / 2;
  const cy = canvas.height / 2;
  ctx.save();
  ctx.translate(cx, cy);
  ctx.fillStyle = "#45c4a0";
  ctx.beginPath();
  ctx.arc(0, 0, 10, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = "#f2b84b";
  ctx.lineWidth = 4;
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(28, 0);
  ctx.stroke();
  ctx.restore();
}

function updateCamera(state) {
  const camera = state.camera || {};
  const fusion = state.fusion || {};
  if (!camera.path) {
    cameraViewport.hidden = true;
    cameraImage.removeAttribute("src");
    clearCameraOverlay();
    lastCameraTimestamp = null;
    overlayToggle.disabled = true;
    cameraCaption.textContent = "Camera disabled";
    return;
  }

  cameraViewport.hidden = false;
  overlayToggle.disabled = !fusion.enabled;
  if (camera.timestamp !== lastCameraTimestamp) {
    cameraImage.src = `/api/latest.jpg?t=${camera.timestamp || Date.now()}`;
    lastCameraTimestamp = camera.timestamp;
  }

  drawCameraOverlay(camera, fusion);
  const captured = camera.timestamp ? new Date(camera.timestamp * 1000).toLocaleTimeString() : "";
  const details = [captured || "Latest frame"];
  if (fusion.enabled) {
    details.push(`${fusion.projected_count || 0} pts`);
    if (Number.isFinite(fusion.sync_delta_ms)) {
      const sign = fusion.sync_delta_ms > 0 ? "+" : "";
      details.push(`sync ${sign}${fusion.sync_delta_ms.toFixed(1)} ms`);
    }
  }
  cameraCaption.textContent = details.join(" | ");
}

function drawCameraOverlay(camera, fusion) {
  const width = camera.width || 1;
  const height = camera.height || 1;
  if (cameraOverlay.width !== width || cameraOverlay.height !== height) {
    cameraOverlay.width = width;
    cameraOverlay.height = height;
  }
  clearCameraOverlay();
  if (!fusion.enabled || !overlayToggle.checked) {
    return;
  }

  const baseRadius = Math.max(5, width / 180);
  for (const point of fusion.points || []) {
    cameraOverlayContext.beginPath();
    cameraOverlayContext.arc(
      point.u,
      point.v,
      point.quality > 20 ? baseRadius : baseRadius * 0.75,
      0,
      Math.PI * 2,
    );
    cameraOverlayContext.fillStyle = lidarPointColor(point.distance_m);
    cameraOverlayContext.fill();
    cameraOverlayContext.strokeStyle = "rgba(4, 8, 15, 0.85)";
    cameraOverlayContext.lineWidth = Math.max(2, width / 960);
    cameraOverlayContext.stroke();
  }
}

function clearCameraOverlay() {
  cameraOverlayContext.clearRect(0, 0, cameraOverlay.width, cameraOverlay.height);
}

function lidarPointColor(distanceM) {
  if (distanceM < 1) return "#fb7185";
  if (distanceM < 2) return "#facc15";
  if (distanceM < 4) return "#38bdf8";
  return "#a78bfa";
}

tick();
