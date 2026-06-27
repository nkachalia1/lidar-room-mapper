const canvas = document.getElementById("mapCanvas");
const ctx = canvas.getContext("2d");
const scans = document.getElementById("scans");
const occupied = document.getElementById("occupied");
const free = document.getElementById("free");
const uptime = document.getElementById("uptime");
const source = document.getElementById("source");
const points = document.getElementById("points");
const error = document.getElementById("error");
const cameraImage = document.getElementById("cameraImage");
const cameraCaption = document.getElementById("cameraCaption");
const resetButton = document.getElementById("resetButton");
const pauseButton = document.getElementById("pauseButton");

let paused = false;

resetButton.addEventListener("click", async () => {
  await fetch("/api/reset");
});

pauseButton.addEventListener("click", () => {
  paused = !paused;
  pauseButton.textContent = paused ? "Resume" : "Pause";
});

async function tick() {
  if (!paused) {
    try {
      const response = await fetch("/api/state");
      const state = await response.json();
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
  if (!camera.path) {
    cameraCaption.textContent = "Camera disabled";
    return;
  }
  cameraImage.src = `/api/latest.jpg?t=${Date.now()}`;
  const captured = camera.timestamp ? new Date(camera.timestamp * 1000).toLocaleTimeString() : "";
  cameraCaption.textContent = captured ? `Latest frame ${captured}` : "Latest frame";
}

tick();
