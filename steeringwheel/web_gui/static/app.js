let canvas = document.getElementById("joystick");
let ctx = canvas.getContext("2d");

let center = { x: 125, y: 125 };
let radius = 100;

let dragging = false;
let joyPos = { x: center.x, y: center.y };

let currentSpeed = 0;
let currentSteer = 90;

let speedStep = 0;
let steerStep = 0;

let controlEnabled = false;

// ================= DRAW =================

function drawJoystick() {
    ctx.clearRect(0, 0, 250, 250);

    ctx.beginPath();
    ctx.arc(center.x, center.y, radius, 0, Math.PI * 2);
    ctx.strokeStyle = "white";
    ctx.lineWidth = 3;
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(joyPos.x, joyPos.y, 25, 0, Math.PI * 2);
    ctx.fillStyle = "red";
    ctx.fill();
}

drawJoystick();

// ================= INPUT ZONE LOGIC =================

function computeZones() {
    let dx = joyPos.x - center.x;
    let dy = joyPos.y - center.y;

    let dist = Math.sqrt(dx * dx + dy * dy);

    let percent = dist / radius;

    speedStep = 0;
    steerStep = 0;

    // SPEED control (vertical)
    if (percent > 0.8) {  // gần rìa
        if (dy < -20) speedStep = +10;
        if (dy > 20) speedStep = -10;
    }

    // STEER control (horizontal)
    if (percent > 0.3) {
        if (dx < -20) steerStep = -5;
        if (dx > 20) steerStep = +5;
    }
}

function toggleControl() {
    controlEnabled = !controlEnabled;
    let btn = document.getElementById("controlBtn");

    if (controlEnabled) {
        btn.innerText = "CONTROL ON";
        btn.style.background = "green";
    } else {
        btn.innerText = "CONTROL OFF";
        btn.style.background = "";
    }
}


// ================= UPDATE LOOP (100ms) =================

setInterval(() => {

    if (!controlEnabled) return;

    // ===== SPEED =====
    if (speedStep !== 0) {
        currentSpeed += speedStep;
    } else {
        // auto decay về 0
        if (currentSpeed > 0) currentSpeed -= 10;
        if (currentSpeed < 0) currentSpeed += 10;
    }

    currentSpeed = Math.max(-200, Math.min(200, currentSpeed));

    // ===== STEER =====
    if (steerStep !== 0) {
        currentSteer += steerStep;
    }

    // KHÔNG auto return nữa
    currentSteer = Math.max(45, Math.min(135, currentSteer));

    fetch(`/set_input/${currentSpeed}/${currentSteer}`, {
        method: "POST"
    });

}, 100);


// ================= DRAG EVENTS =================

function getPos(evt) {
    let rect = canvas.getBoundingClientRect();
    return {
        x: evt.touches ? evt.touches[0].clientX - rect.left : evt.clientX - rect.left,
        y: evt.touches ? evt.touches[0].clientY - rect.top : evt.clientY - rect.top
    };
}

function startDrag(evt) {
    evt.preventDefault();

    let pos = getPos(evt);
    let dx = pos.x - center.x;
    let dy = pos.y - center.y;
    let dist = Math.sqrt(dx * dx + dy * dy);

    // Nếu bấm gần tâm → center steer
    if (dist < 30) {
        currentSteer = 90;
        fetch(`/set_input/${currentSpeed}/${currentSteer}`, {
            method: "POST"
        });
        return;
    }

    dragging = true;
}


function stopDrag() {
    dragging = false;
    joyPos = { x: center.x, y: center.y };
    drawJoystick();
    computeZones();
}

function drag(evt) {
    if (!dragging) return;

    evt.preventDefault();

    let pos = getPos(evt);
    let dx = pos.x - center.x;
    let dy = pos.y - center.y;

    let dist = Math.sqrt(dx * dx + dy * dy);

    if (dist > radius) {
        dx = dx / dist * radius;
        dy = dy / dist * radius;
    }

    joyPos.x = center.x + dx;
    joyPos.y = center.y + dy;

    drawJoystick();
    computeZones();
}

// mouse
canvas.addEventListener("mousedown", startDrag);
canvas.addEventListener("mouseup", stopDrag);
canvas.addEventListener("mousemove", drag);

// touch
canvas.addEventListener("touchstart", startDrag);
canvas.addEventListener("touchend", stopDrag);
canvas.addEventListener("touchmove", drag);

// disable context menu
canvas.addEventListener("contextmenu", e => e.preventDefault());

// ================= WebSocket =================

let ws = new WebSocket(`ws://${location.host}/ws`);

ws.onmessage = function(event) {
    let data = JSON.parse(event.data);

    let mode = data.vehicle.mode;

    document.getElementById("manualScreen").style.display =
        mode === "manual" ? "block" : "none";

    document.getElementById("followScreen").style.display =
        mode === "follow" ? "block" : "none";

    document.getElementById("autoScreen").style.display =
        mode === "auto" ? "block" : "none";

    // MANUAL
    document.getElementById("m_speed").innerText = data.vehicle.speed;
    document.getElementById("m_steer").innerText = data.vehicle.steer;

    // FOLLOW
    document.getElementById("f_speed").innerText = data.vehicle.speed;
    document.getElementById("f_steer").innerText = data.vehicle.steer;
    document.getElementById("f_error").innerText = data.follow.error.toFixed(3);
    document.getElementById("f_visible").innerText = data.follow.visible;

    // AUTO
    document.getElementById("a_speed").innerText = data.vehicle.speed;
    document.getElementById("a_steer").innerText = data.vehicle.steer;
    document.getElementById("a_error").innerText = data.lane.error.toFixed(3);
};


function setMode(mode) {
    fetch(`/set_mode/${mode}`, { method: "POST" });
}

function setLevel(level) {
    fetch(`/set_level/${level}`, { method: "POST" });
}

function emergency() {
    fetch("/emergency", { method: "POST" });
}

// expose ra global để onclick dùng được
window.setMode = setMode;
window.setLevel = setLevel;
window.emergency = emergency;


let frontEnabled = false;
let downEnabled = false;
let lidarEnabled = false;
let followEnabled = false;

function toggleFrontCam() {
    frontEnabled = !frontEnabled;

    if (frontEnabled) {
        fetch("/camera/1/start", { method: "POST" });
    } else {
        fetch("/camera/1/stop", { method: "POST" });
    }

    document.querySelectorAll(".follow-controls button")[0].innerText =
        frontEnabled ? "Front Cam OFF" : "Front Cam ON";
}


function toggleDownCam() {
    downEnabled = !downEnabled;

    if (downEnabled) {
        fetch("/camera/0/start", { method: "POST" });
    } else {
        fetch("/camera/0/stop", { method: "POST" });
    }

    document.querySelectorAll(".follow-controls button")[1].innerText =
        downEnabled ? "Down Cam OFF" : "Down Cam ON";
}


function toggleLidar() {
    lidarEnabled = !lidarEnabled;
    document.querySelectorAll(".follow-controls button")[2].innerText =
        lidarEnabled ? "Lidar OFF" : "Lidar ON";
}

function toggleFollow() {
    followEnabled = !followEnabled;

    if (followEnabled) {
        setMode("follow");
        document.querySelectorAll(".follow-controls button")[3].innerText =
            "STOP FOLLOW";
    } else {
        setMode("manual");
        document.querySelectorAll(".follow-controls button")[3].innerText =
            "START FOLLOW";
    }
}

window.toggleFrontCam = toggleFrontCam;
window.toggleDownCam = toggleDownCam;
window.toggleLidar = toggleLidar;
window.toggleFollow = toggleFollow;

let frontCanvas = document.getElementById("frontCanvas");
let fctx = frontCanvas.getContext("2d");
let frontImg = new Image();
frontImg.src = "/camera/1";

function updateFrontCamera() {
    if (frontEnabled) {
        fctx.drawImage(frontImg, 0, 0, 400, 300);
    } else {
        fctx.clearRect(0, 0, 400, 300);
    }
    requestAnimationFrame(updateFrontCamera);
}
updateFrontCamera();

let lidarCanvas = document.getElementById("lidarCanvas");
let lctx = lidarCanvas.getContext("2d");

function drawMockLidar() {
    if (!lidarEnabled) {
        lctx.clearRect(0, 0, 300, 300);
        return;
    }

    lctx.clearRect(0, 0, 300, 300);
    lctx.strokeStyle = "white";
    lctx.beginPath();
    lctx.arc(150, 150, 120, 0, Math.PI * 2);
    lctx.stroke();

    requestAnimationFrame(drawMockLidar);
}
drawMockLidar();

let downCanvas = document.getElementById("downCanvas");
let dctx = downCanvas.getContext("2d");
let downImg = new Image();
downImg.src = "/camera/0";
function updateDownCamera() {
    if (downEnabled) {
        dctx.drawImage(downImg, 0, 0, 400, 300);
    } else {
        dctx.clearRect(0, 0, 400, 300);
    }
    requestAnimationFrame(updateDownCamera);
}
updateDownCamera();
