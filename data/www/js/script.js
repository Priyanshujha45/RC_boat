let socket;

document.addEventListener("DOMContentLoaded", function () {
  // Login form handling
  const loginForm = document.getElementById("loginForm");
  if (loginForm) {
    loginForm.addEventListener("submit", function (e) {
      e.preventDefault();
      const password = document.getElementById("password").value;

      fetch("/api/login", {
        method: "POST",
        headers: {
          "Content-Type": "application/x-www-form-urlencoded",
        },
        body: "password=" + encodeURIComponent(password),
      })
        .then((response) => response.json())
        .then((data) => {
          if (data.success) {
            window.location.href = "/control.html";
          } else {
            document.getElementById("loginError").textContent =
              data.error || "Login failed";
          }
        });
    });
  }

  // Logout button
  const logoutBtn = document.getElementById("logoutBtn");
  if (logoutBtn) {
    logoutBtn.addEventListener("click", function () {
      fetch("/api/logout").then(() => (window.location.href = "/login.html"));
    });
  }

  // Speed control
  const speedSlider = document.getElementById("speedSlider");
  if (speedSlider) {
    speedSlider.addEventListener("input", function () {
      document.getElementById("speedValue").textContent = this.value;
      sendCommand("SPEED:" + this.value);
    });
  }

  // Initialize WebSocket
  initWebSocket();
});

function initWebSocket() {
  socket = new WebSocket("ws://" + window.location.hostname + ":81/");

  socket.onmessage = function (event) {
    const data = JSON.parse(event.data);
    if (data.type === "status") {
      updateStatusDisplay(data);
    }
  };

  socket.onclose = function () {
    setTimeout(initWebSocket, 2000);
  };
}

function updateStatusDisplay(data) {
  // Obstacle alert
  const alertDiv = document.getElementById('obstacleAlert');
  if (data.obstacle) {
    alertDiv.classList.remove('hidden');
    // Force stop command (redundant safety)
    sendCommand("STOP");
  } else {
    alertDiv.classList.add('hidden');
  }
  if (data.gps) {
    document.getElementById("gpsData").textContent = data.gps;
  }
  if (data.distance) {
    document.getElementById("distanceData").textContent = data.distance + " cm";
  }
  if (data.temperature !== undefined) {
    document.getElementById("temperatureData").textContent =
      data.temperature + " °C";
  }
  if (data.humidity !== undefined) {
    document.getElementById("humidityData").textContent = data.humidity + " %";
  }
}

function sendCommand(cmd) {
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send(cmd);
  }
}

