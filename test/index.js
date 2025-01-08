const sio = io("http://localhost:5000");

window.setInterval(() => sio.emit("camera_image_request", []), 1000);

sio.on("connect", () => {
    console.log("connected");
});

sio.on('connect_error', (e) => {
    console.log(e.message);
});

sio.on('error', (e) => {
    console.log(e.message);
});

sio.on("disconnect", () => {
    console.log("disconnected");
});

sio.on("cabot_version", (data) => {
    document.getElementById("cabot_version").innerText = data;
});

sio.on("device_status", (data) => {
    document.getElementById("device_status").innerText = JSON.stringify(JSON.parse(data), null, 4);
});

sio.on("system_status", (data) => {
    document.getElementById("system_status").innerText = JSON.stringify(JSON.parse(data), null, 4);
});

sio.on("battery_status", (data) => {
    console.log("battery_status", data);
});

sio.on("touch", (data) => {
    document.getElementById("touch").innerText = JSON.stringify(JSON.parse(data), null, 4);
});

sio.on("speak", (data) => {
    console.log("speak", data);
});

sio.on("navigate", (data) => {
    console.log("navigate", data);
});

sio.on("log_response", (data) => {
    console.log("log_response", data);
});

sio.on("share", (data) => {
    console.log("share", data);
});

sio.on("camera_image", (data) => {
    document.getElementById("camera_image").src = data;
});

sio.on("location", (data) => {
    document.getElementById("location").innerText = JSON.stringify(JSON.parse(data), null, 4);
});
