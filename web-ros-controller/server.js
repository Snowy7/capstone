const express = require("express");
const path = require("path");
const os = require("os");

const app = express();
const PORT = 80;
const ROSBRIDGE_PORT = 9090;

function getLocalIP() {
  const ifs = os.networkInterfaces();
  for (const name of Object.keys(ifs)) {
    for (const net of ifs[name]) {
      if (net.family === "IPv4" && !net.internal) return net.address;
    }
  }
  return "127.0.0.1";
}

app.use(express.static(path.join(__dirname, "public"), {
  setHeaders: (res) => {
    res.setHeader("Cache-Control", "no-cache");
  },
}));

app.get("/config.js", (req, res) => {
  const ip = getLocalIP();
  res.type("application/javascript");
  res.send(`
    window.ROS_CONFIG = {
      url: "ws://${ip}:${ROSBRIDGE_PORT}"
    };
  `);
});

app.listen(PORT, "0.0.0.0", () => {
  const ip = getLocalIP();
  console.log("===============================================");
  console.log(" AutoCharge Web Controller");
  console.log("-----------------------------------------------");
  console.log(` Local:   http://localhost:${PORT}`);
  console.log(` Network: http://${ip}:${PORT}`);
  console.log(` Bridge:  ws://${ip}:${ROSBRIDGE_PORT}`);
  console.log("===============================================");
});