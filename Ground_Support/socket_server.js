const http = require("http");
const fs = require("fs");
const path = require("path");
const { Server } = require("socket.io");

const server = http.createServer((req, res) => {
  if (req.url === "/chart.js") {
    const filePath = path.join(
      __dirname,
      "node_modules/chart.js/dist/chart.umd.min.js"
    );

    res.writeHead(200, {
      "Content-Type": "application/javascript",
      "X-Content-Type-Options": "nosniff",
    });

    fs.createReadStream(filePath).pipe(res);
    console.log("Requested Chart.js");
    return;
  }

  res.writeHead(404);
  res.end();
});

const io = new Server(server, {
  cors: { 
    origin: "*", // wildcard for dev; specify IP in production
    methods: ["GET", "POST"],
    credentials: false,
  },
});

server.listen(3000, "0.0.0.0");
