// app.js

const express = require('express');
const http = require('http');
const path = require('path');
const socketIo = require('socket.io');
const rosnodejs = require('rosnodejs');
const std_srvs = rosnodejs.require('std_srvs').srv;
const my_pkg = rosnodejs.require('vlns');
const my_service = my_pkg.srv.sges;
const app = express();
const server = http.createServer(app);
const io = socketIo(server);

app.use(express.static(path.join(__dirname, 'public')));
app.use(express.json());

const PORT = 3000;
server.listen(PORT, () => {
  console.log(`Server running at http://0.0.0.0:${PORT}/`);
});

let latestSocket = null;

io.on('connection', (socket) => {
  console.log('A client connected:', socket.id);
  //latestSocket = socket;

  socket.on('disconnect', () => {
    console.log('Client disconnected:', socket.id);
    //latestSocket = null;
  });

  // Listen for control commands from the browser
  

socket.on('charge', async (_, cb) => {
  try {
    const available = await rosnodejs.nh.waitForService('/moder', 1000);
    if (!available) {
      cb && cb(false);
      return;
    }
    const req = new my_service.Request();
    req.mode = "charge";
    const resp = await rosServices.mode.call(req);
    cb && cb(resp.success);
  } catch (e) {
    console.error('charge service call failed:', e);
    cb && cb(false);
  }
});

socket.on('fast', async (_, cb) => {
  try {
    const available = await rosnodejs.nh.waitForService('/moder', 1000);
    if (!available) {
      cb && cb(false);
      return;
    }
    const req = new my_service.Request();
    req.mode = "fast";
    const resp = await rosServices.mode.call(req);
    cb && cb(resp.success);
  } catch (e) {
    console.error('Fast Charge service call failed:', e);
    cb && cb(false);
  }
});

socket.on('discharge', async (_, cb) => {


  try {
    const available = await rosnodejs.nh.waitForService('/moder', 1000);
    if (!available) {
      cb && cb(false);
      return;
    }
    const req = new my_service.Request();
    req.mode = "discharge";
    const resp = await rosServices.mode.call(req);
    cb && cb(resp.success);
  } catch (e) {
    console.error('Discharge service call failed:', e);
    cb && cb(false);
  }
});

  
  
});

// Store service clients
const rosServices = {
  forceStop: null,
  charge: null,
  discharge: null,
};

// Initialize ROS node and set up topics/services
rosnodejs.initNode('/web_dashboard_node', { rosMasterUri: 'http://raspberrypi:11311' })
  .then(() => {
    const nh = rosnodejs.nh;

    // Subscribers
/*   nh.subscribe('/energy_storage/volt', 'std_msgs/Float32', (msg) => {
  io.emit('voltage', msg.data);
});*/
nh.subscribe('/energy_storage/encoder', 'std_msgs/Float32', (msg) => {
  io.emit('encoder', msg.data);
});
nh.subscribe('/mode', 'std_msgs/String', (msg) => {
  io.emit('mode', msg.data);
});
nh.subscribe('/cars_count', 'std_msgs/Int32', (msg) => {
  io.emit('cars_count', msg.data);
});
nh.subscribe('/energy', 'std_msgs/Float32', (msg) => {
  io.emit('energy', msg.data);
});
nh.subscribe('/height', 'std_msgs/Float32', (msg) => {
  io.emit('height', msg.data);
});
/*
nh.subscribe('/energy_storage/status', 'std_msgs/String', (msg) => {
  io.emit('status', msg.data);
});*/
nh.subscribe('/car_passed', 'std_msgs/Bool', (msg) => {
  io.emit('car_passed', msg.data);
});

    // Service Clients
    rosServices.mode = nh.serviceClient('/moder', my_service);

    console.log('ROS node initialized and subscribers/services set up.');
  })
  .catch((err) => {
    console.error('Failed to initialize ROS node:', err);
  });
