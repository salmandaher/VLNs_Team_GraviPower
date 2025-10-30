# Solid-Gravity-Energy-Storage-System
Robotic Gravity-based energy storage system designed for smart cities — converts vehicle motion into storable mechanical energy and regenerates it electrically through controlled descent.

# Project Video and Report

Below is the high-quality version of the video we submitted to the WRO competition, showcasing our project in action.  
[WRO Competition Video](https://youtu.be/KDbOAnvwZOE)

You can also download and review the detailed project report we submitted:  
**[Project Report (WRO Submission)](YOUR_REPORT_LINK_HERE)**

---


# Mechanism Design

This section provides an overview of the key mechanical components used in our project. Each mechanism plays a crucial role in ensuring efficient operation and durability. The associated STL files for 3D printing or modeling are included in the repository.

## Pseudo-Bump Mechanism Overview

The pseudo-bump functions by converting the mechanical energy from vehicles passing over it into usable energy for the system. It includes several key components that work in harmony to achieve this. The fixed joint acts as the pivot point for the bump's rotation when a vehicle passes over it. Inside this joint is an anti-reverse bearing (Rollman bearing) that prevents backward motion from affecting the gearbox when the bump resets. Once the vehicle passes, a return spring compresses under pressure and then pushes the bump back to its starting position. The moving joint allows the bump to articulate and flatten as vehicles pass, transferring the force efficiently to the fixed joint and maintaining system reliability.

### Fixed Joint

This section contains two STL parts: the connecting part between the fixed joint and the moving joint, and the bearing holder that houses the bearing linking to the Tetrix system.  
**STL files:**  
- [First Part (Fixed Joint) STL](https://github.com/salmandaher/VLNs_Team_GraviPower/blob/d1e83de02112018f51a4b01dffcafd82a4864781/3d%20designs/Bump/first.stl)  
- [Bearing Holder STL](https://github.com/salmandaher/VLNs_Team_GraviPower/blob/d1e83de02112018f51a4b01dffcafd82a4864781/3d%20designs/stls/inner%20bearing%20to%20tetrix.stl)  

### Moving Joint

This section includes the STL part that forms the component between the moving joint and the free end of the bump, the part pressed directly by vehicles.  
**STL file:**  
- [Moving Joint Linkage STL](https://github.com/salmandaher/VLNs_Team_GraviPower/blob/d1e83de02112018f51a4b01dffcafd82a4864781/3d%20designs/Bump/second.stl)  

### The Base

This section includes the STL part that forms the base, which holds the fixed and moving joints together, providing structural stability.  
**STL file:**  
- [Base Station STL](https://github.com/salmandaher/VLNs_Team_GraviPower/blob/d1e83de02112018f51a4b01dffcafd82a4864781/3d%20designs/Bump/station.stl)  

---

## Motion Transmission and Torque Amplification (The Gearbox)

This phase of the project is critical as it ensures efficient transfer and amplification of the torque necessary to lift the mass. The total torque amplification ratio achieved here is 20:1, an essential factor for the prototype’s performance. The motion transmission is divided into two main steps, each playing a specific role in the mechanism.

### Gearbox 

The first step involves a gearbox with a 2:1 gear ratio, providing speed amplification. This gearbox is specifically designed for our prototype, providing smooth and reliable operation while efficiently increasing output speed. Its compact design and gear ratio make it ideal for the limited scale and precision required by our system.

**[Insert gearbox photo here]**

### Worm Gear 

The second step uses a worm gear, which serves two main purposes. First, it amplifies torque with a 40:1 ratio, making it easier for the car to press the bump and significantly increasing the mechanical advantage. Second, the worm gear acts as a self-locking mechanism, preventing the mass from falling while being lifted. This is because the worm gear cannot be driven backward by the gear connected to it; it only transmits motion forward, thus securing the mass position reliably.

[https://github.com/salmandaher/VLNs_Team_GraviPower/blob/0817ff04903d763af26a9c54c416ef887e680cd4/imgs/Motor%20gearbox%20(2).jpg]

---

## Lifting System

The lifting system is responsible for managing the vertical movement of the mass with precision and stability. It consists of three main components:

### Main Pulley (Spool)

This is the central pulley around which the rope winds. Its radius directly affects the torque experienced in the system, playing a key role in the mechanical advantage and energy efficiency. During the project, we designed and tested three main versions of this pulley to optimize performance and durability.

**STL files:**  
- [Version 1 Pulley STL](https://github.com/salmandaher/VLNs_Team_GraviPower/blob/a61094ec972bfa96dd2c8d812677eebc3b2d72a3/3d%20designs/stls/Pulley%20v1.0-01.stl)  
- [Version 2 Pulley STL](https://github.com/salmandaher/VLNs_Team_GraviPower/blob/a61094ec972bfa96dd2c8d812677eebc3b2d72a3/3d%20designs/stls/Pulley%20v1.0-03.stl)  
- [Final Pulley Design STL](https://github.com/salmandaher/VLNs_Team_GraviPower/blob/a61094ec972bfa96dd2c8d812677eebc3b2d72a3/3d%20designs/stls/Pulley%20v3.2.stl)  <!-- This is the design we currently use -->

### Guiding Pulleys

Two guiding pulleys are installed at the top of the tower. Their main function is to route the rope correctly, ensuring it remains aligned throughout the lifting and lowering operations.

**STL file:**  
- [Guiding Pulley STL](https://github.com/salmandaher/VLNs_Team_GraviPower/blob/a61094ec972bfa96dd2c8d812677eebc3b2d72a3/3d%20designs/stls/direction_pulley.stl)

### Mass Pulley

Attached directly to the mass, this pulley serves multiple purposes: it protects the rope from excessive wear, improves the stability of the mass during motion, and doubles the torque applied when lifting. This crucial component helps regulate the discharge time, improving system reliability.

**[Insert mass pulley image here]**

---

## Connecting and Disconnecting Mechanism

In our project, it is essential to disconnect the main pulley from the gearbox and connect it to the energy generation system, and the opposite, which consists of the motor and its dedicated gearbox. This requirement led to the development of a specialized connecting and disconnecting mechanism.

The core component consists of 3D-printed PLA parts that resemble circular saw-tooth gears designed to transfer energy unidirectionally. When the main pulley is shifted towards the gearbox side, two opposing saw-tooth parts engage, allowing the gearbox to drive the pulley. Conversely, when the pulley is pushed in the opposite direction, the saw-tooth parts disengage on the gearbox side and engage on the motor side gearbox, thus transferring motion accordingly.

To facilitate the movement between these two positions, we utilize an HTS-35H Servo motor with a torque capacity of 35 KG·cm. This servo motor is connected to a rack and pinion mechanism that converts the servo’s rotational input into linear motion, enabling precise and reliable movement of the connecting and disconnecting assembly. The servo is controlled by an ESP32 core board alongside a multifunctional extension board for efficient operation.

### Saw-Tooth Gears

These saw-tooth gears are the key elements that allow unidirectional energy transmission during connection and disconnection. The design includes two complementary halves to ensure smooth engagement and disengagement.

**STL files:**  
- [Saw-Tooth Gear Half 1 STL](https://github.com/salmandaher/VLNs_Team_GraviPower/blob/a61094ec972bfa96dd2c8d812677eebc3b2d72a3/3d%20designs/stls/Coupler%20V2.0-2.stl)  
- [Saw-Tooth Gear Half 2 STL](https://github.com/salmandaher/VLNs_Team_GraviPower/blob/a61094ec972bfa96dd2c8d812677eebc3b2d72a3/3d%20designs/stls/Coupler%20V2.0.stl)

### Rack and Pinion Mechanism

The rack and pinion mechanism translates the servo motor’s rotational motion into linear displacement. The pinion, rotated by the servo, engages with the rack teeth to produce controlled linear movement. This mechanism enables the precise shifting required to connect or disconnect the main pulley from either gearbox side.

**STL files:**  
- [Rack STL](https://github.com/salmandaher/VLNs_Team_GraviPower/blob/a61094ec972bfa96dd2c8d812677eebc3b2d72a3/3d%20designs/stls/39300B_TXM-Linear%20Slide%20Rail.stl)  
- [Pinion STL](https://github.com/salmandaher/VLNs_Team_GraviPower/blob/a61094ec972bfa96dd2c8d812677eebc3b2d72a3/3d%20designs/stls/39300A_TXM-Linear%20Slide%20Gear.stl)

---

## Motor Gearbox
The motor gearbox serves to adjust the motor output speed and torque to desired levels. It employs a gear reduction system that balances speed with mechanical power, optimizing performance for the specific application needs.  
**STL file:** [motor_gearbox.stl](path/to/motor_gearbox.stl)

---

# The coding of the solution
## The main control code (the Python code)

This Python code serves as the central control unit for our system. It manages the system’s operational states, including charging, discharging, and fast charge modes, by coordinating commands to various controllers such as the PRIZM and the ESP. The code continuously monitors sensor data to make decisions that ensure smooth transitions between modes and optimize system performance. Additionally, it implements a PID control algorithm to regulate the descent speed of the mass, maximizing energy generation efficiency by dynamically adjusting the electrical load without wasting energy through conventional braking. The program also supports real-time monitoring and manual mode control via a connected web-based interface, enabling effective management of the entire energy storage process.

### Main Tasks of the Python Control Code

The Python script performs critical functions to optimize the operation of the gravity-based energy storage system:

1. It calculates the generator's rotational speed during discharging using encoder feedback.
2. It applies a PID (Proportional-Integral-Derivative) control algorithm to regulate the generator speed at the point of maximum power generation efficiency. The PID output is fed to the PWM driver to dynamically adjust the electrical load. The general form of a PID controller is expressed as:

$$
u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{d e(t)}{d t}
$$

Where:

- $$\( u(t) \) $$ is the control signal output,
- $$\( e(t) \) $$ is the error between target and actual speed,
- $$\( K_p \) $$ is the proportional gain,
- $$\( K_i \) $$ is the integral gain,
- $$\( K_d \) $$ is the derivative gain.

3. It counts the number of vehicles passing over the pseudo-bump to track energy input events.
4. It estimates the current height of the lifted mass by correlating the vehicle count and encoder data using the equation:

$$
h = N_{cars} \times h_{car} - \Delta E \times h_{encoder}
$$

   Where:

   - $$\( h \) $$ is the current height of the lifted mass
   - $$\( N_{cars} \) $$ is the number of cars passing over the pseudo-bump
   - $$\( h_{car} \) = $$ is the height increment per car press (how much one car lifts the mass)
   - $$\( \Delta E \) = $$ is the change in encoder readings during the last discharge
   - $$\( h_{encoder} \) = $$ is the height decrement per encoder pulse (how much one encoder pulse corresponds to the mass descending)

   This equation estimates the net height of the mass by aggregating the lifting increments caused by passing cars
   and subtracting the descent quantified by encoder pulses during discharging.
   
5. Based on the mass height and ambient lighting conditions, it determines the system's operating mode: switching to "maxCharge" when the mass height exceeds 105 units, entering "discharge" mode at night, and defaulting to "charge" or manual mode as selected through the dashboard interface.



### Code Explanation Flowchart
Below is a flowchart illustrating the workflow and main components of the code:

![Flowchart](https://github.com/salmandaher/VLNs_Team_GraviPower/blob/main/imgs/Python%20Node%20Flowchart.png)

### Code
You can view the full code [here](https://github.com/salmandaher/VLNs_Team_GraviPower/blob/main/src/vlns/src/serverna.py).

---

## The PRIZM code:
The PRIZM code is responsible for directly interfacing with hardware components to support the energy storage system's operation. It continuously reads encoder feedback to monitor the generator's rotational position and speed. The code controls the generator, turning it on during the fast charge mode and off during discharging. It detects when a car passes over the pseudo-bump to coordinate energy accumulation events. Furthermore, it manages the servo motor that controls the bump's state, flattening (deactivating) it when not charging and activating it during charging. Lastly, the PRIZM code reads ambient lighting sensor feedback to help determine system modes based on day/night conditions.

### Main tasks of the PRIZM code:

1. Reading encoder feedback for generator monitoring.
2. Controlling the generator state (activated during fast charge mode, deactivated during other modes).
3. Detecting cars passing over the pseudo-bump.
4. Managing the servo motor to flatten or activate the bump based on the mode.
5. Reading the lighting sensor to support automatic mode switching.


### Code Explanation Flowchart
Below is a flowchart illustrating the workflow and main components of the code:

![Flowchart](https://github.com/salmandaher/VLNs_Team_GraviPower/blob/main/imgs/PRIZM%20Node%20Flowchart.png)

### Code
You can view the full code [here](https://github.com/salmandaher/VLNs_Team_GraviPower/blob/main/src/vlns/src/PRIZM_node/PRIZM_node.ino).

---
## The ESP code:
The ESP code functions as the core controller for mechanical and electrical actuations within the energy storage system. It manages the disconnecting and connecting mechanism, dynamically linking the main pulley to the generator during discharge or fast charge modes, and to the gearbox in charge or max charge modes. The code also controls the electrical load system by activating the LED lights during discharge mode and deactivating it otherwise. Furthermore, the ESP code modulates the PWM driver (BTS), adjusting the electrical load on the generator in response to PID control signals computed externally, ensuring precise and efficient energy management.

### Main tasks of the ESP code:

1. Control the disconnecting and connecting mechanism, linking the main pulley to the generator or gearbox based on system mode.
2. Manage the electrical load system indicator light, turning it on only during discharge mode.
3. Regulate the PWM driver (BTS) to adjust the electrical load on the generator using PID output from the Python controller.

### Code Explanation Flowchart
Below is a flowchart illustrating the workflow and main components of the code:

![Flowchart](https://github.com/salmandaher/VLNs_Team_GraviPower/blob/main/imgs/ESP%20Node%20Flowchart.png)

### Code
You can view the full code [here](https://github.com/salmandaher/VLNs_Team_GraviPower/blob/main/src/vlns/src/Esp_Node/Esp_Node.ino).

---
## The Dashboard codes:

The Dashboard integrates frontend and backend components to provide a real-time, interactive user interface for the energy storage system. It combines HTML for structure, a custom CSS stylesheet for tailored styling and responsive design, and Node.js for backend communication management using server-side JavaScript. This interface displays current system status metrics, such as the height of the mass and the number of cars passing over the bump, and allows users to manually select or override operating modes through an intuitive interactive dashboard.

### Main tasks of the Dashboard:

1. Display system status in real time, including key metrics like mass height and vehicle count.
2. Enable users to change operating modes manually via an interactive interface.

### Code Explanation Flowchart
Below is a flowchart illustrating the workflow and main components of the code:

![Flowchart](https://github.com/salmandaher/VLNs_Team_GraviPower/blob/main/imgs/Dashboard%20Node%20Flowchart.png)



### Code Links:
You can view the codes here: 
- [HTML Code](https://github.com/salmandaher/VLNs_Team_GraviPower/blob/main/src/Web%20Dashboard/public/New/index3.html)
- [CSS Code](https://github.com/salmandaher/VLNs_Team_GraviPower/blob/main/src/Web%20Dashboard/public/New/style.css)
- [Node.js Code](https://github.com/salmandaher/VLNs_Team_GraviPower/blob/main/src/Web%20Dashboard/app.js)

---
# ROS Implementation

Since our project involves multiple controllers and a web-based dashboard, we implemented the concept of **ROS (Robot Operating System)** to facilitate communication between these controllers and maintain an organized and synchronized system workflow. The ROS version used in this project is **ROS 1 (Noetic)**

In the following sections, we explain the ROS structure that underpins the operation of our project — including detailed descriptions of each ROS node, how the nodes communicate with one another, and the types of information exchanged between them.

## Introduction

Our ROS system consists of four main nodes that communicate with each other to manage the overall functionality of the project, these nodes are in the **vlns** ROS package in the catkin workspace:

- **Python Node:**  
  Runs a Python program that serves as the central control of the system. It performs key calculations and manages the behavior of other nodes through ROS communication.

- **PRIZM Node:**  
  Executes an Arduino program that reads data from various sensors and controls the actuator used as a generator. It functions as an endpoint in the ROS communication network.

- **ESP Node:**  
  Runs another Arduino program that serves as an additional endpoint in the ROS communication structure, handling specific sensing or control tasks.

- **Dashboard Node:**  
  Responsible for displaying system values obtained from the controllers’ sensor readings. It also allows manual mode switching to adjust system behavior.


The following **rqt graph** illustrates the communication links and message flow between the nodes in our ROS system, which will be explained in detail later on:

![ROS Node Graph](https://github.com/salmandaher/VLNs_Team_GraviPower/blob/main/imgs/rosgraph.png)


To start the project, we open a terminal and launch the **ROS master** using the following command:
```bash
roscore
```
After that, each node should be started separately in a new terminal tab.
Before running any node, we source the terminal from the catkin workspace:
```bash
Source devel/setup.bash
```

---

## Python Node

The **Raspberry Pi** device (**Raspberry Pi 4**, running **Debian Bookworm**) serves as the **ROS Master** in our system. It hosts the main Python node that manages communication and coordination among all other nodes through various **services**, **publishers**, and **subscribers**.



### Services
- **Mode Service:**  
  The Python node participates in a mode management service with the Dashboard node.  
  When the mode is changed manually from the Dashboard, the Dashboard node sends a service request to the Python node containing the selected mode in the request message.  
  The Python node then updates the global variable representing the current mode and returns a success response to the Dashboard node.

We built our own **custom service structure**, which includes:  
- A **string variable** in the request message to specify the selected mode.  
- A **boolean variable** in the response message that returns `true` to indicate successful operation.  

You can view the service structure file [here](https://github.com/salmandaher/VLNs_Team_GraviPower/blob/main/src/vlns/srv/sges.srv).



### Publishers
- **`/mode`:**  
  The Python node continuously publishes the current mode to all other nodes, ensuring system-wide synchronization and state consistency.

- **`/height`:**  
  The node calculates the height of the mass based on sensor data and publishes it for display on the Dashboard.

- **`/cars_count`:**  
  The code tracks the number of times the bump push button is activated and publishes this value to be displayed on the Dashboard.

- **`/bts_value`:**  
  The node processes mass falling speed readings, applies a PID algorithm to compute the BTS value, and publishes it to control the BTS system—optimizing load management and fall speed.



### Subscribers
- **`/encoder`:**  
  Receives encoder data for BTS value and height calculations.  
- **`/car_passed`:**  
  Monitors signals indicating that a car has passed over the bump.  
- **`/light_sensor`:**  
   Reads light intensity data for use in mode adjustments.


### Communication with the ROS Master
The Python code runs as a ROS node on the Raspberry Pi. It registers with the ROS master using the ROS network protocol (ROS Master URI). The node uses ROS client libraries (rospy) to publish and subscribe to topics, provide or call services, and interact with other ROS nodes.

This is the terminal command for initiating the connection.
```bash
rosrun vlns serverna.py
```
---


## PRIZM Node

The **PRIZM node** is one of the two nodes in the system running Arduino code.  
The program on the PRIZM controller is responsible for several key tasks, in addition to participating in multiple ROS communications with other nodes.



### Publishers
- **`/encoder`:**  
  The PRIZM node reads the encoder values from the TorqNedo motor used as a generator and publishes them. These readings are used by the Python node for height and speed calculations.

- **`/light_sensor`:**  
  This node publishes data from the light sensor, which is subscribed to by the Python node to trigger the system’s autonomous behavior.

- **`/car_passed`:**  
  The code monitors the push button mounted on the bump and detects when it is pressed (FALLING state). These readings are published and subscribed to by other nodes in the system to track passing vehicles.



### Subscribers
- **`/mode`:**  
  The node subscribes to the mode topic published by the Python node. It adjusts its behavior based on the current mode — whether the system is **charging**, **discharging**, or **fast charging**.



### Communication with the ROS Master
The **PRIZM node** communicates with the **ROS Master** via a USB serial connection on port `/dev/ttyUSB0` with a baud rate of **57600**.  

To initiate this connection, we use the following terminal command:
```bash
rosrun rosserial_python serial_node.py /dev/ttyUSB0 _baud:= 57600
```
---

## ESP Node

The **ESP node** is another endpoint in the ROS communication system running Arduino code.  
It is responsible for controlling the switching mechanism and managing the load on the generator, which requires key information from other ROS nodes.



### Subscribers
- **`/mode`:**  
  Similar to the PRIZM node, this node subscribes to the **mode** topic published by the Python node.  
  It adjusts its behavior dynamically based on the current mode updates received from the ROS network.



### Communication with the ROS Master
The **ESP node** communicates with the **ROS Master** over a **TCP connection via Wi-Fi**.  
It connects to the Raspberry Pi device at the IP address **192.168.43.47** through **port 11411**, with a baud rate of **115200**.

This is the terminal command for initiating the connection.
```bash
rosrun rosserial_python serial_node.py tcp _baud:=115200
```
---

## Dashboard Node

The **Dashboard node** is an essential component of the ROS network that allows users to monitor system values and manually control its behavior.  
It participates in multiple ROS communications with other nodes to ensure accurate data display and responsive control.



### Services
- **Mode Service:**  
  When the user changes the mode manually using one of the mode buttons on the dashboard, the Dashboard node sends a service request to the Python node.  
  This request updates the global system mode to the newly selected one.


### Subscribers
- **`/mode`:**  
  The system mode can be changed either autonomously by internal system processes or manually from the dashboard.  
  In both cases, the Dashboard node subscribes to the **mode** topic published by the Python node and displays the current mode.  
  This ensures system synchronization and helps in detecting communication or logic errors.

- **`/car_passed`:**  
  Subscribes to the **car_passed** topic published by the PRIZM node and displays real-time status updates indicating when a car passes over the bump.

- **`/cars_count`:**  
  Subscribes to the **cars_count** topic (published by the PRIZM node) and displays the total number of cars that have passed over the bump since the system started.

- **`/height`:**  
  Subscribes to the **height** topic published by the Python node and displays the current height of the mass on the dashboard.



### Communication with the ROS Master
The dashboard is a web interface accessible from any device on the network. The dashboard web client connects to the ROS master through a Socket.IO server, which runs on the Raspberry Pi. It connects with the IP address of the Raspberry Pi on the network through port 3000.

This is the terminal command for initiating the connection.
```bash
node app.js
```

---

## Launch File

To simplify the process of running the project, we created a **launch file** — [**sges.launch**](https://github.com/salmandaher/VLNs_Team_GraviPower/blob/main/src/vlns/launch/sges.launch) — which automatically starts all the required nodes.  

After running the **roscore** and sourcing the terminal, we launch the entire system with a single command:
```bash
roslaunch vlns sges.launch
```
---
# Business Calculations
## Payback Time and Annual Revenue Calculation

The project's annual revenue from produced electricity is calculated using:

$$
R_{year} = \frac{E_{year}}{1000} \times 128.8
$$

Where:
- $R_{year}$ is the yearly revenue in dollars.
- $E_{year}$ is the total energy generated yearly in watt-hours (Wh).
- 128.8 is the price in dollars per 1000 kWh (where 1 kWh = 1000 Wh).

The payback time $T$ (in years) is calculated as:

$$
T = \frac{\text{Total Project Cost}}{R_{year}}
$$

Where:
- **Total Project Cost** is the full installation and equipment cost.
- $R_{year}$ is as defined above.

### Example:

If your system generates $E_{year} = 40\,000$ Wh per year, the yearly revenue is:

$$
R_{year} = \frac{40\,000}{1000} \times 128.8 = 40 \times 128.8 = 5\,152\,\text{USD}
$$

If the total project cost is $59\,000$ USD, the payback time is:

$$
T = \frac{59\,000}{5\,152} \approx 11.46\,\text{years}
$$

## ROI Calculation Method

ROI formula for your project:

$$
ROI = \frac{\text{Net Profit}}{\text{Cost of Investment}} \times 100
$$

## Real-Example Study

- **Power used per day:** $17\, \text{LEDs} \times 40\,\text{W} \times 6\,\text{h} = 4,080\,\text{Wh/day} = 4.08\,\text{kWh/day}$
- **Yearly usage:** $4.08\,\text{kWh/day} \times 365 = 1,489.2\,\text{kWh/year}$
- **Annual value at $128$/1,000 kWh:** 
  $$
  \text{Annual Value} = \frac{1,489.2}{1,000} \times 128 = \$190.63
  $$

### Homs City Data

- **Total investment cost:** \$8,648
- **Annual energy output:** $1,489.2\,\text{kWh/year}$
- **Estimated benefit:** $\$190.63$

### ROI for Year 1:

$$
ROI_{\text{Year 1}} = \frac{190.63}{8,648} \times 100 \approx 2.2\\%
$$

### ROI After 2 Years

Assuming similar annual revenue, the ROI after 2 years is:

$$
ROI_{\text{Year 2}} = \frac{2 \times 190.63}{8,648} \times 100 \approx 4.4\\%
$$

This assumes revenue accumulates linearly and no extra costs occur during the 2 years.

---

# Project Challenges

## Tower Design Evolution

Throughout the project's development, the tower structure underwent multiple design iterations due to various challenges. Below are three old versions of the tower with explanations on why each was changed:

### Version 1
![Tower Version 1](imgs/tower_v1.jpg)
The first version of the tower was made with a simple concept, four main columns where each two adjacent columns are connected with each other with flat beams.

### Version 2
![Tower Version 2](imgs/tower_v2.jpg)
The first version of the tower was somehow steady, but to increase its stability according to engineering concepts we removed the straight beams that connect every two adjacent channels and replaced them with triangular connections, to increase stability and efficiency reaching the second design version of the tower.

### Version 3
![Tower Version 3](imgs/tower_v3.jpg)
The second version seemed to work well at first, but we faced some problems with sitting it without tilting, it also costs too many Tetrix parts, which made us think of a solution for this problem leading us again to the third and final version of the tower, having a strong base to correct tilting, and high efficiency using fewer Tetrix parts.










