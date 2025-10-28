# Solid-Gravity-Energy-Storage-System
Robotic Gravity-based energy storage system designed for smart cities — converts vehicle motion into storable mechanical energy and regenerates it electrically through controlled descent.

# The coding of the solution
## The main control code (the Python code)

This Python code serves as the central control unit for our system. It manages the system’s operational states, including charging, discharging, and fast charge modes, by coordinating commands to various controllers such as the PRIZM and the ESP. The code continuously monitors sensor data to make decisions that ensure smooth transitions between modes and optimize system performance. Additionally, it implements a PID control algorithm to regulate the descent speed of the mass, maximizing energy generation efficiency by dynamically adjusting the electrical load without wasting energy through conventional braking. The program also supports real-time monitoring and manual mode control via a connected web-based interface, enabling effective management of the entire energy storage process.

### Main Tasks of the Python Control Code

The Python script performs critical functions to optimize the operation of the gravity-based energy storage system:

1. It calculates the generator's rotational speed during discharging using encoder feedback.
2. It applies a PID (Proportional-Integral-Derivative) control algorithm to regulate the generator speed at the point of maximum power generation efficiency. The PID output is fed to the PWM driver to adjust the electrical load dynamically. The general form of a PID controller is expressed as:

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
You can view the full code [here](src/vlns/src/serverna.py).

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
You can view the full code [here](relative/path/to/your_code.py).

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
You can view the full code [here](relative/path/to/your_code.py).

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
- [HTML Code](relative/path/to/dashboard.html)
- [CSS Code](relative/path/to/bootstrap.css)
- [Node.js Code](relative/path/to/dashboard_server.js)

# ROS Implementation

Since our project involves multiple controllers and a web-based dashboard, we implemented the concept of **ROS (Robot Operating System)** to facilitate communication between these controllers and maintain an organized and synchronized system workflow.  

In the following sections, we explain the ROS structure that underpins the operation of our project — including detailed descriptions of each ROS node, how the nodes communicate with one another, and the types of information exchanged between them.

---

## Table of Contents
1. [Introduction](#introduction)
2. [Raspberry Pi Node](#raspberry-pi-node)
3. [PRIZM Node](#prizm-node)
4. [ESP Node](#esp-node)
5. [Dashboard Node](#dashboard-node)

---

## Introduction

Our ROS system consists of four main nodes that communicate with each other to manage the overall functionality of the project:

- **Raspberry Pi Node:**  
  Runs a Python program that serves as the central control of the system. It performs key calculations and manages the behavior of other nodes through ROS communication.

- **PRIZM Node:**  
  Executes an Arduino program that reads data from various sensors and controls the actuator used as a generator. It functions as an endpoint in the ROS communication network.

- **ESP Node:**  
  Runs another Arduino program that serves as an additional endpoint in the ROS communication structure, handling specific sensing or control tasks.

- **Dashboard Node:**  
  Responsible for displaying system values obtained from the controllers’ sensor readings. It also allows manual mode switching to adjust system behavior.

---

### ROS Node Connection Graph
The following **rqt graph** illustrates the communication links and message flow between the nodes in our ROS system:

![ROS Node Graph](images/rqt.png)


---

## Raspberry Pi Node

The **Raspberry Pi** device serves as the **ROS Master** in our system. It hosts the main Python node that manages communication and coordination among all other nodes through various **services**, **publishers**, and **subscribers**.

---

### Services
- **Mode Service:**  
  The Python node participates in a mode management service with the Dashboard node.  
  When the mode is changed manually from the Dashboard, the Dashboard node sends a service request to the Python node containing the selected mode in the request message.  
  The Python node then updates the global variable representing the current mode and returns a success response to the Dashboard node.

---

### Publishers
- **`/mode`:**  
  The Python node continuously publishes the current mode to all other nodes, ensuring system-wide synchronization and state consistency.

- **`/height`:**  
  The node calculates the height of the mass based on sensor data and publishes it for display on the Dashboard.

- **`/cars_count`:**  
  The code tracks the number of times the bump push button is activated and publishes this value to be displayed on the Dashboard.

- **`/bts_value`:**  
  The node processes mass falling speed readings, applies a PID algorithm to compute the BTS value, and publishes it to control the BTS system—optimizing load management and fall speed.

---

### Subscribers
- **`/encoder`:**  
  Receives encoder data for BTS value and height calculations.  
- **`/car_passed`:**  
  Monitors signals indicating that a car has passed over the bump.  
- **`/light_sensor`:**  
   Reads light intensity data for use in mode adjustments.

---

## PRIZM Node

The **PRIZM node** is one of the two nodes in the system running Arduino code.  
The program on the PRIZM controller is responsible for several key tasks, in addition to participating in multiple ROS communications with other nodes.

---

### Publishers
- **`/encoder`:**  
  The PRIZM node reads the encoder values from the TorqNedo motor used as a generator and publishes them. These readings are used by the Python node for height and speed calculations.

- **`/light_sensor`:**  
  This node publishes data from the light sensor, which is subscribed to by the Python node to trigger the system’s autonomous behavior.

- **`/car_passed`:**  
  The code monitors the push button mounted on the bump and detects when it is pressed (FALLING state). These readings are published and subscribed to by other nodes in the system to track passing vehicles.

---

### Subscribers
- **`/mode`:**  
  The node subscribes to the mode topic published by the Python node. It adjusts its behavior based on the current mode — whether the system is **charging**, **discharging**, or **fast charging**.

---

## ESP Node

The **ESP node** is another endpoint in the ROS communication system running Arduino code.  
It is responsible for controlling the switching mechanism and managing the load on the generator, which requires key information from other ROS nodes.

---

### Subscribers
- **`/mode`:**  
  Similar to the PRIZM node, this node subscribes to the **mode** topic published by the Python node.  
  It adjusts its behavior dynamically based on the current mode updates received from the ROS network.

---

## Dashboard Node

The **Dashboard node** is an essential component of the ROS network that allows users to monitor system values and manually control its behavior.  
It participates in multiple ROS communications with other nodes to ensure accurate data display and responsive control.

---

### Services
- **Mode Service:**  
  When the user changes the mode manually using one of the mode buttons on the dashboard, the Dashboard node sends a service request to the Python node.  
  This request updates the global system mode to the newly selected one.

---

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

### our study case

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

Assuming similar annual revenue the ROI after 2 years is:

$$
ROI_{\text{Year 2}} = \frac{2 \times 190.63}{8,648} \times 100 \approx 4.4\\%
$$

This assumes revenue accumulates linearly and no extra costs occur during the 2 years.



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

### our study case

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

Assuming similar annual revenue the ROI after 2 years is:

$$
ROI_{\text{Year 2}} = \frac{2 \times 190.63}{8,648} \times 100 \approx 4.4\\%
$$

This assumes revenue accumulates linearly and no extra costs occur during the 2 years.











