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

![Flowchart](https://github.com/ZainAlfai/VLNs-ZainAlfai-Testing/blob/main/white%20Python%20Node.png)

### Code
You can view the full code [here](relative/path/to/your_code.py).

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

![Flowchart](https://github.com/ZainAlfai/VLNs-ZainAlfai-Testing/blob/main/PRIZM%20Node%20(2).png)

### Code
You can view the full code [here](relative/path/to/your_code.py).

## The ESP code:
The ESP code functions as the core controller for mechanical and electrical actuations within the energy storage system. It manages the disconnecting and connecting mechanism, dynamically linking the main pulley to the generator during discharge or fast charge modes, and to the gearbox in charge or max charge modes. The code also controls the electrical load system by activating an LED lights during discharge mode and deactivating it otherwise. Furthermore, the ESP code modulates the PWM driver (BTS), adjusting the electrical load on the generator in response to PID control signals computed externally, ensuring precise and efficient energy management.

### Main tasks of the ESP code:

1. Control the disconnecting and connecting mechanism, linking the main pulley to the generator or gearbox based on system mode.
2. Manage the electrical load system indicator light, turning it on only during discharge mode.
3. Regulate the PWM driver (BTS) to adjust the electrical load on the generator using PID output from the Python controller.

### Code Explanation Flowchart
Below is a flowchart illustrating the workflow and main components of the code:

![Flowchart](https://github.com/ZainAlfai/VLNs-ZainAlfai-Testing/blob/main/ESP%20Node.drawio%20(1).png)

### Code
You can view the full code [here](relative/path/to/your_code.py).

## The Dashboard codes:

The Dashboard integrates frontend and backend components to provide a real-time, interactive user interface for the energy storage system. It combines HTML for structure, a custom CSS stylesheet for tailored styling and responsive design, and Node.js for backend communication management using server-side JavaScript. This interface displays current system status metrics, such as the height of the mass and the number of cars passing over the bump, and allows users to manually select or override operating modes through an intuitive interactive dashboard.

### Main tasks of the Dashboard:

1. Display system status in real time, including key metrics like mass height and vehicle count.
2. Enable users to manually change operating modes via an interactive interface.

### Code Explanation Flowchart
Below is a flowchart illustrating the workflow and main components of the code:

![Flowchart](https://github.com/ZainAlfai/VLNs-ZainAlfai-Testing/blob/main/Dashboard%20Node.drawio.png)



### Code Links:
You can view the codes here: 
- [HTML Code](relative/path/to/dashboard.html)
- [CSS Code](relative/path/to/bootstrap.css)
- [Node.js Code](relative/path/to/dashboard_server.js)
