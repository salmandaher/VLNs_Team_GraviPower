# Solid-Gravity-Energy-Storage-System
Robotic Gravity-based energy storage system designed for smart cities — converts vehicle motion into storable mechanical energy and regenerates it electrically through controlled descent.
GraviPower-SGES/
│
├── README.md
├── LICENSE
├── docs/
│   ├── project_poster.pdf
│   ├── final_report.pdf
│   ├── presentation_slides.pdf
│
├── hardware/
│   ├── CAD/
│   │   ├── tower_model.step
│   │   ├── gearbox_assembly.step
│   │   └── bump_mechanism.stl
│   └── wiring_diagram.png
│
├── firmware/
│   ├── arduino/
│   │   ├── main_controller.ino
│   ├── esp32/
│   │   ├── Swicthing_Controller.ino
│
├── simulation/
│   ├── simulink_model.slx
│   ├── multibody_simulation.slx
│   └── matlab_scripts/
│       ├── system_parameters.m
│       └── generator_dynamics.m
│
├── ros/
│   ├── launch/
│   │   └── vlns.launch
│   ├── src/
│   │   └── control.py
│   └── srv/
│       └── moder.srv
│
└── ai/
    ├── traffic_analysis.ipynb
    ├── dataset/
    │   └── city_traffic_data.csv
    └── ai_site_selection_model.py
