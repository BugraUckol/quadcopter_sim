# Quadcopter Simulation

## Aim
This repository is developed to be able to simulate the flight behavior of quadcopters, especially to be used in control system design and validation.

![Plot Example](/pics/example.png "Example Results for Tello Drone")

## Submodules
This repository is a collection of rather small additions on top of an other repository `rigid_body_dynamics`. It adds:
* Several (currently one) drones
* Propeller forces

## Convention
In the `rigid_body_dynamics` repository, NED frame is used and body frame's z-axis is taken to be pointing below from the object. Here it continues with a minor change. In the propellers, directions of the propellers are taken `positive for the CCW` and `negative for the CW`, which does not agree with the frame attached to the body. However, it seems more natural when plotted.

## TODOs
* Add aerodynamic drags, moments etc.
* Add actuator dynamics
* Create a Simulink file instead of the script, which will be more modular