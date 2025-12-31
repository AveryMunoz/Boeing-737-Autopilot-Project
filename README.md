# Boeing 737 Autopilot Project
This is a full flight simulation for a boeing 737-800 plane using real world parameters taken from online sources as well as dual PID (proportional-integral-derivative) controllers to simulate realistic flight.
## Features
- Full flight profile: climb (starting from 1000 ft), cruise (2 hours at 35,000 ft), and descent (down to ~3000 ft)
- Dual PID controllers for airspeed (Velocity upwards and horizontally) and altitude tracking
- Realistic aerodynamics, atmospheric, and thrust modeling
- Multi-phase flight management with smooth transitions to simulate realistic and safe flight
- ## Results
![Flight Simulation Results](flight_simulation_results.png)
The simulation displays steady climb for 3 different phases, altitude control at 35000 ft with ±250 ft error, and steady descent
## Usage
Run `fullFlightSimulation.m` in MATLAB with required dependencies.
