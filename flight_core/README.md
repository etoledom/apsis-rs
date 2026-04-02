# flight_core

Core library for the [apsis-rs](../README.md) flight controller and simulator.

## Contents

- **Simulator** — physics engine integrating quaternion kinematics, thrust model, drag, and angular dynamics
- **Controller** — cascaded PID pipeline: `TrajectoryGenerator → VelocityController → AttitudeController → RateController`

## Usage

This crate is a library — it is not intended to be run directly. See `telemetry_ui` for the binary that runs the simulator with a real-time display.
```bash
cargo test -p flight_core
```
