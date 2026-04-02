# apsis-rs

A flight controller and physics simulator written in Rust. Built from first principles, following aerospace industry conventions (NED/FRD coordinate frames, PID cascaded architecture). 

This is a **learning project**. The goal is to deeply understand flight controller design and industry conventions while writing idiomatic, well-typed Rust.

![CleanShot 2026-04-02 at 11 00 33](https://github.com/user-attachments/assets/80447859-ef96-4bdb-bc97-261270686cb0)


## AI Disclosure

No "vibe-coding" has been used in this project, and AI has never had access to the full source code. AI was used as a learning aid and general guidance tool to understand aerospace conventions and flight controller theory. AI-generated content is limited to: unit tests and documentation/clarifying comments, all of which have been reviewed and most modified to fit.

# Build & Run

```bash
# Run the simulator with real-time telemetry UI
cargo run -p telemetry_ui

# Run tests
cargo test
```

---

## Crates

### `flight_core`

The heart of the project. Contains:

- **Simulator** — physics engine integrating quaternion kinematics, thrust model, drag, and angular dynamics at ~130 Hz
- **Controller** — full cascaded PID pipeline: `TrajectoryGenerator → VelocityController → AttitudeController → RateController → Inputs`

### `primitives`

Strongly-typed unit system (`Meters`, `Velocity`, `Acceleration`, `Jerk`, `Radians`, `Quaternion`, ...) with unit algebra enforced at compile time.

```rust
let distance: Meters = 2.mps() * 1.seconds(); // d = v * t
```

### `telemetry_ui`

Real-time telemetry display built with [egui](https://github.com/emilk/egui).

Pilot input (W/A/S/D + arrow keys) is body-frame.
W: Forward
S: Backward
A: Left
D: Right
↑: + Altitude
↓: - Altitude
←: Rotate counter-clock wise
→: Rotate clock wise.
R: Reset commands.

---

## Controller architecture

```
Pilot Input (body frame)
        ↓  rotate_body_to_world()
  FlightTarget (AxisTarget per axis)
        ↓
  TrajectoryGenerator   ← jerk/accel/velocity limits derived from specs
        ↓
  PositionController (PID - target from trajectory)
        ↓
  (*) VelocityController (P + ARW integral + D on acceleration + ff)
        ↓
  q_target_from_acceleration()
  throttle_from_acceleration() - tilt-compensated
        ↓
  AttitudeController (PID)
        ↓  
  RateController (PID)
        ↓
  Inputs { throttle, pitch, roll, yaw }
        ↓
  Simulator
```

(*) *VelocityController* note: Not a standard PID. The derivative term acts on measured acceleration (not velocity error derivative) to avoid setpoint-kick noise. The integral uses Anti-Reset Windup (ARW / tracking). The saturation error is fed back to modify the integrated error before accumulation, keeping the integrator from winding up when output is clamped. The vertical axis uses a conditional integrator instead of ARW: it only accumulates when the saturation error and velocity error have opposite signs (i.e. the output is not already pushing in the right direction).

Coordinate conventions follow aerospace standard throughout: **NED world frame, FRD body frame, positive pitch = nose up**.

---

## References

- [Quaternion Algebra](https://graphics.stanford.edu/courses/cs348a-17-winter/Papers/quaternion.pdf)
- [Trajectory Generator](https://arxiv.org/pdf/2105.04830)
