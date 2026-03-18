use approx::assert_relative_eq;
use flight_core::controller::*;
use flight_core::simulator::*;
use flight_core::units::traits::RawRepresentable;
use flight_core::units::*;

#[test]
fn hover() {
    let target_altitude = 10.meters();
    let delta_t = 0.1.seconds();
    let drone = DefaultDrone {};
    let mut simulator = Simulator::new(drone);
    simulator.state.position_ned = PositionNed::from_altitude_ned(-10.meters());

    let mut flight_controller = FlightController::for_drone(drone);
    flight_controller.set_target_down(AxisTarget::Loiter);

    for _ in 0..100 {
        let inputs = flight_controller.update(&simulator.state, delta_t);
        simulator.tick(&inputs, delta_t);
    }

    assert_relative_eq!(
        simulator.state.altitude.raw(),
        target_altitude.raw(),
        epsilon = 1e-2
    );
}

#[test]
fn north_velocity() {
    let target_velocity = VelocityNed::new(5.mps(), 0.mps(), 0.mps());
    let delta_t = 0.01.seconds();
    let drone = DefaultDrone {};
    let mut simulator = Simulator::new(drone);
    simulator.state.position_ned = PositionNed::from_altitude_ned(-10.meters()); // starts in air.
    let mut flight_controller = FlightController::for_drone(drone);
    flight_controller.set_target_north(AxisTarget::Velocity(target_velocity.north()));
    flight_controller.set_target_down(AxisTarget::Loiter);

    for _ in 0..1000 {
        let inputs = flight_controller.update(&simulator.state, delta_t);
        simulator.tick(&inputs, delta_t);
    }

    assert_relative_eq!(
        simulator.state.velocity_ned.north().raw(),
        target_velocity.north().raw(),
        epsilon = 1e-1
    );
}

#[test]
fn east_velocity() {
    let target_velocity = VelocityNed::new(0.mps(), 2.mps(), 0.mps());
    let delta_t = 0.1.seconds();
    let drone = DefaultDrone {};

    let mut simulator = Simulator::new(drone);
    simulator.state.position_ned = PositionNed::from_altitude_ned(-10.meters()); // starts in air.
    let mut flight_controller = FlightController::for_drone(drone);
    flight_controller.set_target_east(AxisTarget::Velocity(target_velocity.east()));
    flight_controller.set_target_down(AxisTarget::Loiter);

    for _ in 0..100 {
        let inputs = flight_controller.update(&simulator.state, delta_t);
        simulator.tick(&inputs, delta_t);
    }

    assert_relative_eq!(
        simulator.state.velocity_ned.east().raw(),
        target_velocity.east().raw(),
        epsilon = 1e-1
    );
}

#[test]
fn increases_altitude_east_north_velocity() {
    let target_altitude = 10.meters();
    let target_velocity = VelocityNed::new(5.mps(), 2.mps(), 0.mps());
    let delta_t = 0.01.seconds();
    let drone = DefaultDrone {};

    let mut simulator = Simulator::new(drone);
    simulator.state.position_ned = PositionNed::from_altitude_ned(-target_altitude);

    let mut flight_controller = FlightController::for_drone(drone);
    flight_controller.set_target_down(AxisTarget::Loiter);
    flight_controller.set_target_north(AxisTarget::Velocity(target_velocity.north()));
    flight_controller.set_target_east(AxisTarget::Velocity(target_velocity.east()));

    for _ in 0..1000 {
        let inputs = flight_controller.update(&simulator.state, delta_t);
        simulator.tick(&inputs, delta_t);
    }

    assert_relative_eq!(
        simulator.state.velocity_ned.east().raw(),
        target_velocity.east().raw(),
        epsilon = 1e-1
    );

    assert_relative_eq!(
        simulator.state.velocity_ned.east().raw(),
        target_velocity.east().raw(),
        epsilon = 1e-1
    );

    assert_relative_eq!(
        simulator.state.altitude.raw(),
        target_altitude.raw(),
        epsilon = 1e-1
    );
}

#[test]
fn controller_yaw_rate_rotates_drone() {
    let drone = DefaultDrone {};
    let mut sim = Simulator::new(drone);
    let mut flight_controller = FlightController::for_drone(drone);

    // Command a yaw rate
    flight_controller.set_yaw_rate_target(1.0.into());

    let initial_yaw = sim.state.attitude.yaw().to_degrees().0;

    let dt = 0.01.seconds();
    for _ in 0..200 {
        let inputs = flight_controller.update(&sim.state, dt);
        sim.tick(&inputs, dt);
    }

    let final_yaw = sim.state.attitude.yaw().to_degrees().0;
    assert!(
        final_yaw > initial_yaw + 10.0,
        "expected yaw to increase, got {} -> {}",
        initial_yaw,
        final_yaw
    );
}

#[test]
fn position_target_overshoot_within_tolerance() {
    let target_north = 20.meters();
    let delta_t = 0.01.seconds();
    let drone = DefaultDrone {};

    let mut simulator = Simulator::new(drone);
    simulator.state.position_ned = PositionNed::from_altitude_ned(-10.meters());

    let mut flight_controller = FlightController::for_drone(drone);
    flight_controller.set_target_north(AxisTarget::Position(target_north));
    flight_controller.set_target_east(AxisTarget::Loiter);
    flight_controller.set_target_down(AxisTarget::Loiter);

    let mut max_north = f64::NEG_INFINITY;

    for _ in 0..3000 {
        let inputs = flight_controller.update(&simulator.state, delta_t);
        simulator.tick(&inputs, delta_t);
        max_north = max_north.max(simulator.state.position_ned.north().raw());
    }

    let overshoot = (max_north - target_north.raw()).max(0.0);
    assert!(
        overshoot < 1.0,
        "overshoot too large: {:.2}m past target {:.1}m (max north reached: {:.2}m)",
        overshoot,
        target_north,
        max_north
    );
}

#[test]
fn velocity_to_zero_overshoot_within_tolerance() {
    let delta_t = 0.01.seconds();
    let drone = DefaultDrone {};

    let mut simulator = Simulator::new(drone);
    simulator.state.position_ned = PositionNed::from_altitude_ned(-10.meters());

    let mut flight_controller = FlightController::for_drone(drone);
    flight_controller.set_target_north(AxisTarget::Velocity(5.mps()));
    flight_controller.set_target_east(AxisTarget::Loiter);
    flight_controller.set_target_down(AxisTarget::Loiter);

    // Accelerate for 5 seconds
    for _ in 0..500 {
        let inputs = flight_controller.update(&simulator.state, delta_t);
        simulator.tick(&inputs, delta_t);
    }

    // Now command loiter — drone should decelerate and hold
    flight_controller.set_target_north(AxisTarget::Loiter);
    let position_at_loiter = simulator.state.position_ned.north().raw();
    let mut max_north = position_at_loiter;

    // One tick to trigger loiter capture inside the trajectory generator
    let inputs = flight_controller.update(&simulator.state, delta_t);
    simulator.tick(&inputs, delta_t);

    let loiter_target = flight_controller.loiter_targets().x;

    println!("Loiter target: {:?}", loiter_target);

    println!("LOITER!");

    for i in 0..2000 {
        let inputs = flight_controller.update(&simulator.state, delta_t);
        simulator.tick(&inputs, delta_t);
        max_north = max_north.max(simulator.state.position_ned.north().raw());

        if i % 10 == 0 {
            println!(
                "tick={}, pos={:.3}, vel={:.4}, max={:.3}",
                i,
                simulator.state.position_ned.north(),
                simulator.state.velocity_ned.north(),
                max_north
            );
        }
    }

    let overshoot = (max_north - loiter_target.unwrap().raw()).max(0.0);
    assert!(
        overshoot < 1.0,
        "loiter overshoot too large: {:.2}m (position at loiter: {:.2}m, max: {:.2}m)",
        overshoot,
        position_at_loiter,
        max_north
    );
}
