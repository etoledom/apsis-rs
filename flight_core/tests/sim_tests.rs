use approx::assert_relative_eq;
use flight_core::controller::*;
use flight_core::simulator::*;
use flight_core::units::angles::DegreesLiteral;
use flight_core::units::*;

#[test]
fn hover() {
    let target_altitude = 10.meters();
    let delta_t = 0.1.seconds();

    let mut simulator = Simulator::new(DefaultDrone {});
    let mut flight_controller = FlightController::new(target_altitude, VelocityNED::default());

    for _ in 0..100 {
        let inputs = flight_controller.update(&simulator.state, delta_t);
        simulator.tick(&inputs, delta_t);
    }

    assert_relative_eq!(
        simulator.state.altitude.0,
        target_altitude.0,
        epsilon = 1e-2
    );
}

#[test]
fn north_velocity() {
    let target_velocity = VelocityNED::new(5.mps(), 0.mps(), 0.mps());
    let delta_t = 0.1.seconds();

    let mut simulator = Simulator::new(DefaultDrone {});
    simulator.state.position_ned = PositionNed::from_altitude_ned(-10.meters()); // starts in air.
    let mut flight_controller = FlightController::new(10.meters(), target_velocity);

    for _ in 0..100 {
        let inputs = flight_controller.update(&simulator.state, delta_t);
        simulator.tick(&inputs, delta_t);
    }

    assert_relative_eq!(
        simulator.state.velocity_ned.north().0,
        target_velocity.north().0,
        epsilon = 1e-1
    );
}

#[test]
fn east_velocity() {
    let target_velocity = VelocityNED::new(0.mps(), 2.mps(), 0.mps());
    let delta_t = 0.1.seconds();

    let mut simulator = Simulator::new(DefaultDrone {});
    simulator.state.position_ned = PositionNed::from_altitude_ned(-10.meters()); // starts in air.
    let mut flight_controller = FlightController::new(10.meters(), target_velocity);

    for _ in 0..100 {
        let inputs = flight_controller.update(&simulator.state, delta_t);
        simulator.tick(&inputs, delta_t);
    }

    assert_relative_eq!(
        simulator.state.velocity_ned.east().0,
        target_velocity.east().0,
        epsilon = 1e-1
    );
}

#[test]
fn increases_altitude_east_north_velocity() {
    let target_altitude = 10.meters();
    let target_velocity = VelocityNED::new(5.mps(), 2.mps(), 0.mps());
    let delta_t = 0.1.seconds();

    let mut simulator = Simulator::new(DefaultDrone {});
    let mut flight_controller = FlightController::new(target_altitude, target_velocity);

    for _ in 0..100 {
        let inputs = flight_controller.update(&simulator.state, delta_t);
        simulator.tick(&inputs, delta_t);
    }

    assert_relative_eq!(
        simulator.state.velocity_ned.east().0,
        target_velocity.east().0,
        epsilon = 1e-1
    );

    assert_relative_eq!(
        simulator.state.velocity_ned.east().0,
        target_velocity.east().0,
        epsilon = 1e-1
    );

    assert_relative_eq!(
        simulator.state.altitude.0,
        target_altitude.0,
        epsilon = 1e-1
    );
}

#[test]
fn yaw_reaches_target() {
    let drone = DefaultDrone {};
    println!("max_heading_rate: {:?}", drone.max_heading_rate());

    let mut sim = Simulator::new(DefaultDrone {});
    let mut controller = FlightController::new(10.0.meters(), Default::default());

    // Set a yaw target of 45°
    controller.set_heading_rate_target(angles::DegreesPerSecond(30.0));

    // Run for 5 seconds
    let steps = 500;
    let dt = (5.0 / steps as f64).seconds();

    for i in 0..steps {
        let inputs = controller.update(&sim.state, dt);
        sim.tick(&inputs, dt);

        if i % 50 == 0 {
            println!(
                "t={:.1}s yaw={:.2}° ang_vel_z={:.4}",
                i as f64 * dt.0,
                sim.state.attitude.yaw().to_degrees().0,
                sim.state.angular_velocity_body.z(),
            );
        }
    }

    // Should have rotated significantly in 5 seconds at 30°/s
    let final_yaw = sim.state.attitude.yaw().to_degrees().0;
    assert!(
        final_yaw > 10.0,
        "yaw should have increased significantly, got {}",
        final_yaw
    );
}
