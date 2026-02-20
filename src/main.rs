mod controller;

mod simulator;
mod units;

use simulator::default_drone::DefaultDrone;
use simulator::simulator::Simulator;
use simulator::types;
use units::units::Seconds;

use crate::controller::flight_controller::{AirframeLimits, FlightController};
use crate::simulator::drone::Drone;
use crate::simulator::force_model::context::Context;
use crate::simulator::force_model::force_model::ForceModel;
use crate::simulator::types::acceleration_3d::WorldFrameAcceleration;
use crate::simulator::types::velocity_ned::VelocityNED;
use crate::units::acceleration::Acceleration;
use crate::units::units::{MettersLiteral, SecondsLiteral, VelocityLiteral};

struct Wind;

impl<Vehicle: Drone> ForceModel<Vehicle> for Wind {
    fn acceleration_contribution<'a>(
        &self,
        _: &Context<Vehicle>,
        _: Seconds,
    ) -> WorldFrameAcceleration {
        WorldFrameAcceleration::new(Acceleration(0.5), Acceleration(0.5), Acceleration(0.0))
    }
}

fn main() {
    let mut sim_time = Seconds::zero();

    let wind = Wind {};

    let mut simulator = Simulator::new(DefaultDrone {});
    simulator.add_force(wind);
    //
    let target_altitude = 10.meters();
    let target_north_speed = 5.mps();

    let mut flight_controller = FlightController::new(
        target_altitude,
        VelocityNED::default(),
        AirframeLimits {
            max_pitch: simulator.drone.max_pitch().to_radians(),
            max_roll: simulator.drone.max_roll().to_radians(),
        },
    );

    let delta_t = 0.1.seconds();

    for i in 0..800 {
        let inputs = flight_controller.update(&simulator.state, delta_t);

        simulator.tick(&inputs, delta_t);

        sim_time = sim_time + delta_t;

        if (simulator.state.altitude - target_altitude).0.abs() < 0.01
            && (simulator.state.velocity_ned.north()).0 < 0.1
            && (simulator.state.velocity_ned.east()).0 < 0.1
        {
            flight_controller.set_target_velocity(VelocityNED::new(5.mps(), 0.mps(), 0.mps()));
        }

        if (simulator.state.altitude - target_altitude).0.abs() < 0.01
            && (simulator.state.velocity_ned.north() - target_north_speed)
                .0
                .abs()
                < 0.1
            && (simulator.state.velocity_ned.east()).0 < 0.1
        {
            flight_controller.set_target_velocity(VelocityNED::new(3.mps(), 2.mps(), 0.mps()));
        }

        if i % 10 == 0 {
            println!(
                "Time: {}, Throttle: {:.2}, target_v_n: {}, Velocity_north: {}, Velocity_east: {}, Pitch: {}, Roll: {}, velocity_up: {}, Altitude: {}",
                sim_time,
                inputs.throttle.get(),
                flight_controller.get_target_velocity().north(),
                simulator.state.velocity_ned.north(),
                simulator.state.velocity_ned.east(),
                simulator.state.attitude.pitch().to_degrees(),
                simulator.state.attitude.roll().to_degrees(),
                -simulator.state.velocity_ned.down(),
                simulator.state.altitude,
            );
        }
    }
}

#[cfg(test)]
mod pid_tunning_test {
    use crate::simulator::types::position_ned::PositionNed;

    use super::*;

    #[test]
    fn velocity_north_step_response() {
        let mut ctrl = FlightController::new(
            10.meters(),
            VelocityNED::new(5.mps(), 0.mps(), 0.mps()),
            AirframeLimits::default(),
        );
        let mut sim = Simulator::new(DefaultDrone {});
        sim.state.position_ned = PositionNed::from_altitude_ned(-10.meters());

        let dt = 0.1.seconds();
        let mut rise_time = None;
        let mut max_velocity = 0.0_f64;

        for i in 0..500 {
            let inputs = ctrl.update(&sim.state, dt);
            sim.tick(&inputs, dt);
            let t = i as f64 * dt.0;
            let v = sim.state.velocity_ned.north().0;
            max_velocity = max_velocity.max(v);
            if rise_time.is_none() && v >= 5.0 * 0.9 {
                rise_time = Some(t);
            }
            println!(
                "t={:.1} v_north={:.2} pitch={:.1}",
                t,
                v,
                sim.state.attitude.pitch().to_degrees().0
            );
        }

        let final_velocity = sim.state.velocity_ned.north().0;
        let overshoot = ((max_velocity - 5.0) / 5.0) * 100.0;

        println!("Rise time: {:?}s", rise_time);
        println!("Max velocity: {:.2} m/s", max_velocity);
        println!("Overshoot: {:.1}%", overshoot);
        println!("Final velocity: {:.2} m/s", final_velocity);
        println!(
            "Steady state error: {:.2} m/s",
            (5.0 - final_velocity).abs()
        );

        assert!(rise_time.is_some(), "should reach 90% of target");
        assert!(overshoot < 20.0, "overshoot should be less than 20%");
        assert!(
            (final_velocity - 5.0).abs() < 0.1,
            "steady state error should be less than 0.1 m/s"
        );
    }
}
