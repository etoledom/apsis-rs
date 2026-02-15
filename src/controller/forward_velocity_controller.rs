use crate::{
    controller::{gain::LinearGain, pid::ForwardVelocityPID},
    units::{
        angles::Radians,
        units::{Seconds, Velocity},
    },
};

pub struct ForwardVelocityController {
    velocity_target: Velocity,
    max_pitch: Radians,
    forward_velocity_pid: ForwardVelocityPID,
}

impl ForwardVelocityController {
    pub fn new(target: Velocity, max_pitch: Radians) -> Self {
        Self {
            velocity_target: target,
            max_pitch,
            forward_velocity_pid: ForwardVelocityPID::new(
                LinearGain::new(0.2),
                LinearGain::zero(),
                LinearGain::zero(),
            ),
        }
    }
    pub fn update(&mut self, current_forward_velocity: Velocity, dt: Seconds) -> Radians {
        let error = self.velocity_target - current_forward_velocity;

        self.forward_velocity_pid
            .update(error, dt)
            .clamp(-self.max_pitch, self.max_pitch)
    }
}

#[cfg(test)]
mod tests {
    use crate::units::{
        angles::DegreesLiteral,
        units::{SecondsLiteral, VelocityLiteral},
    };

    use super::*;

    #[test]
    fn test_name() {
        let mut controller = ForwardVelocityController::new(5.mps(), 45.degrees().to_radians());
        let output = controller.update(0.mps(), 1.seconds());

        assert_eq!(output.to_degrees().0, 45.0);
    }
}
