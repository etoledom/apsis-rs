use std::{
    marker::PhantomData,
    ops::{Add, Sub},
};

use crate::{
    simulator::types::vec2::Vec2,
    units::{
        acceleration::{Acceleration, AccelerationLiteral},
        angles::Radians,
    },
};

#[derive(Copy, Clone, Debug)]
pub struct Acceleration3D<F> {
    pub vertical: Acceleration,
    pub forward: Acceleration,
    pub right: Acceleration,
    _frame: core::marker::PhantomData<F>,
}

impl<F> Acceleration3D<F> {
    pub fn new(forward: Acceleration, right: Acceleration, vertical: Acceleration) -> Self {
        Self {
            vertical,
            forward,
            right,
            _frame: PhantomData,
        }
    }

    pub fn vertical(vertical: Acceleration) -> Self {
        Self {
            vertical,
            forward: 0.mps2(),
            right: 0.mps2(),
            _frame: PhantomData,
        }
    }

    pub fn ground_acceleration(&self) -> Vec2<Acceleration> {
        Vec2::new(self.right, self.forward)
    }

    fn rotate(&self, angle: Radians) -> Acceleration3D<F> {
        let ground_acceleration = self.ground_acceleration();
        let rotated = ground_acceleration.rotate(angle);
        Acceleration3D::new(rotated.y, rotated.x, self.vertical)
    }
}

impl<F> Add for Acceleration3D<F> {
    type Output = Acceleration3D<F>;

    fn add(self, rhs: Acceleration3D<F>) -> Self::Output {
        Acceleration3D {
            vertical: self.vertical + rhs.vertical,
            forward: self.forward + rhs.forward,
            right: self.right + rhs.right,
            _frame: PhantomData,
        }
    }
}

impl<F> Sub for Acceleration3D<F> {
    type Output = Acceleration3D<F>;

    fn sub(self, rhs: Acceleration3D<F>) -> Self::Output {
        Acceleration3D {
            vertical: self.vertical - rhs.vertical,
            forward: self.forward - rhs.forward,
            right: self.right - rhs.right,
            _frame: PhantomData,
        }
    }
}

pub struct BodyFrameAccelerationType;
pub type BodyFrameAcceleration = Acceleration3D<BodyFrameAccelerationType>;

pub struct WorldFrameAccelerationType;
pub type WorldFrameAcceleration = Acceleration3D<WorldFrameAccelerationType>;

impl BodyFrameAcceleration {
    pub fn rotate_to_world_frame_by(self, angle: Radians) -> WorldFrameAcceleration {
        let rotated = self.rotate(angle);
        WorldFrameAcceleration::new(rotated.forward, rotated.right, rotated.vertical)
    }
}
