pub trait Ned<T> {
    fn new(north: T, east: T, down: T) -> Self;

    fn north(&self) -> T;
    fn east(&self) -> T;
    fn down(&self) -> T;

    fn update_north(&mut self, new_value: T);
    fn update_east(&mut self, new_value: T);
    fn update_down(&mut self, new_value: T);

    fn add_north(&mut self, added_value: T);
    fn add_east(&mut self, added_value: T);
    fn add_down(&mut self, added_value: T);

    // fn clamp_north(&mut self, min: T, max: T);
    // fn clamp_east(&mut self, min: T, max: T);
    // fn clamp_down(&mut self, min: T, max: T);
}

#[macro_export]
macro_rules! impl_ned_vec3 {
    ($type:ident, $inner:ty) => {
        impl Ned<$inner> for $type {
            fn new(north: $inner, east: $inner, down: $inner) -> Self {
                Self(Vec3 {
                    x: north,
                    y: east,
                    z: down,
                })
            }

            fn north(&self) -> $inner {
                self.0.x
            }
            fn east(&self) -> $inner {
                self.0.y
            }
            fn down(&self) -> $inner {
                self.0.z
            }

            fn update_north(&mut self, v: $inner) {
                self.0.x = v;
            }
            fn update_east(&mut self, v: $inner) {
                self.0.y = v;
            }
            fn update_down(&mut self, v: $inner) {
                self.0.z = v;
            }

            fn add_north(&mut self, v: $inner) {
                self.0.x += v;
            }
            fn add_east(&mut self, v: $inner) {
                self.0.y += v;
            }
            fn add_down(&mut self, v: $inner) {
                self.0.z += v;
            }

            // fn clamp_north(&mut self, min: $inner, max: $inner) {
            //     self.0.x = self.0.x.clamp(min, max);
            // }
            // fn clamp_east(&mut self, min: $inner, max: $inner) {
            //     self.0.y = self.0.y.clamp(min, max);
            // }
            // fn clamp_down(&mut self, min: $inner, max: $inner) {
            //     self.0.z = self.0.z.clamp(min, max);
            // }
        }
    };
}
