pub enum Phase {
    Idle,
    Arming,
    Takeoff,
    Climb,
    Cruise,
    Descent,
    Landing,
    Landed,
}

impl Default for Phase {
    fn default() -> Self {
        Self::Idle
    }
}
