pub mod drive;

use std::f32::consts::PI;

#[allow(unused)]
use drive::drive::{Drive, Direction, Angle, Wheel, Stepper, WHEEL_DISTANCE};

const STEPS: usize = 180;

fn main() {
    
    let left_wheel = Wheel::new_from_diameter(Stepper::from_pin_nums(15, 14, STEPS).unwrap(), 1.0, 125.0, false);
    let right_wheel = Wheel::new_from_diameter(Stepper::from_pin_nums(23, 24, STEPS).unwrap(), 1.0, 125.0, false);

    let mut drive = Drive::new(right_wheel, left_wheel, WHEEL_DISTANCE);

    
    loop{
        drive.go(Direction::Forward, 0.5);
        drive.turn(Angle::Radians(PI/2.0));
    }
    
}
