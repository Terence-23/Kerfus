pub mod drive;
pub mod utilities;
pub mod lidar;

use std::f32::consts::PI;

#[allow(unused)]
use drive::drive::{Drive, Direction, Wheel, Stepper, WHEEL_DISTANCE};
use utilities::geometry::Angle;
const STEPS: usize = 200;


fn main() {
    
    // let left_wheel = Wheel::new_from_diameter(
    //     Stepper::from_pin_nums(
    //         17,
    //         27,
    //         STEPS)
    //     .unwrap(), 
    //     1.0,
    //     0.067,
    //     false);
    // let right_wheel = Wheel::new_from_diameter(
    //     Stepper::from_pin_nums(
    //         23,
    //         24,
    //         STEPS)
    //     .unwrap(),
    //     1.0,
    //     0.067,
    //     false);

    // let mut drive = Drive::new(right_wheel, left_wheel, WHEEL_DISTANCE);

    
    // loop{
        // drive.go(Direction::Forward, 0.1);
        drive.turn(Angle::Radians(PI/2.0));
    // }
    
}
