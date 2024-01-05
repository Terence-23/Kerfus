pub mod drive;
pub mod encoder;
pub mod lidar;
pub mod utilities;
pub mod wall_detection;

use std::{f32::consts::PI, fs::OpenOptions};

use as5600::As5600;
use drive::drive::Brushless;
#[allow(unused)]
use drive::drive::{Direction, Drive, Stepper, WHEEL_DISTANCE};
use encoder::print_encoder_values;
use linux_embedded_hal::I2cdev;
use utilities::geometry::Angle;
const STEPS: usize = 200;

fn main() {
    /*
    let mut encoder = As5600::new(I2cdev::new("/dev/i2c-1").unwrap());
    print_encoder_values(&mut encoder);
    */
    
    let left_motor = Stepper::from_pin_nums(
        17,
        27,
        STEPS,
        2.0
         * PI * 0.067
    )
    .unwrap();
    let right_motor = Stepper::from_pin_nums(
        23,
        24,
        STEPS,
        2.0 * PI * 0.067
    )
    .unwrap();
    

    let mut drive = Drive::new(left_motor, right_motor, WHEEL_DISTANCE);

    let runtime = tokio::runtime::Builder::new_multi_thread().build().unwrap();
    runtime.block_on(async{
        loop{
            drive.go(Direction::Forward, 0.1).await;
            drive.turn(Angle::Radians(PI/2.0)).await;
        }
    }
    );
}
