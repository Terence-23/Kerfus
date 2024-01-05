use crate::utilities::geometry::Angle;
use as5600::As5600;
use linux_embedded_hal::I2cdev;
use std::{f32::consts::PI, time::Duration};

// Assume Angle is defined in the same module as the Brushless motor

pub fn print_encoder_values(encoder: &mut As5600<I2cdev>) {
    
    loop {
        // Get the current angle from the magnetic encoder
        let current_angle = encoder.angle().unwrap() as f32 * (360.0 / 0x0FFF as f32);

        // Print the current angle to the screen
        println!("Current Angle: {:.2} degrees", current_angle);

        // Optionally, add a delay to control the update rate
        std::thread::sleep(Duration::from_millis(100));
    }
}
