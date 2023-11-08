
#[allow(dead_code)]
pub mod drive{
    use std::{thread, time, f32::consts::PI, ops::Neg};
    use crate::utilities::geometry::Angle;

    use rppal::gpio::{Gpio,OutputPin, Error};
    pub const STEP_TIME: u64 = 500;
    pub const WHEEL_DISTANCE: f32 = 0.1885;

    #[async_trait::async_trait]
    pub trait Motor {
        async fn rotate(&mut self, deg: Angle, dir: Direction);
    }


    #[derive(Clone, Debug)]
    pub enum Direction{
        Forward,
        Backward
    }
    impl Neg for Direction{
        type Output = Self;

        fn neg(self) -> Self {
            match self{
                Direction::Forward => Self::Backward,
                Direction::Backward => Self::Forward,
            }
        }
    }
    pub struct Stepper{
        pub dir_pin: OutputPin,
        pub step_pin: OutputPin,
        pub steps: usize 
    }
    impl Stepper{
        pub fn new(dir_pin: OutputPin, step_pin: OutputPin, steps:usize) -> Self{
            Self{
                dir_pin,
                step_pin,
                steps
            }
        }
        pub fn from_pin_nums(dir_pin: u8, step_pin: u8, steps:usize) -> Result<Self, Error>{
            Ok(Self{
                dir_pin: Gpio::new()?.get(dir_pin)?.into_output(),
                step_pin: Gpio::new()?.get(step_pin)?.into_output(),
                steps
            })
        }
        pub fn forward(&mut self, steps: Option<usize>){
            self.dir_pin.set_high();
            for _ in 0..steps.unwrap_or(1){
                self.step_pin.set_high();
                thread::sleep(time::Duration::from_micros(STEP_TIME));
                self.step_pin.set_low();
                thread::sleep(time::Duration::from_micros(STEP_TIME));
            }
        }
        pub fn backward(&mut self, steps: Option<usize>){
            self.dir_pin.set_low();
            for _ in 0..steps.unwrap_or(1){
                self.step_pin.set_high();
                thread::sleep(time::Duration::from_micros(STEP_TIME));
                self.step_pin.set_low();
                thread::sleep(time::Duration::from_micros(STEP_TIME));
            }
        }
    }

    #[async_trait::async_trait]
    impl Motor for Stepper{
        async fn rotate(&mut self, deg: Angle, dir: Direction){
            let steps: usize = (self.steps as f32 * deg.radians() / (2.0 * PI)) as usize;
            match dir {
                Direction::Forward => {
                    for _ in 0..steps{
                        self.forward(Some(1));
                    }
                },
                Direction::Backward => {
                    for _ in 0..steps{
                        self.backward(Some(1));
                    }
                },
            }
        }
    }


    pub struct Drive<T1, T2> 
    where T1: Motor, T2: Motor
    {
        left_motor: T1,
        right_motor: T2,
        r1: bool,
        r2: bool,
        distance: f32,
    }
    use std::time::Instant;
    
    impl<T1, T2> Drive<T1, T2> 
    where T1: Motor, T2: Motor{
        
        pub fn new(left_motor: T1, right_motor: T2, distance: f32) -> Self{
            Self{
                left_motor,
                right_motor,
                r1: false,
                r2: false,
                distance
            }
        }

        pub async fn go(& mut self, dir: Direction, distance: f32){
            

        }
    
        pub async fn turn(& mut self, angle: Angle){
            let distance = angle.radians().abs() * WHEEL_DISTANCE / 2.0;
            


            // println!("skręcił: {}, {}", r_steps, l_steps);
        }
    }

}
