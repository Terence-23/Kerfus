
#[allow(dead_code)]
pub mod drive{
    use std::time::Duration;
    use std::{thread, time, f32::consts::PI, ops::Neg};
    use crate::utilities::geometry::Angle;
    use as5600::As5600;
    use linux_embedded_hal::I2cdev;
    use rppal::pwm;

    use rppal::gpio::{Gpio,OutputPin, Error, InputPin};
    pub const STEP_TIME: u64 = 500;
    pub const WHEEL_DISTANCE: f32 = 0.1885;

    #[async_trait::async_trait]
    pub trait Motor {
        async fn rotate(&mut self, deg: Angle, dir: Direction);
        fn distance_to_angle(&self, dist: f32) -> Angle;
    }

    #[async_trait::async_trait]
    impl Motor for Brushless{
        async fn rotate(&mut self, deg: Angle, dir: Direction){
            //let steps: usize = (self.steps as f32 * deg.radians().abs() / (2.0 * PI)) as usize;
        }
        fn distance_to_angle(&self, dist: f32) -> Angle{
            Angle::Radians(dist * 2.0 * PI / self.circumference)
        }
    }
    pub struct Brushless{
        pwm: pwm::Pwm,
        pub data_pin: InputPin,
        pub circumference: f32,
    }
    impl Brushless {

        const PERIOD_MS: u64 = 20;
        const PULSE_MIN_US: u64 = 500;
        const PULSE_MAX_US: u64 = 2500;//to change

        
        pub fn new(ch: pwm::Channel) -> pwm::Result<Self> { 
            Ok(
                Self { 
                    pwm: pwm::Pwm::with_period(
                        ch,
                        Duration::from_millis(Brushless::PERIOD_MS),
                        Duration::from_micros(Brushless::PULSE_MAX_US),
                        pwm::Polarity::Normal,
                        true
                    )?,
                    circumference: 1.0,
                    data_pin: todo!(),
                    
                    
                } 
            )
        }
        pub fn speed(&mut self, a: f32) -> pwm::Result<()>{
            let add_val = (Brushless::PULSE_MAX_US - Brushless::PULSE_MIN_US) as f32 
            * a * std::f32::consts::FRAC_1_PI;
            let pulse: u64 = Brushless::PULSE_MIN_US + add_val as u64;
            println!("{}", pulse);
            self.pwm.set_pulse_width(Duration::from_micros(pulse))?;
            Ok(())
        }
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
        pub steps: usize,
        pub circumference: f32, 
    }
    impl Stepper{
        pub fn new(dir_pin: OutputPin, step_pin: OutputPin, steps:usize, circumference: f32) -> Self{
            Self{
                dir_pin,
                step_pin,
                steps,
                circumference
            }
        }
        pub fn from_pin_nums(dir_pin: u8, step_pin: u8, steps:usize, circumference: f32) -> Result<Self, Error>{
            Ok(Self{
                dir_pin: Gpio::new()?.get(dir_pin)?.into_output(),
                step_pin: Gpio::new()?.get(step_pin)?.into_output(),
                steps,
                circumference
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
            let steps: usize = (self.steps as f32 * deg.radians().abs() / (2.0 * PI)) as usize;
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
        fn distance_to_angle(&self, dist: f32) -> Angle{
            Angle::Radians(dist * 2.0 * PI / self.circumference)
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
    // use std::time::Instant;
    
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
             
            let left_angle = self.left_motor.distance_to_angle(distance);
            let lf = self.left_motor.rotate(left_angle, dir.to_owned());
            let right_angle = self.right_motor.distance_to_angle(distance);
            let rf = self.right_motor.rotate(right_angle, dir.to_owned());
            for f in vec![lf, rf]{
                f.await;
            }

        }
    
        pub async fn turn(& mut self, angle: Angle){
            let distance = angle.radians().abs() * self.distance / 2.0;
            let left = angle.radians() < 0.0;

            let l_dir = match left{
                true => Direction::Backward,
                false => Direction::Forward
            }; 
            let left_angle = self.left_motor.distance_to_angle(distance);
            let lf = self.left_motor.rotate(left_angle, l_dir);

            let r_dir = match left{
                true => Direction::Forward,
                false => Direction::Backward
            };
            let right_angle = self.right_motor.distance_to_angle(distance);            
            let rf = self.right_motor.rotate(right_angle, r_dir);

            for f in vec![lf, rf]{
                f.await;
            }
            println!("skręcił: {:?}, {:?}", right_angle, left_angle);
        }
    }

}
