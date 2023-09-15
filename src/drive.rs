
#[allow(dead_code)]
pub mod drive{
    use std::{thread, time, f32::consts::PI};

    use rppal::gpio::{Gpio,OutputPin, Error};
    pub const STEP_TIME: u64 = 5;
    pub const WHEEL_DISTANCE: f32 = 0.1885;
    #[derive(Clone, Debug)]
    pub enum Direction{
        Forward,
        Backward
    }
    #[derive(PartialOrd, Clone, Copy, Debug)]
    pub enum Angle{
        Degrees(f32),
        Radians(f32)
    }

    impl PartialEq for Angle {
        fn eq(&self, other: &Self) -> bool {
            match (self, other) {
                (Self::Degrees(l0), Self::Degrees(r0)) => l0 == r0,
                (Self::Radians(l0), Self::Radians(r0)) => l0 == r0,
                (l0, l1) => l0.radians() == l1.radians(),
            }
        }
    }
    impl Angle{
        fn radians(&self) -> f32{
            match self{
                Angle::Degrees(n) => n * PI / 180.0,
                Angle::Radians(n) => *n,
            }
        }
        fn degrees(&self) -> f32{
            match self {
                Angle::Degrees(n) => *n,
                Angle::Radians(n) => n/ PI * 180.0,
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
                thread::sleep(time::Duration::from_millis(STEP_TIME));
                self.step_pin.set_low();
                thread::sleep(time::Duration::from_millis(STEP_TIME));
            }
        }
        pub fn backward(&mut self, steps: Option<usize>){
            self.dir_pin.set_low();
            for _ in 0..steps.unwrap_or(1){
                self.step_pin.set_high();
                thread::sleep(time::Duration::from_millis(STEP_TIME));
                self.step_pin.set_low();
                thread::sleep(time::Duration::from_millis(STEP_TIME));
            }
        }
    }

    
    pub struct Wheel{
        pub motor: Stepper,
        pub gear_ratio: f32,
        pub circumference: f32,
        pub reverse: bool,
    }

    impl Wheel {
        pub fn new(stepper: Stepper, gear_ratio: f32, circumference: f32, reverse: bool) -> Self{
            Self{
                motor: stepper, gear_ratio, circumference: circumference.abs(), reverse
            }
        }  
        pub fn new_from_diameter(stepper: Stepper, gear_ratio: f32, diameter: f32, reverse: bool) -> Self {
            Self{
                motor: stepper, gear_ratio, circumference: (diameter * 2.0 * PI).abs(), reverse
            }
        } 
        pub fn move_wheel(&mut self, dir: Direction, distance: f32){
            let steps: usize = (distance / self.circumference / self.gear_ratio * self.motor.steps as f32).abs() as usize;
            
            let m_dir = if self.reverse{
                match dir {
                    Direction::Backward => Direction::Forward,
                    Direction::Forward => Direction::Backward
                }
            } else {
                dir
            };

            match m_dir {
                Direction::Forward => self.motor.forward(Some(steps)),
                Direction::Backward => self.motor.backward(Some(steps))
            }
        }
        pub fn steps_and_dir(&self, dir: Direction, distance: f32) -> (usize, Direction){
            let steps: usize = (distance / self.circumference / self.gear_ratio * self.motor.steps as f32).abs() as usize;
            
            let m_dir = if self.reverse{
                match dir {
                    Direction::Backward => Direction::Forward,
                    Direction::Forward => Direction::Backward
                }
            } else {
                dir
            };
            return (steps, m_dir);
        }


    }

    pub struct Drive{
        left_wheel: Wheel,
        right_wheel: Wheel,
        distance: f32,
    }
    impl Drive {
        
        pub fn new(left_wheel: Wheel, right_wheel: Wheel, distance: f32) -> Self{
            Self{
                left_wheel,
                right_wheel,
                distance
            }
        }

        pub fn go(& mut self, dir: Direction, distance: f32){
            let l_dir; let l_steps;
            (l_steps, l_dir) = self.left_wheel.steps_and_dir(dir.to_owned(), distance);

            let r_dir; let r_steps;
            (r_steps, r_dir) = self.right_wheel.steps_and_dir(dir.to_owned(), distance);
            
            let r_inv_step = 1 as f32/r_steps as f32;
            let l_inv_step = 1 as f32/l_steps as f32;

            let mut r_steps_done = 0.0;
            let mut l_steps_done = 0.0;
            
            while r_steps_done < 1.0 || l_steps_done < 1.0{
                if r_steps_done < l_steps_done{
                    match r_dir{
                        Direction::Forward => self.right_wheel.motor.forward(Some(1)),
                        Direction::Backward => self.right_wheel.motor.backward(Some(1))
                    }
                    r_steps_done += r_inv_step;
                }
                else{
                    match l_dir{
                        Direction::Forward => self.left_wheel.motor.forward(Some(1)),
                        Direction::Backward => self.left_wheel.motor.backward(Some(1))
                    }
                    l_steps_done += l_inv_step;
                }
            }


        }
    
        pub fn turn(& mut self, angle: Angle){
            let distance = angle.radians().abs() * WHEEL_DISTANCE / 2.0;
            let left = angle.radians() < 0.0;
            let l_dir; let l_steps;
            (l_steps, l_dir) = self.left_wheel
            .steps_and_dir(
                match left{
                    true => Direction::Backward,
                    false => Direction::Forward
                },
                distance
            );

            let r_dir; let r_steps;
            (r_steps, r_dir) = self.right_wheel
            .steps_and_dir(match left{
                true => Direction::Forward,
                false => Direction::Backward
            }, distance);
            
            let r_inv_step = 1 as f32/r_steps as f32;
            let l_inv_step = 1 as f32/l_steps as f32;

            let mut r_steps_done = 0.0;
            let mut l_steps_done = 0.0;
            
            while r_steps_done < 1.0 || l_steps_done < 1.0{
                if r_steps_done < l_steps_done{
                    match r_dir{
                        Direction::Forward => self.right_wheel.motor.forward(Some(1)),
                        Direction::Backward => self.right_wheel.motor.backward(Some(1))
                    }
                    r_steps_done += r_inv_step;
                }
                else{
                    match l_dir{
                        Direction::Forward => self.left_wheel.motor.forward(Some(1)),
                        Direction::Backward => self.left_wheel.motor.backward(Some(1))
                    }
                    l_steps_done += l_inv_step;
                }
            }
        }
    }

}
