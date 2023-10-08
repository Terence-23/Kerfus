
#[allow(dead_code)]
pub mod geometry{

    use num::Float;
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct Vec2<T>{
        pub x: T,
        pub y: T
    }
    impl<T> Vec2<T>{
        pub fn new(x: T, y: T) -> Vec2<T>{
            Self{
                x, 
                y
            }
        }
        pub fn from_polar(len: f32, angle: f32) -> Vec2<f32>{
            Vec2::<f32>{ 
                x: angle.cos() * len,
                y: angle.sin() * len 
            }
        }
    }
    impl<T> From<(T,T)> for Vec2<T>{
        fn from(value: (T,T)) -> Self {
            Self { x: value.0, y: value.1 }
        }
    }
    impl<T> From<Vec2<T>> for (T,T){
        fn from(val: Vec2<T>) -> Self {
            (val.x, val.y)
        }
    }

    impl<T> Vec2<T> where T: Float {
        pub fn length(&self) -> T{
            (self.x * self.x + self.y * self.y).sqrt()
        }
    }
    ///Line using ax + by + c = 0 equation
    pub struct Line{
        a: f32,
        b: f32,
        c: f32
    }
    impl Line{
        pub fn new(a: f32, b: f32, c: f32) -> Line{
            Self{
                a, b, c
            }
        }
        pub fn from2(a: f32, b: f32) -> Line{
            Self { a: a, b: -1.0 , c: b }
        }
        pub fn distance(&self, p:  Vec2<f32>) -> f32{
            (self.a * p.x + self.b * p.y + self.c).abs() / 
            (self.a * self.a + self.b * self.b).sqrt()
        }
        pub fn is_parallel(&self, other: &Line) -> bool {
            // Check if both lines are vertical (b1 and b2 are zero)
            if self.b == 0.0 && other.b == 0.0 {
                return true;
            }
            // Check if both lines are parallel but not vertical (a1/a2 == b1/b2)
            else if self.b != 0.0 && other.b != 0.0 && (self.a / self.b) == (other.a / other.b) {
                return true;
            }
            else{
                return false;
            }
        }
        pub fn is_close_to_parallel(&self, other: &Line, tolerance: f32) -> bool {
            if self.is_parallel(other){ true}
            else if self.b == 0.0 || other.b == 0.0{
                (self.b + other.b).abs() < tolerance
            }else{
                (self.a / self.b - other.a / other.b).abs() <= tolerance
            }
        }

        pub fn is_close_to_collinear(&self, other: &Line, slope_diff: f32, distance: f32) -> bool{
            self.is_close_to_parallel(other, slope_diff) && (self.c - other.c).abs() <= distance
        }
    }


    #[derive(Clone, Copy, PartialEq, Debug)]
    pub struct Segment{
        pub p1: Vec2<f32>,
        pub p2: Vec2<f32>
    } 
    impl Segment{
        pub fn new(p1: Vec2<f32>, p2: Vec2<f32>) -> Self { Self { p1, p2 } }
        pub fn distance(&self, p: Vec2<f32>) -> f32{
            Line::from(*self).distance(p)
        }
    }
    impl From<Segment> for Line{
        fn from(v: Segment) -> Self {
            Self { 
                a: v.p1.y - v.p2.y, 
                b: v.p2.x - v.p1.x, 
                c: v.p1.x * (v.p2.y - v.p1.y) - v.p1.y * (v.p2.x - v.p1.x) 
            }
        }
    }
    impl From<&Segment> for Line{
        fn from(v: &Segment) -> Self {
            Self { 
                a: v.p1.y - v.p2.y, 
                b: v.p2.x - v.p1.x, 
                c: v.p1.x * (v.p2.y - v.p1.y) - v.p1.y * (v.p2.x - v.p1.x) 
            }
        }
    }
}