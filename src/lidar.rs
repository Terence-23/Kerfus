
use crate::utilities::geometry;

#[allow(dead_code)]
mod communication{
    use std::{error::Error, fmt::Display, time::Duration};

    use rppal::uart;

    
    #[derive(Debug)]
    enum LidarError{
        BadLength,
        BadChecksum,
        WeakSignal,
        UartError(uart::Error),
        BadStart
    }

    impl Display for LidarError {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            
            return write!(f, "{}", match self {
                LidarError::BadLength => "a packet of wrong length was recieved".to_owned(),
                LidarError::BadChecksum => "a packet with wrong checksum was recieved".to_owned(),
                LidarError::WeakSignal => "lidar signal is not good enough for proper reading".to_owned(),
                LidarError::BadStart => "first bytes of a recieved data package were improper".to_owned(),
                LidarError::UartError(e) => format!("uart has returned an error: {}", e).to_owned(),
            })
        }
    }

    impl Error for LidarError {
        fn source(&self) -> Option<&(dyn Error + 'static)> {
            match self{
                Self::UartError(e) => Some(e),
                _ => None
            }
        }

    }
    struct LIDAR{
        pub uart: uart::Uart,
    }

    impl LIDAR {
        const DATA_LENGTH: usize = 9;
        const RESET_SUCCESS: [u8; 5] = [0x5a, 0x05, 0x10, 0x00, 0x6e];
        const SAVE_SUCCESS: [u8; 5] = [0x5a, 0x05, 0x11, 0x00, 0x6f];

        fn new(uart: uart::Uart) -> Self { Self { uart } }
        
        pub fn read_point(&mut self) -> Result<f32, LidarError>{

            let send_buf:[u8; 0x04] = [0x5a, 0x04, 0x04, 0x62];
            match self.uart.write(&send_buf){
                Ok(_) => (),
                Err(e) => return Err(LidarError::UartError(e))
            }
            let mut buf = [0 as u8 ; LIDAR::DATA_LENGTH];
            
            let len = self.uart.read(&mut buf);
            match len{
                Ok(n) => if n != LIDAR::DATA_LENGTH {return Err(LidarError::BadLength)},
                Err(e) => return Err(LidarError::UartError(e)),
            }
            if buf[0] != 0x59 || buf[1] != 0x59{
                return Err(LidarError::BadStart);
            } 
            //check checksum
            if buf[8] != (buf.map(|u| u as u16).iter().sum::<u16>() & 255) as u8{
                return Err(LidarError::BadChecksum);
            }
            let signal: u16 = buf[4] as u16 + (buf[5] as u16) << 8;
            if signal < 100 || signal == 65535{
                return Err(LidarError::WeakSignal);
            }

            let distance = buf[2] as u16 + (buf[3] as u16) << 8;

            Ok(distance as f32)
        }
        pub fn configure(&mut self) -> uart::Result<()>{

            self.uart.set_write_mode(true)?;
            self.uart.set_baud_rate(115200)?;
            self.uart.set_read_mode(5, Duration::from_secs(0))?;

            let factory_settings_packet= [0x5a, 0x04, 0x10, 0x6e as u8];
            self.uart.write(&factory_settings_packet)?;
            let mut response = [0 as u8; 5];
            self.uart.read(&mut response)?;
            if response != LIDAR::RESET_SUCCESS{
                return Err(uart::Error::InvalidValue)
            }

            let frame_rate_packet = [0x5a, 0x06, 0x03, 0, 0, ((0x5a + 0x06 + 0x03) & 255) as u8 ];
            self.uart.write(&frame_rate_packet)?;
            let mut fr_response = [0 as u8; 5];
            self.uart.read(&mut fr_response)?;
            if !fr_response[3] == 0 && !fr_response[4] == 0{
                return Err(uart::Error::InvalidValue);
            }

            let save_settings_packet= [0x5a, 0x04, 0x11, 0x6f as u8];
            self.uart.write(&save_settings_packet)?;
            self.uart.read(&mut response)?;
            if response != LIDAR::SAVE_SUCCESS{
                return Err(uart::Error::InvalidValue)
            }

            self.uart.set_read_mode(9, Duration::from_secs(0))?;
            Ok(())
        }
    }
    
}
pub mod control{

}

#[allow(dead_code)]
pub mod math{
    use std::iter::Iterator;

    use super::geometry::{Vec2, Line, Segment};

    pub enum SegmentType{
        Interpolated(Segment),
        Measured(Segment)
    }
    pub struct Wall{
        pub segments: Vec<SegmentType>
    }

    type Walls = Vec<Wall>;
    const SLOPE_DIFF: f32 = 0.1;
    const DISTANCE: f32 = 5.0;

    pub fn find_walls(points: &[Vec2<f32>]) -> Walls{
        
        let segments = create_segments(points);
        let walls = connect_segemnts(segments);
        
        return walls;
    }
    
    fn connect_segemnts(segments: Vec<Segment>) -> Walls{

        let mut segments = segments.to_owned();
        let mut walls = Walls::new();
        while !segments.is_empty() {
            let segment = segments[0];
            let line: Line = segment.into();
            let mut wall = Wall{
                segments: vec![
                    SegmentType::Measured(segment.to_owned())
                    ]
                };
            let mut remove = vec![0];

            let mut i = 1;
            while i < segments.len(){
                if line.is_close_to_collinear(&segments[i].into(), SLOPE_DIFF, DISTANCE){
                    if let Some(SegmentType::Measured(segment)) = wall.segments.last(){
                        wall.segments.push(
                            SegmentType::Interpolated(
                                Segment { p1: segment.p2, p2: segments[i].p1 }
                            )
                        );
                    }
                    wall.segments.push(SegmentType::Measured(segments[i].to_owned()));
                    remove.push(i);
                }
                i+=1;
            }
            i = 0;
            let mut r_i = 0;
            segments.retain(|_| {
                if i == remove[r_i]{
                    i+=1;
                    r_i +=1;
                    false
                }else{
                    i+=1;
                    true
                }
            });
            walls.push(wall);
        }
        walls
    }
    
    fn create_segments(points: &[Vec2<f32>]) -> Vec<Segment>{
        let mut segments: Vec<Segment> = vec![];
        if points.len() < 2{
            return segments;
        }
        let mut left = 0;
        let mut right = 1;

        while right < points.len(){
            
            let test_segment = Segment::new(points[left], points[right]);
            for p in points[left+1..right].into_iter(){
                let distance = test_segment.distance(*p);
                let len = p.length();
                if (len > 600.0 && distance > len * 0.01) || (len < 600.0 && (distance - len).abs() < 5.0) {
                    segments.push(Segment::new(points[left], points[right - 1]));
                    left = right;
                    break;
                } 
            }
            right  += 1;
        }

        if left != right && segments.last().unwrap_or(&Segment::new(points[0], points[0])).p2 != points[right]{
            segments.push(Segment::new(points[left], points[right]));
        }
        
        segments
    }

    fn least_square_method<I>(it: I) -> Option<Line> where I : Iterator<Item = Vec2<f32>> + Clone {
        let (sumx , sumy, count) =
        it.clone().fold((0.0, 0.0, 0_isize), |(sx,sy,ct), v| {
            let (x, y) = v.into();
            (sx + x, sy + y, ct + 1)
        });
        if count < 2 { return None; }
        let (meanx, meany) = (sumx / count as f32, sumy / count as f32);
        let (_sxx,_sxy) =
            it.fold((0.0,0.0), |(sxx,sxy), v| {
                let (x,y) = v.into();
                let (dx, dy) = (x - meanx, y - meany);
                (sxx + dx*dx, sxy + dx*dy)
            });
        
    
        todo!()
    }
    
}