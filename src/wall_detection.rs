
#[allow(dead_code)]
mod wall_detection{
    use crate::utilities::geometry::{Interval, Vec2};
    
    #[derive(Clone, Copy)]
    struct Cell {
        x_span: Interval,
        y_span: Interval,
        point_count: usize
    }

    struct DynGrid{
        data: Vec<Vec<Cell>>,
        x_size: usize,
        y_size: usize,
        x_span: f32,
        y_span: f32,
    }
    impl DynGrid{
        fn new(x_size: usize, y_size: usize, x_span: f32, y_span: f32, x_start: f32, y_start: f32) -> DynGrid{
            let int = Interval::new(0.0, 0.0);
            let mut data = vec![
                vec![
                    Cell{
                        x_span: int, 
                        y_span: int,
                        point_count: 0
                    }
                    ; y_size]
                ; x_size];
            
            for x in 0..x_size{
                for y in 0..y_size{
                    let x_s = x_start + x as f32 * x_span;
                    let y_s  = y_start + y as f32 * y_span;

                    data[x][y].x_span = Interval::new(x_s, x_s + x_span);
                    data[x][y].y_span = Interval::new(y_s, y_s + y_span);
                    
                }
            }

            Self{
                data,
                x_size,
                y_size,
                x_span,
                y_span
            }

        }
    }
    
    struct FloorGrid{
        grid: DynGrid,
        threshold: usize,

    }
    impl FloorGrid{
        fn new(mut grid: DynGrid, points: &[Vec2<f32>]) -> Self{
            //assign each point to a cell
            let mut max = 0;
            for p in points{
                let x = (p.x / grid.x_span).floor() as usize;
                let y = (p.y / grid.y_span).floor() as usize;
                grid.data[x][y].point_count +=1;
                if max < grid.data[x][y].point_count{
                    max = grid.data[x][y].point_count
                }
            }

            Self{
                grid,
                threshold: max /4
            }
        }
    }

    struct NavGraph{

    }
    impl NavGraph{
        pub fn new_from_grid(grid : FloorGrid, dest: Vec2<f32>) -> NavGraph{
            todo!()
        }
    }
    
}