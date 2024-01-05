#[allow(dead_code)]
pub mod wall_detection {
    use image::{DynamicImage, ImageBuffer, Rgb};

    use crate::utilities::{
        geometry::{Interval, Vec2},
        maxf, minf,
    };

    #[derive(Clone, Copy)]
    pub struct Cell {
        x_span: Interval,
        y_span: Interval,
        point_count: usize,
    }

    pub struct DynGrid {
        data: Vec<Vec<Cell>>,
        x_size: usize,
        y_size: usize,
        x_span: f32,
        y_span: f32,
    }
    impl DynGrid {
        fn new(
            x_size: usize,
            y_size: usize,
            x_span: f32,
            y_span: f32,
            x_start: f32,
            y_start: f32,
        ) -> DynGrid {
            let int = Interval::new(0.0, 0.0);
            let mut data = vec![
                vec![
                    Cell {
                        x_span: int,
                        y_span: int,
                        point_count: 0
                    };
                    y_size
                ];
                x_size
            ];

            for x in 0..x_size {
                for y in 0..y_size {
                    let x_s = x_start + x as f32 * x_span;
                    let y_s = y_start + y as f32 * y_span;

                    data[x][y].x_span = Interval::new(x_s, x_s + x_span);
                    data[x][y].y_span = Interval::new(y_s, y_s + y_span);
                }
            }

            Self {
                data,
                x_size,
                y_size,
                x_span,
                y_span,
            }
        }
    }

    pub struct FloorGrid {
        grid: DynGrid,
        threshold: usize,
    }
    impl FloorGrid {
        /// points: point cloud for the grid
        /// cell_count: number of cells in one dimention; actual ammount of cells = cell_count^2
        pub fn new(points: &[Vec2<f32>], cell_count: usize) -> Self {
            let mut min_x = points[0].x;
            let mut min_y = points[0].y;
            let mut max_x = points[0].x;
            let mut max_y = points[0].y;

            for p in &points[1..] {
                min_x = minf(p.x, min_x);
                min_y = minf(p.y, min_y);

                max_x = maxf(p.x, max_x);
                max_y = maxf(p.y, max_y);
            }

            let cell_span_x = (max_x - min_x + 1.0) / cell_count as f32;
            let cell_span_y = (max_y - min_y + 1.0) / cell_count as f32;

            let grid = DynGrid::new(
                cell_count,
                cell_count,
                cell_span_x,
                cell_span_y,
                min_x,
                min_y,
            );

            Self::new_with_grid(grid, points)
        }
        pub fn new_with_grid(mut grid: DynGrid, points: &[Vec2<f32>]) -> Self {
            //assign each point to a cell
            let mut max = 0;
            for p in points {
                let x = (p.x / grid.x_span).floor() as usize;
                let y = (p.y / grid.y_span).floor() as usize;
                grid.data[x][y].point_count += 1;
                if max < grid.data[x][y].point_count {
                    max = grid.data[x][y].point_count
                }
            }

            Self {
                grid,
                threshold: max / 10,
            }
        }

        pub fn into_image(&self) -> DynamicImage {
            let mut img = ImageBuffer::new(self.grid.x_size as u32, self.grid.y_size as u32);

            for (x, y, pix) in img.enumerate_pixels_mut() {
                let cell = self.grid.data[y as usize][x as usize];
                *pix = if cell.point_count > self.threshold {
                    Rgb::<u8> { 0: [0, 0, 0] }
                } else {
                    Rgb::<u8> { 0: [255, 255, 255] }
                }
            }

            img.into()
        }
    }

    pub struct NavGraph {}
    impl NavGraph {
        pub fn new_from_grid(grid: FloorGrid, dest: Vec2<f32>) -> NavGraph {
            let (_, _) = (grid, dest);
            todo!()
        }
    }
}
