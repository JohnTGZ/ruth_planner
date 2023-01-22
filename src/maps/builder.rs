use std::collections::HashSet;
use std::path::Path;

use image::{GenericImageView, GrayImage};
use plotters::coord::types::RangedCoordf32;
use plotters::{coord::Shift, prelude::*};

use super::gridmap::*;
use crate::planners::planner_common::MotionPlan;

/// Builds a Gridmap struct from a PGM Image
pub fn build_gridmap_from_pgm(file_path: &str) -> Option<Gridmap> {
    let img = image::open(&Path::new(file_path)).unwrap();

    let (width, height) = img.dimensions();

    let mut gridmap = Gridmap::new(width, height, 0.05);

    let img_grayscale = img.as_luma8().unwrap();

    // 0 is black, 255 is white (free space)
    for (x, y, pixel) in img_grayscale.enumerate_pixels() {
        gridmap.set_val_xy(pixel.0[0], (x, y));
    }

    Some(gridmap)
}

/// Builds a gridmap struct from a 2d vector of u8 values
pub fn build_gridmap_from_2d_arr(arr_map: &Vec<Vec<u8>>) -> Option<Gridmap> {
    if arr_map.len() == 0 || arr_map[0].len() == 0 {
        return None;
    }

    let (width, height) = (arr_map[0].len() as u32, arr_map.len() as u32);

    let mut gridmap = Gridmap::new(width, height, 0.05);

    for y in 0..height {
        for x in 0..width {
            gridmap.set_val_xy(arr_map[y as usize][x as usize], (x, y));
        }
    }

    Some(gridmap)
}

/// Saves gridmap as PGM file
pub fn save_gridmap(gridmap: &Gridmap, file_path: &str) -> () {
    let (width, height) = (
        gridmap.get_width(),
        gridmap.get_height(),
    );

    let mut imgbuf = GrayImage::new(width, height);

    for (x, y, pixel) in imgbuf.enumerate_pixels_mut() {
        *pixel = image::Luma([gridmap.get_val_xy((x, y))]);
    }

    imgbuf.save(&Path::new(file_path)).unwrap();
}

pub fn plot_gridmap<'a>(
    gridmap: &'a Gridmap,
    start_cell: &'a (u32, u32),
    goal_cell: &'a (u32, u32),
    motion_plan: &'a MotionPlan,
    file_path: &'a str,
) -> () {
    let (width, height, resolution) = (
        gridmap.get_width(),
        gridmap.get_height(),
        gridmap.get_resolution(),
    );

    let aspect_ratio = (width as f32) / (height as f32);
    let img_width: u32 = 640;
    let img_height: u32 = (640.0 / aspect_ratio) as u32;

    let root = BitMapBackend::new(&file_path, (img_width, img_height)).into_drawing_area();

    root.fill(&WHITE).unwrap();

    let mut chart = ChartBuilder::on(&root)
        .margin(5)
        .set_label_area_size(LabelAreaPosition::Left, 40)
        .set_label_area_size(LabelAreaPosition::Bottom, 40)
        .build_cartesian_2d(
            0.0..(width as f32 * resolution),
            0.0..(height as f32 * resolution),
        )
        .unwrap();

    chart
        .configure_mesh()
        .x_max_light_lines(1)
        .y_max_light_lines(1)
        .draw()
        .unwrap();

    plot_obstacles(&mut chart, gridmap);
    plot_start_and_goal(&mut chart, gridmap, start_cell, goal_cell);
    plot_closed_list(&mut chart, gridmap, &motion_plan.closed_list);
    plot_path(&mut chart, gridmap, &motion_plan.path);

    // Legend
    // chart.configure_series_labels()
    //   .border_style(&BLACK)
    //   .background_style(&WHITE.mix(0.8))
    //   .draw()
    //   .unwrap();

    root.present()
        .expect("Unable to write result to file, please make sure directory exists");
}

/// Plot Obstacles (grayscale)
pub fn plot_obstacles(
    chart: &mut ChartContext<BitMapBackend, Cartesian2d<RangedCoordf32, RangedCoordf32>>,
    gridmap: &Gridmap,
) {
    chart
        .draw_series(
            gridmap
                .get_cells()
                .iter()
                .enumerate()
                .map(|(idx, cell_val)| (gridmap.idx_to_xy(idx as u32), cell_val))
                .map(|(xy, cell_val)| get_obs_rect(&gridmap, &xy, cell_val)),
        )
        .unwrap();
}

/// Plot Start and goal cell
pub fn plot_start_and_goal(
    chart: &mut ChartContext<BitMapBackend, Cartesian2d<RangedCoordf32, RangedCoordf32>>,
    gridmap: &Gridmap,
    start_cell: &(u32, u32),
    goal_cell: &(u32, u32),
) {
    chart
        .draw_series([Circle::new(
            get_cell_centroid(gridmap, start_cell),
            20,
            CYAN.filled(),
        )])
        .unwrap();
    chart
        .draw_series([Circle::new(
            get_cell_centroid(gridmap, goal_cell),
            20,
            GREEN.filled(),
        )])
        .unwrap();
}

/// Plot Closed List (visited cells)
pub fn plot_closed_list(
    chart: &mut ChartContext<BitMapBackend, Cartesian2d<RangedCoordf32, RangedCoordf32>>,
    gridmap: &Gridmap,
    closed_list: &HashSet<(u32, u32)>,
) {
    chart
        .draw_series(closed_list.iter().map(|xy| {
            Circle::new(
                get_cell_centroid(gridmap, xy),
                20,
                RGBAColor(255, 255, 0, 0.5).filled(),
            )
        }))
        .unwrap();
}

/// Plot points from the final path    
pub fn plot_path(
    chart: &mut ChartContext<BitMapBackend, Cartesian2d<RangedCoordf32, RangedCoordf32>>,
    gridmap: &Gridmap,
    path: &Vec<(u32, u32)>,
) {
    chart
        .draw_series(
            LineSeries::new(
                path.iter().map(|xy| get_cell_centroid(gridmap, xy)),
                BLUE.filled(),
            )
            .point_size(10),
        )
        .unwrap();
}

/// Get cell centroid in plotter coordinate system
pub fn get_cell_centroid(gridmap: &Gridmap, xy: &(u32, u32)) -> (f32, f32) {
    gridmap.get_cell_centroid((xy.0, gridmap.get_height() - 1 - xy.1))
}

/// Construct rectangle for obstacle cell
pub fn get_obs_rect(
    gridmap: &Gridmap,
    xy: &(u32, u32),
    cell_val: &u8,
) -> plotters::element::Rectangle<(f32, f32)> {
    let (x_f, y_f) = get_cell_centroid(gridmap, xy);
    let cell_offset = gridmap.get_resolution() / 2.0;

    Rectangle::new(
        [
            (x_f - cell_offset, y_f - cell_offset),
            (x_f + cell_offset, y_f + cell_offset),
        ],
        RGBAColor(255 - cell_val, 255 - cell_val, 255 - cell_val, 0.75).filled(),
    )
}
