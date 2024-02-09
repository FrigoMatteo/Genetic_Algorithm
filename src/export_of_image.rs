use robotics_lib::runner::Runnable;
use robotics_lib::world::tile::{Tile, TileType};
use robotics_lib::world::tile::TileType::{DeepWater, Grass, Hill, Lava, Mountain, Sand, ShallowWater, Snow, Street, Teleport};

use image::{Rgb, RgbImage};


pub (crate) fn export_to_image(map: &Vec<Vec<Option<Tile>>>, filename: &str, robot:&impl Runnable) {
    if map.len() < 0 {
        panic!("cannot save an empty image")
    } else {
        // TODO implement UI using Bevy
        let width = map.len();
        let height = map[0].len();

        let mut image = RgbImage::new(width as u32, height as u32);
        let pos=robot.get_coordinate();
        for x in 0..width {
            for y in 0..height {
                if x==pos.get_row() && y==pos.get_col(){
                    image.put_pixel(y as u32,x as u32, Rgb([255,0,0]));
                }else{
                    match map[x][y].as_ref() {
                        Some(tile) => {
                            let color = color_for_tile(tile.tile_type);
                            image.put_pixel(y as u32, x as u32, color);
                        }
                        Option::None => {
                            image.put_pixel(y as u32, x as u32, Rgb([0,0,0]));
                        }
                    }
                }
            }
        }
        // Save the image to a file
        image.save(filename).expect("Failed to save image");
    }
}

fn color_for_tile(tile: TileType) -> Rgb<u8> {
    // TODO Adjust colors
    match tile {
        DeepWater => Rgb([0, 0, 125]),
        Grass => Rgb([124, 252, 0]),
        Sand => Rgb([246, 215, 176]),
        ShallowWater => Rgb([35, 137, 218]),
        Mountain => Rgb([90, 75, 65]),
        Lava => Rgb([207, 16, 32]),
        Street => Rgb([50, 50, 50]),
        Sand => Rgb([0, 125, 0]),
        Snow=>Rgb([255,255,255]),
        Hill=>Rgb([1,50,32]),
        Teleport(_)=>Rgb([255,0,255]),
        _ => Rgb([0,0,0]),
    }
}