use crate::InputDir;
use crate::ONE_DIRECTION_DISTANCE;
use crate::DISTANCE;
use crate::PositionToGo;
use crate::WORLD_SIZE;
use robotics_lib::world::tile::{Tile, TileType};
use robotics_lib::world::tile::TileType::{DeepWater, Lava};



pub(crate) fn direction_value(value:&InputDir)->(i32,i32){
    match value{
        InputDir::None=>(0,0),
        InputDir::Top(_)=>(-1,0),
        InputDir::Right(_)=>(0,1),
        InputDir::Left(_)=>(0,-1),
        InputDir::Bottom(_)=>(1,0),
    }
}

pub(crate) fn get_next_position(pos:PositionToGo)->(i32,i32){
    match pos{
        PositionToGo::Down => (ONE_DIRECTION_DISTANCE as i32,0),
        PositionToGo::DownRight => (DISTANCE as i32,DISTANCE as i32),
        PositionToGo::Right => (0,ONE_DIRECTION_DISTANCE as i32),
        PositionToGo::TopRight => (-(DISTANCE as i32),DISTANCE as i32),
        PositionToGo::Top => (-(ONE_DIRECTION_DISTANCE as i32),0),
        PositionToGo::TopLeft => (-(DISTANCE as i32),-(DISTANCE as i32)),
        PositionToGo::Left => (0,-(ONE_DIRECTION_DISTANCE as i32)),
        PositionToGo::DownLeft => (DISTANCE as i32,-(DISTANCE as i32))
    }
}

pub(crate) fn is_good_tile(data:&Option<Tile>) ->bool{
    if data.is_none(){
        false
    }else{
        match data.as_ref().unwrap().tile_type{
            DeepWater => false,
            Lava => false,
            TileType::Wall => false,
            _=>true
        }
    }
}

pub(crate) fn is_not_visualize(next_x:i32, next_y:i32) ->bool{
    if next_y>=WORLD_SIZE as i32|| next_x>=WORLD_SIZE as i32 || next_x<0 || next_y<0 {
        true
    }else{
        false
    }
}