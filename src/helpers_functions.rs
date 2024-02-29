
use std::sync::Arc;
use crate::{ALREADY_VISITED, CONTENT, InputDir, MovesToFollow};
use crate::ONE_DIRECTION_DISTANCE;
use crate::DISTANCE;
use crate::PositionToGo;
use crate::WORLD_SIZE;
use robotics_lib::world::tile::{Tile, TileType};
use robotics_lib::world::tile::TileType::{DeepWater, Lava};
use crate::genetic_algorithm::genetic_cost;


pub(crate) fn direction_value(value:&InputDir)->(i32,i32){
    match value{
        InputDir::None=>(0,0),
        InputDir::Top(_,_)=>(-1,0),
        InputDir::Right(_,_)=>(0,1),
        InputDir::Left(_,_)=>(0,-1),
        InputDir::Bottom(_,_)=>(1,0),
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
    if next_y>WORLD_SIZE as i32|| next_x>WORLD_SIZE as i32 || next_x<0 || next_y<0 {
        true
    }else{
        false
    }
}

pub (crate) fn already_visited(x:i32,y:i32)->bool{
    if ALREADY_VISITED.lock().unwrap()[x as usize][y as usize]{
        true
    }else{
        false
    }
}


pub (crate) fn calculate_cost_dir(moves:&MovesToFollow,mut x:usize,mut y:usize,map:&Vec<Vec<Option<Tile>>>)->usize{

    let mut next_x=x as i32;
    let mut next_y=y as i32;

    let mut cost:usize=0;


    for ele in moves.path_to_follow.iter(){
        let (i,j)=direction_value(ele);

        next_x=x as i32+i;
        next_y=y as i32+j;

        if *ele==InputDir::None || map[next_x as usize][next_y as usize].is_none(){continue;}


        match ele{
            InputDir::Right(_, true) => {
                let c=CONTENT.lock().unwrap().content.clone();
                let quantity=CONTENT.lock().unwrap().quantity;
                cost=cost+(c.properties().cost()*quantity);
            }
            InputDir::Left(_, true) => {
                let c=CONTENT.lock().unwrap().content.clone();
                let quantity=CONTENT.lock().unwrap().quantity;
                cost=cost+(c.properties().cost()*quantity);
            }
            InputDir::Top(_, true) => {
                let c=CONTENT.lock().unwrap().content.clone();
                let quantity=CONTENT.lock().unwrap().quantity;
                cost=cost+(c.properties().cost()*quantity);
            }
            InputDir::Bottom(_, true) => {
                let c=CONTENT.lock().unwrap().content.clone();
                let quantity=CONTENT.lock().unwrap().quantity;
                cost=cost+(c.properties().cost()*quantity);
            }
            _ => {
                cost=cost+genetic_cost((x,y),(next_x as usize,next_y as usize),&Arc::new(map.clone()));
            }
        }


        x=next_x as usize;
        y=next_y as usize;

    }

    cost
}
