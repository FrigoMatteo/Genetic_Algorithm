use rand::{Rng, thread_rng};
use crate::{INFINITE, INPUT_DIR_SIZE};
use robotics_lib::interface::Direction;
use std::collections::HashSet;
use robotics_lib::world::tile::Tile;
use std::sync::Arc;
use charting_tools::charted_coordinate::ChartedCoordinate;
use robotics_lib::utils::calculate_cost_go_with_environment;
use robotics_lib::world::tile::Content::{Coin, Garbage, Tree};
use robotics_lib::world::tile::TileType::ShallowWater;
use crate::helpers_functions::{direction_value, is_good_tile, is_not_visualize};
use crate::ENVIRONMENT;


//I set two bools for the InputDir:
//The first if we need to destroy something.
//The second if we need to put something. (I am lazy)
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InputDir{
    Right(bool,bool),
    Left(bool,bool),
    Top(bool,bool),
    Bottom(bool,bool),
    None
}


impl InputDir{
    fn is_reverse(&self, other:&InputDir)->bool{

        match self{
            InputDir::Top(_,_)=>{if std::mem::discriminant(other)==std::mem::discriminant(&InputDir::Bottom(false,false) ){true} else {false}},
            InputDir::Left(_,_)=>{if std::mem::discriminant(other)==std::mem::discriminant(&InputDir::Right(false,false)) {true} else {false}},
            InputDir::Bottom(_,_)=>{if std::mem::discriminant(other)==std::mem::discriminant(&InputDir::Top(false,false)) {true} else {false}},
            InputDir::Right(_,_)=>{if std::mem::discriminant(other)==std::mem::discriminant(&InputDir::Left(false,false)) {true} else {false}}
            _ => {false}
        }

    }

    pub(crate) fn property(&self) ->Direction{
        match self{
            InputDir::Right(_,_) => Direction::Right,
            InputDir::Left(_,_) => Direction::Left,
            InputDir::Top(_,_) => Direction::Up,
            InputDir::Bottom(_,_)=> Direction::Down,
            _=>{Direction::Down}
        }
    }

    fn random_input_dir()->InputDir{
        let mut rng =thread_rng();
        let t: i32 =rng.gen_range(0..5);

        match t{
            0=>InputDir::Bottom(false,false),
            1=>InputDir::Left(false,false),
            2=>InputDir::None,
            3=>InputDir::Right(false,false),
            _=>InputDir::Top(false,false),
        }
    }

    fn random_input_without_i(i:&InputDir)->InputDir{
        let mut rng =thread_rng();
        let t: i32 =rng.gen_range(0..4);

        match i{
            InputDir::Right(_, _) => {
                match t{
                    0=>InputDir::Bottom(false,false),
                    1=>InputDir::Left(false,false),
                    2=>InputDir::None,
                    _=>InputDir::Top(false,false),
                }
            }
            InputDir::Left(_, _) => {
                match t{
                    0=>InputDir::Bottom(false,false),
                    1=>InputDir::None,
                    2=>InputDir::Right(false,false),
                    _=>InputDir::Top(false,false),
                }
            }
            InputDir::Top(_, _) => {
                match t{
                    0=>InputDir::Bottom(false,false),
                    1=>InputDir::Left(false,false),
                    2=>InputDir::None,
                    _=>InputDir::Right(false,false),
                }
            }
            InputDir::Bottom(_, _) => {
                match t{
                    0=>InputDir::Left(false,false),
                    1=>InputDir::None,
                    2=>InputDir::Right(false,false),
                    _=>InputDir::Top(false,false),
                }
            }
            InputDir::None => {
                match t{
                    0=>InputDir::Bottom(false,false),
                    1=>InputDir::Left(false,false),
                    2=>InputDir::Right(false,false),
                    _=>InputDir::Top(false,false),
                }
            }
        }
    }

    pub fn convert_to_input_dir(mut x:i32,mut y:i32,vet:Vec<ChartedCoordinate>)->Vec<InputDir>{

        let mut result=Vec::new();
        for i in vet{
            result.push(InputDir::convert_int_to_input_dir(x, y, i.0 as i32, i.1 as i32));

            x=i.0 as i32;
            y=i.1 as i32;
        }
        result
    }

    fn convert_int_to_input_dir(x:i32,y:i32,x1:i32,y1:i32)->InputDir{
        match (x1-x,y1-y){
            (0,1)=>InputDir::Right(false,false),
            (0,-1)=>InputDir::Left(false,false),
            (1,0)=>InputDir::Bottom(false,false),
            (-1,0)=>InputDir::Top(false,false),
            _=>{InputDir::None}
        }
    }

    fn convert_to_int(i:&InputDir)->(i32,i32){
        match i{
            InputDir::Right(_, _) => {(0,1)}
            InputDir::Left(_, _) => {(0,-1)}
            InputDir::Top(_, _) => {(-1,0)}
            InputDir::Bottom(_, _) => {(1,0)}
            InputDir::None => {(0,0)}
        }
    }
}


// This is our element used in the genetic algorithm.
// So, from it, we will create all our "robot path" used in the generations.
#[derive(Clone,PartialEq)]
pub struct GeneticSearch{
    pub(crate) vector:Vec<InputDir>,
    pub(crate) cost:usize,
    pub(crate) distanze_from_dest:i32,
    start_x:i32,
    start_y:i32,
    pub(crate) weight:i32
}

impl Default for GeneticSearch{
    fn default() -> Self {

        GeneticSearch{
            vector:Vec::new(),
            cost:INFINITE,
            distanze_from_dest:INFINITE as i32,
            start_y:0,
            start_x:0,
            weight:INFINITE as i32,
        }
    }
}

impl GeneticSearch{

    pub(crate) fn new(n:usize, x:i32, y:i32,inside_thread_map:&Arc<Vec<Vec<Option<Tile>>>>) ->Self{
        let mut g=GeneticSearch{
            vector:Vec::new(),
            cost:INFINITE,
            distanze_from_dest:100,
            start_x:x,
            start_y:y,
            weight:1000,
        };

        g.generate_random_sequence(n,x,y,inside_thread_map);
        g
    }

    fn new_with_vector(x:i32,y:i32,vector:Vec<InputDir>)->Self{
        GeneticSearch{
            vector,
            cost:INFINITE,
            distanze_from_dest:100,
            start_y:y,
            start_x:x,
            weight:1000,
        }
    }

    fn generate_random_sequence(&mut self,n:usize,mut x:i32, mut y:i32,inside_thread_map:&Arc<Vec<Vec<Option<Tile>>>>){

        let mut x_dem=x;
        let mut y_dem=y;

        for _ in 0..n{
            let mut dir=InputDir::random_input_dir();

            (x_dem,y_dem)=InputDir::convert_to_int(&dir);

            let mut x_s=x+x_dem;
            let mut y_s=y+y_dem;

            if x_s<0{
                x_s=0;
            }
            if y_s<0 {
                y_s=0;
            }

            if is_not_visualize(x_s,y_s) || inside_thread_map[x_s as usize][y_s as usize].is_none(){self.vector.push(InputDir::None);continue;}


            while !is_good_tile(&inside_thread_map[x_s as usize][y_s as usize]){
                x_s=x;
                y_s=y;

                dir=InputDir::random_input_dir();

                (x_dem,y_dem)=InputDir::convert_to_int(&dir);

                x_s+=x_dem;
                y_s+=y_dem;

                if is_not_visualize(x,y){dir=InputDir::None;break;}
            }

            x=x_s;
            y=y_s;

            //The rng is used because it is faster to catch the generated value if we have to launch it a lot of times.
            self.vector.push(dir);

        }

    }


    pub(crate) fn genetic_cost(&mut self, inside_thread_map:&Arc<Vec<Vec<Option<Tile>>>>, destination:(usize, usize)){
        let mut x=self.start_x;
        let mut y=self.start_y;

        let mut next_x=x;
        let mut next_y=y;

        let mut backtracking=0;

        let mut null_block=0;

        let mut object_destroy=0;

        let mut sha_water=0;

        let mut set:HashSet<(i32,i32)>=HashSet::new();

        self.cost=0;

        for ele in self.vector.iter_mut(){

            let (i,j)=direction_value(ele);

            next_x=x+i;
            next_y=y+j;


            if is_not_visualize(next_x, next_y) || inside_thread_map[next_x as usize][next_y as usize].is_none(){
                null_block+=1;
                *ele=InputDir::None;
                continue;
            }

            if !is_good_tile(&inside_thread_map[next_x as usize][next_y as usize]){
                null_block+=1;

                let mut e =ele.clone();


                let mut save_x=x+direction_value(&e).0;

                if save_x<0{
                    save_x=0
                }

                let mut save_y=y+direction_value(&e).1;

                if save_y<0{
                    save_y=0;
                }


                while e==*ele ||
                    is_not_visualize(save_x, save_y) ||
                    inside_thread_map[save_x as usize][save_y as usize].is_none() ||
                    !is_good_tile(&inside_thread_map[save_x as usize][save_y as usize]) {

                    e=InputDir::random_input_without_i(&e);

                    save_x=x+direction_value(&e).0;

                    if save_x<0{
                        save_x=0;
                    }

                    save_y=y+direction_value(&e).1;

                    if save_y<0{
                        save_y=0;
                    }

                }

                *ele=e;

                let (i,j)=direction_value(ele);
                next_x=x+i;
                next_y=y+j;
            }


            if next_x<0{
                next_x=0;
            }
            if next_y<0{
                next_y=0;
            }

            if *ele!=InputDir::None{
                self.cost=self.cost+genetic_cost(
                    (x as usize,y as usize),
                    (next_x as usize,next_y as usize),
                    inside_thread_map);
            }


            if inside_thread_map[next_x as usize][next_y as usize].is_some(){
                let tile=inside_thread_map[next_x as usize][next_y as usize].as_ref().unwrap();

                //Because the shallow water doesn't cost much and he keeps walking in there.
                if tile.tile_type==ShallowWater{
                    sha_water+=1;
                }


                // We check if the next tile contains the contents we search.
                if !set.contains(&(next_x, next_y)) && (tile.content.to_default()==Coin(0) || tile.content.to_default()==Garbage(0) || tile.content.to_default()==Tree(0)){

                    object_destroy+=1;

                    //We don't want backtracking to the same block.
                    //This way he can't exploit the fact to constantly go in the same block to lower his weight.
                    set.insert((next_x,next_y));

                    self.cost+=tile.content.properties().cost();

                    //We can consider to grab the object since we walk past it.
                    let element=ele.clone();

                    *ele=match element{
                        InputDir::Right(_,_) => InputDir::Right(true,false),
                        InputDir::Left(_,_) => InputDir::Left(true,false),
                        InputDir::Top(_,_) => InputDir::Top(true,false),
                        InputDir::Bottom(_,_) => InputDir::Bottom(true,false),
                        InputDir::None => InputDir::None,
                    }
                }else{
                    let element=ele.clone();
                    *ele=match element{
                        InputDir::Right(_,_) => InputDir::Right(false,false),
                        InputDir::Left(_,_) => InputDir::Left(false,false),
                        InputDir::Top(_,_) => InputDir::Top(false,false),
                        InputDir::Bottom(_,_) => InputDir::Bottom(false,false),
                        InputDir::None => InputDir::None,
                    }
                }
            }

            if is_good_tile(&inside_thread_map[next_x as usize][next_y as usize]){
                x=next_x;
                y=next_y;
            }

        }
        self.distanze_from_dest=(x-destination.0 as i32).abs()+(y-destination.1 as i32).abs();


        //Checking backtracking:
        //
        // self.vector=[Right, Right, Left, Left]
        //
        // BackTracking=2
        //I want to penalize the fact that he waste those 4 actions

        let mut vet=Vec::new();
        for i in self.vector.iter(){
            if *i==InputDir::None{continue;}
            if vet.is_empty(){
                vet.push(i);
            }
            else{
                if i.is_reverse(vet.last().unwrap()){
                    backtracking+=1;
                    vet.pop();
                }else{
                    vet.push(i);
                }
            }
        }

        //We fixed the weight based on the cost(10%)+backtracking(25%)+null_block(25%)-object_destroyed(25%)+shallow water(15%)
        //I also put a weight on the shallow water since it doesn't cost much and he keep walking in there.

        self.weight=((self.cost as f32*0.01)+((backtracking*10)as f32*0.15)+((null_block*10)as f32*0.25)-((object_destroy*10)as f32*0.25)+((sha_water*50)as f32*0.34)) as i32;
        //println!("Specific weight:{}",self.weight);
    }

}


pub fn genetic_selection(population:&mut Vec<GeneticSearch>)->(GeneticSearch,GeneticSearch){
    population.sort_by(|a,b| {
        let d=a.distanze_from_dest.cmp(&b.distanze_from_dest);
        if d.is_eq(){
            a.weight.cmp(&b.weight)
        }else{
            d
        }
    });

    let probability_choice=||->usize{
        let t=thread_rng().gen_range(0..28);
        match t{
            0..=6=>1,
            7..=12=>2,
            13..=17=>3,
            19..=22=>4,
            23..=25=>5,
            26..=27=>6,
            _=>7,
        }
    };


    let first=population[0].clone();
    let value=probability_choice();
    let second=population[value].clone();

    population.remove(value);
    population.remove(0);

    (first,second)
}

pub fn genetic_mutation(population:&mut Vec<GeneticSearch>){
    let mut rng =thread_rng();
    for element in population.iter_mut(){
        for i in element.vector.iter_mut(){
            let probability=rng.gen_range(0..10);
            //That's equivalent to 10% of probability
            if probability<=0{
                let mut g=i.clone();
                while *i==g{ g=InputDir::random_input_dir(); }

                *i=g;

            }

        }
    }
}

pub fn genetic_crossover(population:&mut Vec<GeneticSearch>,first:&GeneticSearch,second:&GeneticSearch,x:&usize,y:&usize){
    population.clear();

    let piece=INPUT_DIR_SIZE/3;

    let first_part=[&first.vector[..piece],&second.vector[..piece]];
    let second_part=[&first.vector[piece..2*piece],&second.vector[piece..2*piece]];
    let third_part=[&first.vector[2*piece..],&second.vector[2*piece..]];

    population.push(GeneticSearch::new_with_vector(*x as i32,*y as i32,[first_part[0],second_part[1],third_part[0]].concat()));
    population.push(GeneticSearch::new_with_vector(*x as i32,*y as i32,[first_part[0],second_part[1],third_part[1]].concat()));
    population.push(GeneticSearch::new_with_vector(*x as i32,*y as i32,[first_part[1],second_part[0],third_part[0]].concat()));
    population.push(GeneticSearch::new_with_vector(*x as i32,*y as i32,[first_part[1],second_part[0],third_part[1]].concat()));
    population.push(GeneticSearch::new_with_vector(*x as i32,*y as i32,[first_part[0],second_part[0],third_part[1]].concat()));
    population.push(GeneticSearch::new_with_vector(*x as i32,*y as i32,[first_part[1],second_part[1],third_part[0]].concat()));
}


pub fn genetic_cost(current_coord: (usize, usize), target_coord: (usize, usize), map:&Arc<Vec<Vec<Option<Tile>>>>) -> usize {
    // Get tiles
    let target_tile = map[target_coord.0][target_coord.1].clone().unwrap();
    let current_tile = map[current_coord.0][current_coord.1].clone().unwrap();

    // Init costs
    let mut base_cost = target_tile.tile_type.properties().cost();
    let mut elevation_cost = 0;


    // Get information's that influence the cost
    let environmental_conditions = ENVIRONMENT.lock().unwrap().to_owned().unwrap();


    let new_elevation = target_tile.elevation;
    let current_elevation = current_tile.elevation;

    // Calculate cost
    base_cost = calculate_cost_go_with_environment(base_cost, environmental_conditions, target_tile.tile_type);

    // Consider elevation cost only if we are going from a lower tile to a higher tile
    if new_elevation > current_elevation {
        elevation_cost = (new_elevation - current_elevation).pow(2);
    }

    base_cost + elevation_cost
}
