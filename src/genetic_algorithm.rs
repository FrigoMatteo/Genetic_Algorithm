use rand::{Rng, thread_rng};
use crate::{INFINITE, INPUT_DIR_SIZE};
use robotics_lib::interface::Direction;
use std::collections::HashSet;
use robotics_lib::world::tile::Tile;
use std::sync::Arc;
use robotics_lib::utils::calculate_cost_go_with_environment;
use robotics_lib::world::tile::Content::Rock;
use crate::helpers_functions::{direction_value, is_good_tile, is_not_visualize};
use crate::ENVIRONMENT;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InputDir{
    Right(bool),
    Left(bool),
    Top(bool),
    Bottom(bool),
    None
}


impl InputDir{
    fn is_reverse(&self, other:&InputDir)->bool{

        match self{
            InputDir::Top(_)=>{if std::mem::discriminant(other)==std::mem::discriminant(&InputDir::Bottom(false) ){true} else {false}},
            InputDir::Left(_)=>{if std::mem::discriminant(other)==std::mem::discriminant(&InputDir::Right(false)) {true} else {false}},
            InputDir::Bottom(_)=>{if std::mem::discriminant(other)==std::mem::discriminant(&InputDir::Top(false)) {true} else {false}},
            InputDir::Right(_)=>{if std::mem::discriminant(other)==std::mem::discriminant(&InputDir::Left(false)) {true} else {false}}
            _ => {false}
        }

    }

    pub(crate) fn property(&self) ->Direction{
        match self{
            InputDir::Right(_) => Direction::Right,
            InputDir::Left(_) => Direction::Left,
            InputDir::Top(_) => Direction::Up,
            InputDir::Bottom(_)=> Direction::Down,
            _=>{Direction::Down}
        }
    }

    fn random_input_dir()->InputDir{
        let mut rng =thread_rng();
        let t: i32 =rng.gen_range(0..5);

        match t{
            0=>InputDir::Bottom(false),
            1=>InputDir::Left(false),
            2=>InputDir::None,
            3=>InputDir::Right(false),
            _=>InputDir::Top(false),
        }
    }
}

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

    pub(crate) fn new(n:usize, x:i32, y:i32) ->Self{
        let mut g=GeneticSearch{
            vector:Vec::new(),
            cost:INFINITE,
            distanze_from_dest:100,
            start_x:x,
            start_y:y,
            weight:1000,
        };

        g.generate_random_sequence(n);
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

    fn generate_random_sequence(&mut self,n:usize){
        for _ in 0..n{

            //The rng is used because it is faster to catch the generated value if we have to launch it a lot of times.
            self.vector.push(InputDir::random_input_dir());

        }

    }

    pub(crate) fn genetic_cost(&mut self, inside_thread_map:&Arc<Vec<Vec<Option<Tile>>>>, destination:(usize, usize)){
        let mut x=self.start_x;
        let mut y=self.start_y;

        let mut next_x=0;
        let mut next_y=0;

        let mut backtracking=0;

        let mut null_block=0;

        let mut object_destroy=0;

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
                while *ele!=e{
                    e=InputDir::random_input_dir();
                }
                *ele=e;
                continue;
            }


            if *ele!=InputDir::None{
                self.cost=self.cost+genetic_cost(
                    (x as usize,y as usize),
                    (next_x as usize,next_y as usize),
                    inside_thread_map);
            }

            if inside_thread_map[next_x as usize][next_y as usize].is_some(){
                let tile=inside_thread_map[next_x as usize][next_y as usize].as_ref().unwrap();

                if !set.contains(&(next_x, next_y)) && (tile.content.to_default()==Rock(0)){

                    object_destroy+=1;

                    //We don't want backtracking to the same block.
                    //This way he can't exploit the fact to constantly go in the same block to lower his weight.
                    set.insert((next_x,next_y));

                    self.cost+=tile.content.properties().cost();

                    //We can consider to grab the object since we walk past it.
                    let element=ele.clone();

                    *ele=match element{
                        InputDir::Right(_) => InputDir::Right(true),
                        InputDir::Left(_) => InputDir::Left(true),
                        InputDir::Top(_) => InputDir::Top(true),
                        InputDir::Bottom(_) => InputDir::Top(true),
                        InputDir::None => InputDir::None,
                    }
                }else{
                    let element=ele.clone();
                    *ele=match element{
                        InputDir::Right(_) => InputDir::Right(false),
                        InputDir::Left(_) => InputDir::Left(false),
                        InputDir::Top(_) => InputDir::Top(false),
                        InputDir::Bottom(_) => InputDir::Top(false),
                        InputDir::None => InputDir::None,
                    }
                }
            }

            x=next_x;
            y=next_y;

        }
        self.distanze_from_dest=(x-destination.0 as i32).abs()+(y-destination.1 as i32).abs();


        //Checking backtracking:
        ///
        /// self.vector=[Right, Right, Left, Left]
        ///
        /// BackTracking=2
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

        //We fixed the weight based on the cost(10%)+backtracking(30%)+null_block(30%)-object_destroyed(30%)
        self.weight=((self.cost as f32*0.1)+((backtracking*10)as f32*0.3)+((null_block*10)as f32*0.3)-((object_destroy*10)as f32*0.3)) as i32;
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
