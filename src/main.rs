use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::thread::{sleep, spawn};
use std::time::Duration;
use rand::{Rng, thread_rng};
use robotics_lib::energy::Energy;
use robotics_lib::event::events::Event;
use robotics_lib::interface::{debug, destroy, Direction, go, look_at_sky, one_direction_view, put, robot_map};
use robotics_lib::runner::{Robot, Runnable, Runner};
use robotics_lib::runner::backpack::BackPack;
use robotics_lib::world::coordinates::Coordinate;
use robotics_lib::world::environmental_conditions::WeatherType::*;
use robotics_lib::world::tile::{Content, Tile, TileType};
use robotics_lib::world::tile::Content::*;
use robotics_lib::world::tile::TileType::{DeepWater, Grass, Hill, Lava, Mountain, Sand, ShallowWater, Snow, Street, Teleport};
use robotics_lib::world::World;
use sense_and_find_by_rustafariani;
use rust_eze_spotlight::Spotlight;


use charting_tools::charted_paths;
use ghost_amazeing_island::world_generator::*;
use charting_tools::ChartingTools;
use charting_tools::charted_coordinate::ChartedCoordinate;
use charting_tools::charted_paths::ChartedPaths;


use image::{Rgb, RgbImage};
//use image::codecs::png::CompressionType::Default;
use lazy_static::lazy_static;
use robotics_lib::utils::{calculate_cost_go_with_environment, LibError};
use robotics_lib::utils::LibError::{NotEnoughContentProvided, NotEnoughEnergy, OperationNotAllowed};
use robotics_lib::world::environmental_conditions::EnvironmentalConditions;


static DISTANCE:usize=4;
static ONE_DIRECTION_DISTANCE:usize=8;
static INFINITE:usize=10000;
static WORLD_SIZE:usize=300;
static INPUT_DIR_SIZE:usize=12;
static GENERATION_LIMIT:usize=100;
static POPULATION_NUMBER:usize=8;


fn export_to_image(map: &Vec<Vec<Option<Tile>>>, filename: &str,robot:&impl Runnable) {
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
        TileType::Teleport(_)=>Rgb([255,0,255]),
        _ => Rgb([0,0,0]),
        }
}



//Static means it has 'static lifetime.
//ref, we allow myltiple parts of our code to access the same piece of data without copying it
lazy_static! {
    //This mutex will block threads waiting for the lock to become available
    static ref RECHARGE:Mutex<bool>=Mutex::new(false);

    //We created a Arc, because we need to use it inside threads without data racing.
    //We have also created it as Mutex, since we want to modify it.
    static ref POSITION:Arc<Mutex<(usize,usize)>>=Arc::new(Mutex::new((0,0)));

    //We create the hashmap of position that my robot can go

    static ref ROBOT_MAP:Arc<Mutex<Vec<Vec<Option<Tile>>>>>=Arc::new(Mutex::new(Vec::new()));

    static ref ENVIRONMENT:Mutex<Option<EnvironmentalConditions>>=Mutex::new(Option::None);

    static ref POSITIONS_TO_GO:Mutex<Vec<PositionToGo>>=Mutex::new(PositionToGo::new());

    static ref FOLLOW_DIRECTIONS:Mutex<MovesToFollow>=Mutex::new(MovesToFollow::new());

    static ref WAIT_FOR_ENERGY:Mutex<bool>=Mutex::new(false);

}


struct MyRobot{
    robot:Robot,
    interest_points:HashMap<(usize,usize),Content>,
}


impl Runnable for MyRobot {
    fn process_tick(&mut self, world: &mut World) {


        //Immediately return. We are running the threads
        if *RECHARGE.lock().unwrap(){
            return;
        }



        let mut follow_dir=FOLLOW_DIRECTIONS.lock().unwrap();

        if self.enough_energy_to_operate(&(*follow_dir),world){
            *WAIT_FOR_ENERGY.lock().unwrap()=false;
        }else{
            *WAIT_FOR_ENERGY.lock().unwrap()=true;
        }


        //check if we need more energy
        if *WAIT_FOR_ENERGY.lock().unwrap(){
            return;
        }


        //We follow the path created by the threads (we basically move)
        if !follow_dir.path_to_follow.is_empty(){
            println!("Energy before walking:{:?}",self.get_energy());
            //Actuator
            self.move_based_on_threads(world,&follow_dir.path_to_follow);
            follow_dir.path_to_follow.clear();

        }

        // I update my position
        let d=self.get_coordinate();

        POSITION.lock().unwrap().0=d.get_row();
        POSITION.lock().unwrap().1=d.get_col();



        //I visualize the new area I have just moved
        println!("After walking Energy:{:?}",self.get_energy());
        let res_visualize=self.visualize_around(world);
        match res_visualize{
            Ok(_)=>{},
            Err(e)=>{if e==NotEnoughEnergy{*WAIT_FOR_ENERGY.lock().unwrap()=true; return;}
                // TODO do something for the operation not allowed and content not provided
                if e==OperationNotAllowed{return;}
                if e==NotEnoughContentProvided{return;}
            },
        }
        println!("After walking and visualizing Energy:{:?}",self.get_energy());



        //I use the debug function to show the global map only. I never use it for calculation purpose
        let v=debug(self,world).0;
        let mut new:Vec<Vec<Option<Tile>>>=Vec::new();
        for i in v{
            let mut vet:Vec<Option<Tile>>=Vec::new();
            for j in i{
                vet.push(Some(j));
            }
            new.push(vet);
        }

        //I create the graphical image of the map (total view)
        export_to_image(&new,"mappa.jpg",self);


        //I upload the new image of what I have seen
        let v=robot_map(world).unwrap();
        export_to_image(&v,"visualize.jpg",self);

        print!("\n\n");


        //I upload the new static data, which they will be used by the threads.
        self.update_static_data(world);
        self.save_contents(world);



        //let mut charted_path = ChartingTools::tool::<ChartedPaths>().unwrap();
        //charted_path.init(&robot_map(world).unwrap(), world);
        /*
        let de=destroy(self,world,Direction::Right);
        println!("{:?}",de);
        let _=go(self,world,Direction::Right);
        let wh=put(self,world,Garbage(0),5,Direction::Right);
        println!("wh:{:?}",wh);
        */

    }

    fn handle_event(&mut self, event: Event) {
        //println!("{:?}", event);
    }

    fn get_energy(&self) -> &Energy {
        &self.robot.energy
    }

    fn get_energy_mut(&mut self) -> &mut Energy {
        &mut self.robot.energy
    }

    fn get_coordinate(&self) -> &Coordinate {
        &self.robot.coordinate
    }

    fn get_coordinate_mut(&mut self) -> &mut Coordinate {
        &mut self.robot.coordinate
    }

    fn get_backpack(&self) -> &BackPack {
        &self.robot.backpack
    }

    fn get_backpack_mut(&mut self) -> &mut BackPack {
        &mut self.robot.backpack
    }
}

impl MyRobot{

    fn move_based_on_threads(&mut self,world:&mut World,path:&Vec<InputDir>){

        for i in path{
            if *i==InputDir::None{continue}

            let _=go(self,world,i.property());
        }


    }

    fn get_from_to(&self,distance:usize,len:i32)->Vec<(i32,i32)>{

        let my_coordinates=self.get_coordinate();
        let mut from_x=my_coordinates.get_row() as i32-distance as i32-1;
        if from_x<0{ from_x=0}

        let mut to_x=my_coordinates.get_row() as i32+distance as i32+1;
        if to_x>len {to_x=len}

        let mut from_y=my_coordinates.get_col()as i32-distance as i32-1;
        if from_y<0{from_y=0}

        let mut to_y=my_coordinates.get_col() as i32+distance as i32+1;
        if to_y>len {to_y=len}
        vec![(from_x,to_x),(from_y,to_y)]
    }

    fn save_contents(&mut self, world:&World){
        if robot_map(world).is_none(){return}

        let map=robot_map(world).unwrap();

        let v=self.get_from_to(ONE_DIRECTION_DISTANCE,map.len() as i32);

        for i in v[0].0..v[0].1{
            for j in v[1].0..v[1].1{
                if map[i as usize][j as usize].is_none(){continue}

                let content=&map[i as usize][j as usize].as_ref().unwrap().content;

                if content.to_default()!=Rock(0) && content.to_default()!=Garbage(0) && content.to_default()!=Coin(0) && content.to_default()!=Bin(0..0) && content.to_default()!=Crate(0..0){ continue }

                self.interest_points.insert((i as usize,j as usize), content.clone());
            }
        }

        /*
        for i in &self.interest_points{
            println!("At this coordinate:({},{}) there is {:?}",i.0.0,i.0.1,i.1);
        }
         */

    }

    fn update_static_data(&self,world:&World){
        ENVIRONMENT.lock().unwrap().replace(look_at_sky(world));


        //I clear my old view
        ROBOT_MAP.lock().unwrap().clear();
        let map=robot_map(world);

        if map.is_none(){return;}

        let mut m=map.unwrap();

        //I copy the new view into my ROBOT_MAP, used for explore threads.
        ROBOT_MAP.lock().unwrap().append(&mut m);

    }

    fn visualize_around(&mut self,world:&mut World)->Result<(),LibError>{
        POSITIONS_TO_GO.lock().unwrap().clear();

        if robot_map(world).is_none(){return Err(OperationNotAllowed);}

        let rob_map=robot_map(world).unwrap();

        let d=self.get_coordinate();
        let x=d.get_row();
        let y=d.get_col();

        //I initialize the vector also used by the threads, which they will find the best path to it
        let res_vet=PositionToGo::new_with_world(rob_map,x,y);

        *POSITIONS_TO_GO.lock().unwrap()=res_vet.clone();


        let _=Spotlight::illuminate(&Default::default(), self, world, DISTANCE);

        for i in res_vet{
            let _=match i{
                PositionToGo::Down => {Some(one_direction_view(self,world,Direction::Down,ONE_DIRECTION_DISTANCE));},
                PositionToGo::Right => {Some(one_direction_view(self,world,Direction::Right,ONE_DIRECTION_DISTANCE));},
                PositionToGo::Top => {Some(one_direction_view(self,world,Direction::Up,ONE_DIRECTION_DISTANCE));},
                PositionToGo::Left => {Some(one_direction_view(self,world,Direction::Left,ONE_DIRECTION_DISTANCE));},
                _=>{},
            };
        }

        return Ok(());
    }

    fn enough_energy_to_operate(&mut self, moves:&MovesToFollow,world:&World)->bool{

        let mut cost=moves.cost;

        let d=self.get_coordinate();
        let mut x=d.get_row();
        let mut y=d.get_col();

        for i in &moves.path_to_follow{
            if *i==InputDir::None{continue}

            let d=direction_value(&i);

            x=(x as i32+d.0) as usize;
            y=(y as i32+d.1) as usize;

        }

        //Energy cost used for visualize the unknown world:

        if robot_map(world).is_none(){return false;}//return Err(OperationNotAllowed);}

        let rob_map=robot_map(world).unwrap();

        //I initialize the vector also used by the threads, which they will find the best path to it
        let res_vet=PositionToGo::new_with_world(rob_map,x,y);

        //Check energy costs:
        let mut cost_energy =0;
        let mut flag=false;

        for i in &res_vet{

            cost_energy+=match i{
                PositionToGo::Down => ONE_DIRECTION_DISTANCE*3,
                PositionToGo::Right => ONE_DIRECTION_DISTANCE*3,
                PositionToGo::Top => ONE_DIRECTION_DISTANCE*3,
                PositionToGo::Left => ONE_DIRECTION_DISTANCE*3,
                _=>{flag=true; 0},
            };
        }

        if flag{
            //Only problem with the calculation of the total cost:
            let mut sp=Spotlight::calculate_illuminate_cost(&Default::default(),self,world,DISTANCE).unwrap();
            //The spotlight cost isn't correct. idk why

            //For manage the spotlight cost error I put a bigger limit to it
            if sp<200{
                sp=200;
            }

            cost_energy+=sp;
        }

        cost+=cost_energy;

        self.get_energy().has_enough_energy(cost)
    }

}

struct MovesToFollow{
    path_to_follow:Vec<InputDir>,
    cost:usize,
}

impl MovesToFollow{
    fn new()->Self{
        MovesToFollow{
            path_to_follow:Vec::new(),
            cost:0,
        }
    }

}

#[derive(Debug,Clone)]
enum PositionToGo{
    Down,
    DownRight,
    Right,
    TopRight,
    Top,
    TopLeft,
    Left,
    DownLeft,
}

impl PositionToGo{
    fn new()->Vec<PositionToGo>{

        let iter_me=[PositionToGo::Down, PositionToGo::DownRight,
            PositionToGo::Right, PositionToGo::TopRight, PositionToGo::Top,
            PositionToGo::TopLeft, PositionToGo::Left, PositionToGo::DownLeft];

        let mut v:Vec<PositionToGo>=Vec::new();

        for i in iter_me{
            v.push(i);
        }

        v
    }

    fn new_with_world(rob_map:Vec<Vec<Option<Tile>>>,x:usize,y:usize)->Vec<PositionToGo>{


        let iter_me=[PositionToGo::Down, PositionToGo::DownRight,
            PositionToGo::Right, PositionToGo::TopRight, PositionToGo::Top,
            PositionToGo::TopLeft, PositionToGo::Left, PositionToGo::DownLeft];

        let mut v=Vec::new();


        for i in iter_me{
            let position=i.clone();

            let (ds_x,ds_y)=get_next_position(i);

            let destination_x=(x as i32)+ds_x;
            let destination_y=(y as i32)+ds_y;


            if destination_x>=WORLD_SIZE as i32 || destination_y>=WORLD_SIZE as i32 || destination_x<0 || destination_y<0{ println!("here here");continue }

            if rob_map[destination_x as usize][destination_y as usize].is_none(){
                v.push(position);
            }
        }


        v

    }

}

#[derive(Debug, Clone, Copy, PartialEq)]
enum InputDir{
    Right,
    Left,
    Top,
    Bottom,
    None
}


impl InputDir{
    fn is_reverse(&self, other:&InputDir)->bool{

        match self{
            InputDir::Top=>{if *other==InputDir::Bottom {true} else {false}},
            InputDir::Left=>{if *other==InputDir::Right {true} else {false}},
            InputDir::Bottom=>{if *other==InputDir::Top {true} else {false}},
            InputDir::Right=>{if *other==InputDir::Left {true} else {false}}
            _ => {false}
        }

    }

    fn property(&self)->Direction{

        match self{
            InputDir::Right => Direction::Right,
            InputDir::Left => Direction::Left,
            InputDir::Top => Direction::Up,
            InputDir::Bottom => Direction::Down,
            _=>{Direction::Down}
        }

    }
}

#[derive(Clone,PartialEq)]
struct GeneticSearch{
    vector:Vec<InputDir>,
    cost:usize,
    distanze_from_dest:i32,
    start_x:i32,
    start_y:i32,
    weight:i32
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

    fn new(n:usize, x:i32, y:i32)->Self{
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
        let mut rng =thread_rng();
        for _ in 0..n{
            //The rng is used because it is faster to catch the genereted value if we have to lauch it a lot of times.
            let t: i32 =rng.gen_range(0..5);

            self.vector.push(match t{
                0=>InputDir::Bottom,
                1=>InputDir::Left,
                2=>InputDir::None,
                3=>InputDir::Right,
                _=>InputDir::Top,
            });

        }

    }

    fn genetic_cost(&mut self,inside_thread_map:&Arc<Vec<Vec<Option<Tile>>>>,destination:(usize,usize)){
        let mut x=self.start_x;
        let mut y=self.start_y;

        let mut next_x=0;
        let mut next_y=0;

        let mut backtracking=0;

        let mut null_block=0;

        self.cost=0;

        for ele in self.vector.iter_mut(){
            let (i,j)=direction_value(ele);

            next_x=x+i;
            next_y=y+j;


            if next_y>=WORLD_SIZE as i32|| next_x>=WORLD_SIZE as i32 || next_x<0 || next_y<0 || inside_thread_map[next_x as usize][next_y as usize].is_none(){
                null_block+=1;
                //Potrebbe creare problemi se esce dalla mappa ancora prima di arrivare alla destinazione.
                //Quindi non va a coprire il prossimo if.
                if (x-destination.0 as i32).abs()+(y-destination.1 as i32).abs()==0{
                    *ele=InputDir::None;
                }
                continue;
            }


            if *ele!=InputDir::None{
                self.cost=self.cost+genetic_cost(
                    (x as usize,y as usize),
                    (next_x as usize,next_y as usize),
                    inside_thread_map);
            }

            x=next_x;
            y=next_y;
        }
        self.distanze_from_dest=(x-destination.0 as i32).abs()+(y-destination.1 as i32).abs();

        //println!("Costs:{} and distance from destination:{}",self.cost,self.distanze_from_dest);


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

        //Possiamo definire il peso rispetto a: Backtracking, Costo e Movimento non possibile(va verso un blocco Null)
        self.weight=((self.cost as f32*0.1)+((backtracking*10)as f32*0.45)+((null_block*10)as f32*0.45)) as i32;
        //println!("Specific weight:{}",self.weight);
    }

}


fn main() {

    let r = MyRobot {
        robot:Robot::new(),
        interest_points:HashMap::new(),
    };

    //let mut generator: WorldGeneratorRandom = WorldGeneratorRandom::new(10);
    let mut g = WorldGenerator::new(300, true, 0, 0.1);


    let mut run = Runner::new(Box::new(r), &mut g).unwrap();


    for _ in 0..100{
        loop {
            let _ = run.game_tick();
            if !*WAIT_FOR_ENERGY.lock().unwrap(){
                break;
            }
        }

        *RECHARGE.lock().unwrap()=true;

        let time=spawn(||{

            //First thread, which will launch the other threads.
            let mut handlers=vec![];


            let data=ROBOT_MAP.lock().unwrap();
            let map=Arc::new(data.clone());
            let positions=POSITIONS_TO_GO.lock().unwrap().clone();


            let x=POSITION.lock().unwrap().0.clone();
            let y=POSITION.lock().unwrap().1.clone();


            for i in positions{
                //Follows the other threads for specific direction

                //I lauch a thread for every specific direction which we may follow
                let thread_map=Arc::clone(&map);

                //move converts any variables captured by reference or mutable reference to variables captured by value
                let handle=spawn( move ||{
                    //Robot map for the threads.
                    let inside_thread_map=Arc::clone(&thread_map);

                    //Get position of where our thread's destination is.
                    let (destination_x,destination_y)=get_next_position(i);

                    let mut genetic_set=Vec::new();

                    //Initial population
                    for _ in 0..8{
                        let n=GeneticSearch::new(INPUT_DIR_SIZE,x as i32,y as i32);
                        genetic_set.push(n);
                    }


                    let mut f=true;
                    let mut counter=0;
                    let mut index_save=0;
                    let mut new_index=0;

                    let dest_x=(x as i32+destination_x) as usize;
                    let dest_y=(y as i32+destination_y) as usize;

                    //We repeat the Selection, Crossover and mutation:
                    for _ in 0..GENERATION_LIMIT{
                        //I fixed the generation limit to 100, which is optimal, since also the children learn from the parents.


                        counter+=1;
                        //Genetic Fitness, we calculate the weight of the random generated directions
                        for i in genetic_set.iter_mut(){
                            i.genetic_cost(&inside_thread_map,(dest_x,dest_y));
                            if f && i.distanze_from_dest==0{
                                index_save=counter;
                                new_index=index_save+30;
                                f=false;
                            }
                        }


                        //Genetic Selection: we take an elite set and one based on probability.
                        //This way, also the children can learn.
                        //let (first,second)=genetic_selection(&mut genetic_set);
                        let (first,second)=genetic_selection(&mut genetic_set);

                        //Genetic crossover. Here we generete new sons from first and second
                        genetic_crossover(&mut genetic_set, &first, &second, &x, &y);

                        //Genetic mutation. Where are going to change a some value for escaping the local min problem.
                        genetic_mutation(&mut genetic_set);

                        //We also push the two winning parents with their children
                        genetic_set.push(first);
                        genetic_set.push(second);
                    }

                    //We generate the last generation:
                    for i in genetic_set.iter_mut(){
                        i.genetic_cost(&inside_thread_map,(dest_x,dest_y));
                    }
                    /*
                    println!("\n\nAfter 100 generations:");

                    for i in genetic_set.iter(){
                        println!("I got weight:{},distance:{}, cost:{} and this serie:{:?}",i.weight,i.distanze_from_dest,i.cost,i.vector);
                    }

                    println!("I get 0 at n:{}",index_save);

                     */


                    //we take the fastest sample:
                    let mut index_res=INFINITE;
                    for i in genetic_set.iter().enumerate(){
                        if i.1.distanze_from_dest==0{
                            if index_res==INFINITE{
                                index_res=i.0;
                            }else if i.1.weight<genetic_set[index_res].weight{
                                index_res=i.0;
                            }
                        }
                    }


                    if index_res!=INFINITE{
                        Some(genetic_set[index_res].clone())
                    }else{
                        Option::None
                    }

                });
                handlers.push(handle);
            }
            //Example of return value from thead
            let mut min_so_far=GeneticSearch::default();
            //println!("\n\nAt the calculation I got:");
            for i in handlers{
                let value=i.join().unwrap();
                //println!("I got weight:{},distance:{}, cost:{} and this serie:{:?}",value.weight,value.distanze_from_dest,value.cost,value.vector);

                if value==Option::None{continue}

                let value=value.unwrap();

                if value.distanze_from_dest==0 && value.weight<min_so_far.weight{
                    if value.weight==min_so_far.weight{
                        if value.cost<min_so_far.cost{
                            min_so_far=value;
                        }
                    }else{
                        min_so_far=value;
                    }
                }
            }
            println!("\n\nPath to follow:");
            println!("I got weight:{},distance:{}, cost:{} and this serie:{:?}\n",min_so_far.weight,min_so_far.distanze_from_dest,min_so_far.cost,min_so_far.vector);

            min_so_far

        });

        for _ in 0..30{
            let _=run.game_tick();
            sleep(Duration::from_millis(10));
        }

        //Wait
        let t=time.join().unwrap();

        *RECHARGE.lock().unwrap()=false;
        FOLLOW_DIRECTIONS.lock().unwrap().path_to_follow=t.vector;
        FOLLOW_DIRECTIONS.lock().unwrap().cost=t.cost;
    }

}


fn genetic_selection(population:&mut Vec<GeneticSearch>)->(GeneticSearch,GeneticSearch){
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

fn genetic_mutation(population:&mut Vec<GeneticSearch>){
    let mut rng =thread_rng();
    for element in population.iter_mut(){
        for i in element.vector.iter_mut(){
            let probability=rng.gen_range(0..10);
            //That's equivalent to 30% of probability
            if probability<=0{
                let mut g=i.clone();
                while *i==g{
                    let t: i32 =rng.gen_range(0..5);

                    g=match t{
                        0=>InputDir::Bottom,
                        1=>InputDir::Left,
                        2=>InputDir::None,
                        3=>InputDir::Right,
                        _=>InputDir::Top,
                    };

                }

                *i=g;

            }

        }
    }
}

fn genetic_crossover(population:&mut Vec<GeneticSearch>,first:&GeneticSearch,second:&GeneticSearch,x:&usize,y:&usize){
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

fn genetic_cost(current_coord: (usize, usize), target_coord: (usize, usize), map:&Arc<Vec<Vec<Option<Tile>>>>) -> usize {
    // Get tiles
    let target_tile = map[target_coord.0][target_coord.1].clone().unwrap();
    let current_tile = map[current_coord.0][current_coord.1].clone().unwrap();

    // Init costs
    let mut base_cost = target_tile.tile_type.properties().cost();
    let mut elevation_cost = 0;


    // Get informations that influence the cost
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

fn direction_value(value:&InputDir)->(i32,i32){
    match value{
        InputDir::None=>(0,0),
        InputDir::Top=>(-1,0),
        InputDir::Right=>(0,1),
        InputDir::Left=>(0,-1),
        InputDir::Bottom=>(1,0),
    }
}

fn get_next_position(pos:PositionToGo)->(i32,i32){
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