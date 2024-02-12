mod export_of_image;
mod genetic_algorithm;
mod helpers_functions;

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::thread::{sleep, spawn};
use std::time::Duration;
use robotics_lib::energy::Energy;
use robotics_lib::event::events::Event;
use robotics_lib::interface::{debug, destroy, Direction, go, look_at_sky, one_direction_view, put, robot_map};
use robotics_lib::runner::{Robot, Runnable, Runner};
use robotics_lib::runner::backpack::BackPack;
use robotics_lib::world::coordinates::Coordinate;
use robotics_lib::world::tile::{Content, Tile};
use robotics_lib::world::tile::Content::*;
use robotics_lib::world::World;
use robotics_lib::utils::LibError;
use robotics_lib::utils::LibError::{NotEnoughEnergy, OperationNotAllowed};
use robotics_lib::world::environmental_conditions::EnvironmentalConditions;

use rust_eze_spotlight::Spotlight;
use ghost_amazeing_island::world_generator::*;
use charting_tools::ChartingTools;
use charting_tools::charted_coordinate::ChartedCoordinate;
use charting_tools::charted_paths::ChartedPaths;

use genetic_algorithm::{InputDir,GeneticSearch,genetic_selection,genetic_mutation,genetic_crossover};
use helpers_functions::{get_next_position,is_not_visualize,is_good_tile,direction_value};

use lazy_static::lazy_static;


pub static DISTANCE:usize=4;
pub static ONE_DIRECTION_DISTANCE:usize=8;
pub static INFINITE:usize=10000;
static WORLD_SIZE:usize=500;
static INPUT_DIR_SIZE:usize=18;
static GENERATION_LIMIT:usize=180;
static POPULATION_NUMBER:usize=8;



//Static means it has 'static lifetime.
//ref, we allow multiple parts of our code to access the same piece of data without copying it
lazy_static! {
    //This mutex will block threads waiting for the lock to become available.
    // Used when the threads are processing and we want the main thread to run some random game tick
    static ref RECHARGE:Mutex<bool>=Mutex::new(false);

    //We created a Arc, because we need to use it inside threads without data racing.
    //We have also created it as Mutex, since we want to modify it.
    static ref POSITION:Arc<Mutex<(usize,usize)>>=Arc::new(Mutex::new((0,0)));

    //We create the hashmap of position that my robot can go

    static ref ROBOT_MAP:Arc<Mutex<Vec<Vec<Option<Tile>>>>>=Arc::new(Mutex::new(Vec::new()));

    pub(crate) static ref ENVIRONMENT:Mutex<Option<EnvironmentalConditions>>=Mutex::new(Option::None);

    static ref POSITIONS_TO_GO:Mutex<Vec<PositionToGo>>=Mutex::new(PositionToGo::new());

    static ref FOLLOW_DIRECTIONS:Mutex<MovesToFollow>=Mutex::new(MovesToFollow::new());

    // If we need to charge our robot because we don't have enough energy to do our stuff.
    static ref WAIT_FOR_ENERGY:Mutex<bool>=Mutex::new(false);

    //Content we need to put in the crate/bin/bank
    static ref CONTENT:Mutex<PutContent>=Mutex::new(PutContent::default());

}


struct PutContent{
    content:Content,
    quantity:usize,
}
impl Default for PutContent{
    fn default() -> Self {
        Self{
            content:None,
            quantity:0
        }
    }
}

struct MyRobot{
    robot:Robot,
    interest_points:HashMap<(usize,usize),Content>,
    visited:Vec<Vec<bool>>,
}


impl Runnable for MyRobot {
    fn process_tick(&mut self, world: &mut World) {

        //Check if we have some issue with the energy, such as thread running or need energy to operate.
        if !self.is_energy_right(world){
            return;
        }


        let mut follow_dir=FOLLOW_DIRECTIONS.lock().unwrap();
        //We follow the path created by the threads (we basically move)
        if !follow_dir.path_to_follow.is_empty(){
            //Actuator
            self.move_based_on_threads(world,&mut follow_dir.path_to_follow);

            if follow_dir.is_done(){
                follow_dir.path_to_follow.clear();
            }else{
                println!("We left because is not done yet");
                return;
            }
        }

        drop(follow_dir);

        // I update my position
        let d=self.get_coordinate();

        POSITION.lock().unwrap().0=d.get_row();
        POSITION.lock().unwrap().1=d.get_col();


        //Implement functions for go in the bank/crate/bin:
        //check backpack:
        if self.backpack_contains_something(){
            let (content,size)=self.get_content_backpack();
            println!("Content to search:{:?} and size:{}",content,size);

            let search=self.search_respective_content(&content);

            if search!=None && self.container_exists(&search){
                println!("Size backpack:{}",self.get_backpack().get_size());
                for i in self.get_backpack().get_contents().iter(){
                    if i.0.to_default()==Rock(0).to_default() || i.0.to_default()==Coin(0) || i.0.to_default()==Garbage(0) || i.0.to_default()==Tree(0){
                        println!("Size of {}:{}",i.0.to_default(),i.1);
                    }
                }

                let (dest_x,dest_y)=self.search_content(search,d.get_row(),d.get_col());

                let mut charted_path = ChartingTools::tool::<ChartedPaths>().unwrap();
                charted_path.init(&robot_map(world).unwrap(), world);

                let ch1=ChartedCoordinate::from((d.get_row(),d.get_col()));
                let ch2=ChartedCoordinate::from((dest_x,dest_y));

                let path=charted_path.shortest_path(ch1,ch2);

                *CONTENT.lock().unwrap()=PutContent{
                    content,quantity:size,
                };

                if path.is_some(){

                    let path=path.unwrap();

                    FOLLOW_DIRECTIONS.lock().unwrap().cost=path.0;

                    let mut result_path=InputDir::convert_to_input_dir(d.get_row() as i32,d.get_col() as i32,path.1);

                    let dir=result_path.pop().unwrap();
                    let dir=match dir{
                        InputDir::Right(_, _) => {InputDir::Right(false,true)}
                        InputDir::Left(_, _) => {InputDir::Left(false,true)}
                        InputDir::Top(_, _) => {InputDir::Top(false,true)}
                        InputDir::Bottom(_, _) => {InputDir::Bottom(false,true)}
                        InputDir::None => {InputDir::None}
                    };
                    result_path.push(dir);

                    ///ERROR IN THE EXECUTION OF THE PATH
                    println!("Charted path found");
                    for i in result_path.iter(){
                        println!("{:?}",i);
                    }
                    println!("Costo:{}",path.0);//Problema se il costo supera i 1000.
                    FOLLOW_DIRECTIONS.lock().unwrap().path_to_follow=result_path;
                    *WAIT_FOR_ENERGY.lock().unwrap()=true;
                    return;
                }
                else{
                    println!("Error with the creation of path with charted path");
                    return;
                }
            }
        }


        //I visualize the new area I have just moved in
        //println!("After walking Energy:{:?}",self.get_energy());
        let res_visualize=self.visualize_around(world);
        match res_visualize{
            Ok(_)=>{},
            Err(e)=>{if e==NotEnoughEnergy{*WAIT_FOR_ENERGY.lock().unwrap()=true;}
                // TODO do something for the operation not allowed and content not provided
                println!("Operation not allowed:{:?}",e);return;
            },
        }
        //println!("After walking and visualizing Energy:{:?}",self.get_energy());


        //Function for the map image:
        self.visualize_robot_map(world);

        //I upload the new static data, which they will be used by the threads.
        self.update_static_data(world);
        self.save_contents(world);

        //println!("\nMy coordinates:{:?}",self.get_coordinate());

        /*
        let wh=put(self,world,Garbage(0),5,Direction::Right);
        println!("wh:{:?}",wh);
        */

    }

    fn handle_event(&mut self, event: Event) { /*println!("{:?}", event); */}

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
    fn new()->Self{
        let mut vet=Vec::new();
        for _ in 0..WORLD_SIZE{
            let mut v=Vec::new();
            for _ in 0..WORLD_SIZE{
                v.push(false);
            }
            vet.push(v);
        }
        Self{
            robot: Robot::new(),
            interest_points: HashMap::new(),
            visited:vet,
        }
    }

    fn move_based_on_threads(&mut self,world:&mut World,path:&mut Vec<InputDir>){

        for i in path.iter_mut(){
            if *i==InputDir::None{continue}

            match *i{
                InputDir::Right(a,_) => {self.destroy_if_true(world,a,i)},
                InputDir::Left(a,_) => {self.destroy_if_true(world,a,i);}
                InputDir::Top(a,_) => {self.destroy_if_true(world,a,i);}
                InputDir::Bottom(a,_) => {self.destroy_if_true(world,a,i);}
                InputDir::None => {}
            }

            //println!("Move to do:{:?}",i);

            let cont=CONTENT.lock().unwrap().content.clone();
            let quantity=CONTENT.lock().unwrap().quantity.clone();

            match *i{
                InputDir::Right(_,true) => {let d=put(self, world, cont, quantity, Direction::Right);println!("{:?}",d);},
                InputDir::Left(_,true) => {let d=put(self, world,  cont, quantity, Direction::Left);println!("{:?}",d);}
                InputDir::Top(_,true) => {let d=put(self, world, cont, quantity, Direction::Up);println!("{:?}",d);}
                InputDir::Bottom(_,true) => {let d=put(self, world, cont, quantity, Direction::Down);println!("{:?}",d);}
                _ => {
                    let d=go(self,world,i.property());
                    match d{
                        Ok(_) => {}
                        Err(e) => {println!("Error move:{:?}",e)}
                    }
                }
            }

            *i=InputDir::None;
        }

        //We set the coordinate we arrived as true, so we won't go here again.
        let x=self.get_coordinate().get_row();
        let y=self.get_coordinate().get_col();
        self.visited[x][y]=true;

    }

    fn destroy_if_true(&mut self,world:&mut World, t:bool,data:&InputDir){
        if t{
            let _=destroy(self,world,data.property());
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

                if content.to_default()==None || content.to_default()==Fire || content.to_default()==Tree(0) || content.to_default()==Bush(0) || content.to_default()==Fish(0) || content.to_default()==Rock(0) || content.to_default()==Coin(0) || content.to_default()==Garbage(0) || content.to_default()==Market(0){continue}
                else{
                    if match content{
                        Bin(a) => {if a.start==a.end{false}else{true}}
                        Crate(a) => {if a.start==a.end{false}else{true}}
                        Bank(a) => {if a.start==a.end{false}else{true}}
                        _ => {true}
                    }{
                        self.interest_points.insert((i as usize,j as usize), content.clone());
                    }

                }
            }
        }

        for i in &self.interest_points{
            println!("At this coordinate:({},{}) there is {:?}",i.0.0,i.0.1,i.1);
        }

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
        let res_vet =PositionToGo::new_with_world(&rob_map, x, y);



        let _=Spotlight::illuminate(self,world,DISTANCE);
        //let _=Spotlight::illuminate( &Spotlight::default(),self, world, DISTANCE);

        let mut result=Vec::new();

        //We discover new tiles around us
        for i in res_vet.iter().enumerate(){
             match i.1{
                 PositionToGo::Right => {if !is_not_visualize(x as i32, y as i32+ONE_DIRECTION_DISTANCE as i32){let _=one_direction_view(self, world, Direction::Right, ONE_DIRECTION_DISTANCE);}},
                 PositionToGo::Down => {if !is_not_visualize(x as i32+ONE_DIRECTION_DISTANCE as i32, y as i32){let _=one_direction_view(self, world, Direction::Down, ONE_DIRECTION_DISTANCE);}},
                 PositionToGo::Left => {if !is_not_visualize(x as i32, y as i32-ONE_DIRECTION_DISTANCE as i32){let _=one_direction_view(self, world, Direction::Left, ONE_DIRECTION_DISTANCE);}}
                 PositionToGo::Top => {if !is_not_visualize(x as i32-ONE_DIRECTION_DISTANCE as i32, y as i32){let _=one_direction_view(self, world, Direction::Up, ONE_DIRECTION_DISTANCE);}}
                 _=>{},
             }
        }

        //We take the new robot map with the updated robot map
        let rob_map=robot_map(world).unwrap();

        //We create define the vector with the position we have to go
        for i in res_vet.iter().enumerate(){
            //We set the first condition because we don't know if it can go out of bounds
            //We second conditions is set because we don't want to set a point to go which is "Lava", "DeepWater" or Tile=None.
            match i.1{
                PositionToGo::Right => {if !is_not_visualize(x as i32, y as i32+ONE_DIRECTION_DISTANCE as i32) && is_good_tile(&rob_map[x][y+ONE_DIRECTION_DISTANCE]){result.push(i.1.clone());}},
                PositionToGo::DownRight => {if !is_not_visualize(x as i32+DISTANCE as i32, y as i32+DISTANCE as i32) && is_good_tile(&rob_map[x+DISTANCE][y+DISTANCE]){result.push(i.1.clone());}},
                PositionToGo::Down => {if !is_not_visualize(x as i32+ONE_DIRECTION_DISTANCE as i32, y as i32) && is_good_tile(&rob_map[x+ONE_DIRECTION_DISTANCE][y]){result.push(i.1.clone());}},
                PositionToGo::TopRight => {if !is_not_visualize(x as i32-DISTANCE as i32,y as i32+DISTANCE as i32) && is_good_tile(&rob_map[x-DISTANCE][y+DISTANCE]){result.push(i.1.clone());}},
                PositionToGo::Left => {if !is_not_visualize(x as i32, y as i32-ONE_DIRECTION_DISTANCE as i32) && is_good_tile(&rob_map[x][y-ONE_DIRECTION_DISTANCE]){result.push(i.1.clone());}},
                PositionToGo::TopLeft => {if !is_not_visualize(x as i32-DISTANCE as i32, y as i32-DISTANCE as i32) && is_good_tile(&rob_map[x-DISTANCE][y-DISTANCE]){result.push(i.1.clone());}},
                PositionToGo::Top => {if !is_not_visualize(x as i32-ONE_DIRECTION_DISTANCE as i32, y as i32) && is_good_tile(&rob_map[x-ONE_DIRECTION_DISTANCE][y]){result.push(i.1.clone());}},
                PositionToGo::DownLeft => {if !is_not_visualize(x as i32+DISTANCE as i32, y as i32-DISTANCE as i32) && is_good_tile(&rob_map[x+DISTANCE][y-DISTANCE]){result.push(i.1.clone());}}
            }
        }

        /*
        for i in result.iter(){
            println!("Position to go:{:?}",i);
        }
         */

        // We prevent some possible problem. Since, it might get stuck in the threads to search for
        // the best path.
        if result.len()<=2{
            result=PositionToGo::new_already_seen(self,&rob_map,x,y);
        }

        *POSITIONS_TO_GO.lock().unwrap()=result.clone();

        return Ok(());
    }

    fn already_visited(&self,x:i32,y:i32)->bool{
        if self.visited[x as usize][y as usize]{
            true
        }else{
            false
        }
    }

    fn enough_energy_to_operate(&mut self, moves:&MovesToFollow,world:&World)->bool{

        let mut cost=moves.cost;

        let d=self.get_coordinate();
        let mut x=d.get_row();
        let mut y=d.get_col();

        if robot_map(world).is_none(){return false;}//return Err(OperationNotAllowed);}

        let rob_map=robot_map(world).unwrap();

        //I initialize the vector also used by the threads, which they will find the best path to it
        let res_vet=PositionToGo::new_with_world(&rob_map,x,y);

        for i in &moves.path_to_follow{
            if *i==InputDir::None{continue}

            let d=direction_value(&i);

            x=(x as i32+d.0) as usize;
            y=(y as i32+d.1) as usize;


        }


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
            let mut sp=Spotlight::calculate_illuminate_cost(self,world,DISTANCE).unwrap();
            //The spotlight cost isn't correct. idk why

            //For manage the spotlight cost error I put a bigger limit to it
            if sp<200{
                sp=200;
            }

            cost_energy+=sp;
        }

        cost+=cost_energy;

        if self.get_energy().get_energy_level()>500 && cost>1000{
            false
        }else{
            self.get_energy().has_enough_energy(cost)
        }
    }

    fn visualize_robot_map(&mut self,world:&mut World){
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
        export_of_image::export_to_image(&new,"mappa.jpg",self);


        //I upload the new image of what I have seen
        let v=robot_map(world).unwrap();
        export_of_image::export_to_image(&v,"visualize.jpg",self);
    }

    fn is_energy_right(&mut self, world:&mut World) ->bool{
        //Immediately return. We are running the threads
        if *RECHARGE.lock().unwrap(){
            return false;
        }
        let follow_dir=FOLLOW_DIRECTIONS.lock().unwrap();


        // Do we need to do some free cycles because we don't have enough energy?
        if self.enough_energy_to_operate(&(*follow_dir),world){
            *WAIT_FOR_ENERGY.lock().unwrap()=false;
        }else{
            *WAIT_FOR_ENERGY.lock().unwrap()=true;
        }
        //check if we need more energy
        if *WAIT_FOR_ENERGY.lock().unwrap(){
            return false;
        }
        true
    }

    fn get_content_backpack(&self)->(Content,usize){
        let mut max_so_far:usize=0;
        let mut cont=None;

        for i in self.get_backpack().get_contents().iter(){
            if i.0.to_default()==Garbage(0) && *i.1>=5{
                return (i.0.clone(),*i.1);
            }else if (i.0.to_default()==Garbage(0).to_default()|| i.0.to_default()==Coin(0)) && *i.1>max_so_far{
                max_so_far=*i.1;
                cont=i.0.clone();
            }
        }
        (cont,max_so_far)
    }

    fn search_content(&self,content:Content,x:usize,y:usize)->(usize,usize){

        let mut res_x=INFINITE;
        let mut res_y=INFINITE;

        for i in self.interest_points.iter(){
            if i.1.to_default()==content{
                if (x.abs_diff(i.0.0)+(y.abs_diff(i.0.1)))<(x.abs_diff(res_x)+y.abs_diff(res_y)){
                    res_x=i.0.0;
                    res_y=i.0.1;
                }
            }
        }

        (res_x,res_y)
    }

    fn container_exists(&self,content:&Content)->bool{
        for i in self.interest_points.iter(){
            if i.1.to_default()==*content{
                return true;
            }
        }
        false
    }

    fn search_respective_content(&self,content:&Content)->Content{
        match content {
            Garbage(_) => {Bin(0..0)}
            Coin(_) => {Bank(0..0)}
            _=>{None}
        }
    }

    fn backpack_contains_something(&self)->bool{
        let map=self.get_backpack().get_contents();
        let mut max:usize =0;
        for i in map{
            if *i.1>max && self.container_exists(&self.search_respective_content(i.0)){
                max=*i.1;
            }
        }
        if max>5{
            true
        }else{
            false
        }
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

    fn is_done(&self)->bool{
        for i in self.path_to_follow.iter(){
            if *i!=InputDir::None{
                return false;
            }
        }
        true
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

    fn new_with_world(rob_map:&Vec<Vec<Option<Tile>>>,x:usize,y:usize)->Vec<PositionToGo>{

        let iter_me=[PositionToGo::Down, PositionToGo::DownRight,
            PositionToGo::Right, PositionToGo::TopRight, PositionToGo::Top,
            PositionToGo::TopLeft, PositionToGo::Left, PositionToGo::DownLeft];

        let mut v=Vec::new();


        for i in iter_me{
            let position=i.clone();

            let (ds_x,ds_y)=get_next_position(i);

            let destination_x=(x as i32)+ds_x;
            let destination_y=(y as i32)+ds_y;


            if is_not_visualize(destination_x, destination_y){ continue }

            if rob_map[destination_x as usize][destination_y as usize].is_none(){
                v.push(position);
            }
        }

        if v.is_empty(){
            PositionToGo::new()
        }else{
            v
        }
    }

    fn new_already_seen(robot:&MyRobot,rob_map:&Vec<Vec<Option<Tile>>>,x:usize,y:usize)->Vec<PositionToGo>{
        let iter_me=[PositionToGo::Down, PositionToGo::DownRight,
            PositionToGo::Right, PositionToGo::TopRight, PositionToGo::Top,
            PositionToGo::TopLeft, PositionToGo::Left, PositionToGo::DownLeft];


        let mut v=Vec::new();

        for i in iter_me{
            let position=i.clone();

            let (ds_x,ds_y)=get_next_position(i);

            let destination_x=(x as i32)+ds_x;
            let destination_y=(y as i32)+ds_y;


            if is_not_visualize(destination_x, destination_y){ continue }

            if is_good_tile(&rob_map[destination_x as usize][destination_y as usize]) && !robot.already_visited(destination_x,destination_y){
                v.push(position);
            }
        }

        v
    }
    fn new_already_seen_without_rob(rob_map:&Vec<Vec<Option<Tile>>>,x:usize,y:usize)->Vec<PositionToGo>{
        let iter_me=[PositionToGo::Down, PositionToGo::DownRight,
            PositionToGo::Right, PositionToGo::TopRight, PositionToGo::Top,
            PositionToGo::TopLeft, PositionToGo::Left, PositionToGo::DownLeft];


        let mut v=Vec::new();

        for i in iter_me{
            let position=i.clone();

            let (ds_x,ds_y)=get_next_position(i);

            let destination_x=(x as i32)+ds_x;
            let destination_y=(y as i32)+ds_y;


            if is_not_visualize(destination_x, destination_y){ continue }

            if is_good_tile(&rob_map[destination_x as usize][destination_y as usize]){
                v.push(position);
            }
        }

        v
    }

}


fn main() {

    let r = MyRobot::new();

    let mut g = WorldGenerator::new(WORLD_SIZE.clone() as u32, true, 0, 0.1);


    let mut run = Runner::new(Box::new(r), &mut g).unwrap();


    for _ in 0..100{
        loop {
            let _ = run.game_tick();
            if !*WAIT_FOR_ENERGY.lock().unwrap(){
                break;
            }
        }

        // We use this mutex as a semaphore
        *RECHARGE.lock().unwrap()=true;


        let time=spawn(||{
            let mut thread_flag=true;

            //Return values from the threads:
            let mut min_so_far=GeneticSearch::default();

            //We control how many interations we do to search for the best path.
            let mut counter_try=0;

            while thread_flag{
                counter_try+=1;
                min_so_far=GeneticSearch::default();

                //First thread, which will launch the other threads.
                let mut handlers=vec![];


                let data=ROBOT_MAP.lock().unwrap();
                let map=Arc::new(data.clone());
                let positions=POSITIONS_TO_GO.lock().unwrap().clone();


                let x=POSITION.lock().unwrap().0.clone();
                let y=POSITION.lock().unwrap().1.clone();


                for i in positions{

                    //I launch a thread for every specific direction which we may follow
                    let thread_map=Arc::clone(&map);

                    //Move converts any variables captured by reference or mutable reference to variables captured by value
                    let handle=spawn( move ||{
                        //Robot map for the threads.
                        let inside_thread_map=Arc::clone(&thread_map);

                        //Get position of where our thread's destination is.
                        let (destination_x,destination_y)=get_next_position(i);

                        let mut genetic_set=Vec::new();

                        //Initial population
                        for _ in 0..POPULATION_NUMBER{
                            let n=GeneticSearch::new(INPUT_DIR_SIZE,x as i32,y as i32,&inside_thread_map);
                            genetic_set.push(n);
                        }

                        let mut f=true;

                        let dest_x=(x as i32+destination_x) as usize;
                        let dest_y=(y as i32+destination_y) as usize;

                        //We repeat the Selection, Crossover and mutation:
                        for _ in 0..GENERATION_LIMIT{
                            //I fixed the generation limit to 100, which is optimal, since also the children learn from the parents.

                            //Genetic Fitness, we calculate the weight of the random generated directions
                            for i in genetic_set.iter_mut(){
                                i.genetic_cost(&inside_thread_map,(dest_x,dest_y));

                                if f && i.distanze_from_dest==0{ f=false; }

                            }

                            //Genetic Selection: we take an elite set and one based on probability.
                            //This way, also the children can learn.
                            //let (first,second)=genetic_selection(&mut genetic_set);
                            let (first,second)=genetic_selection(&mut genetic_set);

                            //Genetic crossover. Here we generate new sons from first and second
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

                        //we take the fastest sample:
                        let mut index_res=0;
                        for i in genetic_set.iter().enumerate(){
                            if i.1.distanze_from_dest<=genetic_set[index_res].distanze_from_dest{
                                if i.1.weight<genetic_set[index_res].weight{
                                    index_res=i.0;
                                }
                            }
                        }
                        /*
                        println!("\nI am going to pass:");
                        println!("I got weight:{},distance:{}, cost:{} and this series:{:?}",genetic_set[index_res].weight,genetic_set[index_res].distanze_from_dest,genetic_set[index_res].cost,genetic_set[index_res].vector);
                         */

                        if index_res!=INFINITE{
                            Some(genetic_set[index_res].clone())
                        }else{
                            Option::None
                        }

                    });
                    handlers.push(handle);
                }

                //println!("\n\nAt the calculation I got:");
                for i in handlers{
                    let value=i.join().unwrap();

                    if value==Option::None{continue}

                    let value=value.unwrap();

                    //println!("I got weight:{},distance:{}, cost:{}",value.weight,value.distanze_from_dest,value.cost);

                    if value.distanze_from_dest<=min_so_far.distanze_from_dest{
                        if value.weight==min_so_far.weight{
                            if value.cost<min_so_far.cost{
                                min_so_far=value;
                            }
                        }else{
                            min_so_far=value;
                        }
                    }
                }

                //We are going to leave the loop only if the result distance is at least lower than 1.
                //If it doesn't work we will try again.
                //We need to add some conditions, because it might get stucked if we are close to the deep water
                //println!("Counter:{}",counter_try);
                if min_so_far.distanze_from_dest<=1{
                    thread_flag=false;
                }else if counter_try>=3 && counter_try<=10{
                    if min_so_far.distanze_from_dest<=2{
                        thread_flag=false;
                    }
                }else if counter_try>10 && counter_try<=15{
                    let pos=PositionToGo::new_already_seen_without_rob(&map,x,y);
                    *POSITIONS_TO_GO.lock().unwrap()=pos.clone();
                    for i in pos{
                        println!("Position:{:?}",i);
                    }
                    println!("I got weight:{},distance:{}, cost:{}",min_so_far.weight,min_so_far.distanze_from_dest,min_so_far.cost);
                    println!("\n");
                    if min_so_far.distanze_from_dest<=4{
                        thread_flag=false;
                    }
                }else if counter_try>15{
                    println!("Error in the calculation of the path");
                    return GeneticSearch::default();
                }
            }
            println!("Path to follow:");
            //println!("I got weight:{},distance:{}, cost:{} and this series:{:?}\n",min_so_far.weight,min_so_far.distanze_from_dest,min_so_far.cost,min_so_far.vector);
            println!("I got weight:{},distance:{}, cost:{}\n\n\n",min_so_far.weight,min_so_far.distanze_from_dest,min_so_far.cost);

            min_so_far

        });

        //That's an ideal time for the threads to finish their work. It also helps me to keep a balanced energy level for the robot.
        for _ in 0..20{
            let _=run.game_tick();
            sleep(Duration::from_millis(20));
        }

        //We wait for the thread, which contains all the other threads, to finish
        let t=time.join().unwrap();

        //Check if we got an error in the calculation
        if t.weight>=1000{
            return;
        }

        *RECHARGE.lock().unwrap()=false;
        FOLLOW_DIRECTIONS.lock().unwrap().path_to_follow=t.vector;
        FOLLOW_DIRECTIONS.lock().unwrap().cost=t.cost;
    }

}