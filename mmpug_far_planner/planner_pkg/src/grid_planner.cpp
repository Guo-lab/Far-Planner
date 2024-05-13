#include "grid_planner.h"

using namespace std;

/* GridPlanner constructor */
GridPlanner::GridPlanner(int map_size, float map_resolution, int min_obs_cost){
    goal_updated = false;
    
    this->obstacle_cost = min_obs_cost;
    this->map_resolution = map_resolution;
    this->x_offset = -map_size/2;
    this->y_offset = -map_size/2;
    this->x_size = map_size;
    this->y_size = map_size;

    this->map.clear();
    std::vector<int8_t> data(map_size * map_size);
    std::fill(data.begin(), data.end(), this->obstacle_cost/2);
    this->map = data;
}
/* GridPlanner destructor */
GridPlanner::~GridPlanner(){}


inline int GridPlanner::computeKey(int x, int y){
    return (y*y_size + x);
}

inline int GridPlanner::getMapIndex(int x, int y){
    return (y*y_size + x);
}


// ======================= Distance ============================
/**
 * @brief Function to estimate the octile distance between two points
 *      dx + dy - min(dx, dy)
*/
float GridPlanner::estimateOctileDistance(int curr_x, int curr_y, int goal_x, int goal_y){
    int delta_x = abs(curr_x - goal_x);
    int delta_y = abs(curr_y - goal_y);
    return delta_x + delta_y - MIN(delta_x,delta_y);
}

/**
 * @brief Function to estimate the euclidean distance between two points
 *     sqrt(dx^2 + dy^2)
*/
float GridPlanner::estimateEuclideanDistance(int curr_x, int curr_y, int goal_x, int goal_y){
    int delta_x = abs(curr_x - goal_x);
    int delta_y = abs(curr_y - goal_y);
    return sqrt(delta_x*delta_x + delta_y*delta_y);
}


/**
 * @brief Function to update the map.
*/
void GridPlanner::updateMap(const nav_msgs::OccupancyGrid& grid, geometry_msgs::Point& corner1){
    int x1, y1;

    for(int i = 2; i < grid.info.width - 3; i++){
        for(int j = 2; j < grid.info.height - 3 ; j++){

            x1 = std::floor(corner1.x/map_resolution) - x_offset;
            y1 = std::floor(corner1.y/map_resolution) - y_offset;

            if((x1 + i) >= 0 && (x1 + i) < x_size && (y1 + j) >= 0 && (y1 + j) < y_size ){
                if(this->map[(x1 + i) + ((y1 + j) * y_size)] == obstacle_cost/2){
                    this->map[(x1 + i) + ((y1 + j) * y_size)] = (int8_t) grid.data.at(i + (j * grid.info.height));
                }
                else{
                    if(grid.data.at(i + (j * grid.info.height)) != obstacle_cost/2){
                        this->map[(x1 + i) + ((y1 + j) * y_size)] = (int8_t) grid.data.at(i + (j * grid.info.height));
                    }
                }
               
            }
        }
    }

}

void GridPlanner::getMap(std::vector<int8_t>& map_data){
    map_data = this->map;
}



// void GridPlanner::updateMap(const nav_msgs::OccupancyGrid::ConstPtr& grid){
//     map_resolution = grid->info.resolution;
//     y_size = grid->info.width;
//     x_size = grid->info.height;
//     x_offset = (grid->info.origin.position.x)/(map_resolution);
//     y_offset = (grid->info.origin.position.y)/(map_resolution);
//     map.clear();
//     map = grid->data;
// }

void GridPlanner::printInfo(){
    ROS_INFO_STREAM(" Map size: " << map.size());
    // ROS_INFO_STREAM("Robot Position: " <<  start_node.x << " " << start_node.y);
    // ROS_INFO_STREAM("Goal Position: " <<  goal_locations[0].x << " " << goal_locations[0].y);

    ROS_INFO_STREAM("x_offset: "<< x_offset <<" y_offset: " <<y_offset);
}

// void GridPlanner::updateGoalCells(){
//     Node t1;
//     t1.x = (goal_loc.x)/(map_resolution) - x_offset;
//     t1.y = (goal_loc.y)/(map_resolution) - y_offset;
//     t1.key = computeKey(t1.x, t1.y);
//     goal_locations.clear();
//     goal_locations.push_back(t1);   
// }
// bool GridPlanner::reachedGoal(std::shared_ptr<Node> curr_node){
//     for(int i = 0; i < goal_locations.size(); i++){
//         if(*curr_node == goal_locations[i])
//             return true;
//     }    
//     return false;
// }




/**
 * @brief Function to plan in a naive way
*/
int GridPlanner::naivePlanner(
    const geometry_msgs::Pose& robot_pose, 
    const geometry_msgs::Pose& target, 
    geometry_msgs::PoseArray& plan) 
{
    
    start_node.x = robot_pose.position.x/map_resolution - x_offset;
    start_node.y = robot_pose.position.y/map_resolution - x_offset;

    goal_node.x = target.position.x/map_resolution - x_offset;
    goal_node.y = target.position.y/map_resolution - x_offset;


    if(start_node.x < 0 || start_node.x >= x_size || start_node.y < 0 || start_node.y >= x_size){
        ROS_WARN("CANCELLING PLANNER BECAUSE ROBOT IS NOT IN THE GRID");
        return 0;
    }
    if(goal_node.x < 0 || goal_node.x >= x_size || goal_node.y < 0 || goal_node.y >= x_size){
        ROS_WARN("CANCELLING PLANNER BECAUSE GOAL IS NOT IN THE GRID");
        return 0;
    }
    
    //Setup start node, closed and open lists.
    CLOSED_LIST closed_list;
    OPEN_LIST open_list;

    Nodeptr start = std::make_shared<Node>(start_node.x, start_node.y);
    start->key = computeKey(start_node.x, start_node.y);
    start->g = 0;
    start->h = 0;
    start->f = start->g;
    start->parent = NULL;
    start->time = max_steps;
    
    int current_iter = 0;
    int goal_key;
    
    open_list.push(start);
    // ROS_INFO("Setup A star");
    bool pathFound = false;
    while(!open_list.empty() && closed_list.count(computeKey(goal_node.x, goal_node.y)) == 0 ){
        Nodeptr curr_node = open_list.top();
        open_list.pop();
        int curr_key = computeKey(curr_node->x, curr_node->y);
        // ROS_INFO("Here1");
        if(closed_list.count(curr_key) == 0 ){

            closed_list.insert({curr_key, curr_node});
            
            if(*curr_node == goal_node){
                goal_key = curr_key;
                pathFound = true;
                ROS_INFO("Found Path");
                break;
            }

            for(int dir = 0; dir < 8; dir++)
            {
                int newx = curr_node->x + dX[dir];
                int newy = curr_node->y + dY[dir];
                int newt = curr_node->time - 1;

                bool expand = false;
                if(closed_list.count(computeKey(newx, newy)) == 0)
                    expand = true;
                else{
                    if(closed_list.at(computeKey(newx, newy))->time < newt)
                        expand = true;
                }

                if(expand){
                    if (newx >= 0 && newx < x_size && newy >= 0 && newy < y_size)
                    {
                        if (((int)map[getMapIndex(newx, newy)] >= 0) && ((int)map[getMapIndex(newx,newy)] < obstacle_cost))  //if free
                        {   
                            float g_s_dash = curr_node->g + (int)map[getMapIndex(newx, newy)] + sqrt(dX[dir]*dX[dir] + dY[dir]*dY[dir]);
                            float h_s_dash = estimateEuclideanDistance(newx, newy, goal_node.x, goal_node.y);
                            Nodeptr successor = std::make_shared<Node>(newx, newy);
                            successor->parent = curr_node;
                            successor->g = g_s_dash;
                            successor->h = h_s_dash;
                            successor->f = g_s_dash + h_s_dash;
                            open_list.push(successor);
                        }
                    }
                }
            }
        }
            
    }
    
    states_expanded = closed_list.size();
    int pathLength = 0;
    geometry_msgs::PoseArray best_path;
    best_path.poses.reserve(1000);
    
    geometry_msgs::Pose last_wp_n2, last_wp_n1;
    if(pathFound){
        ROS_INFO("Backtracking..");
        pathLength = 0;
        Nodeptr backtrackNode = closed_list.at(goal_key);
        while(backtrackNode->parent != NULL){
            geometry_msgs::Pose wp;
            if(pathLength <= 1){
                wp.position.x = (backtrackNode->x + x_offset)*map_resolution;
                wp.position.y = (backtrackNode->y + y_offset)*map_resolution;
                wp.position.z = 0;
                best_path.poses.push_back(wp);
                if(pathLength == 0)
                    last_wp_n2 = wp;
                else
                    last_wp_n1 = wp;
                pathLength++;
            }
            else{

                wp.position.x = (backtrackNode->x + x_offset)*map_resolution;
                wp.position.y = (backtrackNode->y + y_offset)*map_resolution;
                wp.position.z = 0;

                float prev_dir = atan2(last_wp_n1.position.y - last_wp_n2.position.y, last_wp_n1.position.x - last_wp_n2.position.x);
                float curr_dir = atan2(wp.position.y - last_wp_n1.position.y, wp.position.x - last_wp_n1.position.x);
                float dist = sqrt(std::pow(wp.position.x - last_wp_n1.position.x, 2) + std::pow(wp.position.y - last_wp_n1.position.y, 2));
                if(abs(wrap2PI(prev_dir - curr_dir)) > 0.25 || dist > 8){
                    best_path.poses.push_back(wp);
                    last_wp_n2 = last_wp_n1;
                    last_wp_n1 = wp;
                    pathLength++;
                }

            }    
            backtrackNode = backtrackNode->parent;
        }
        plan.poses = best_path.poses;
        std::reverse(plan.poses.begin(), plan.poses.end());
    }
    return pathLength;

}