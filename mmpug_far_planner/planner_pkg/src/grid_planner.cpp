/** 
 * @file grid_planner.cpp
 * @brief Grid planner class implementation
 * @author Siqi. Edward, Guo
 * @version 1.0
 * @date 2024
 */
#include "grid_planner.h"
using namespace std;


/**
 * @brief Constructor creates a new GridPlanner.
 * 
 * @param map_size The size of the map grid.
 * @param map_resolution The resolution of the map grid.
 * @param min_obs_cost The minimum obstacle cost, which is 100 for now.
 */
GridPlanner::GridPlanner(int map_size, float map_resolution, int min_obs_cost) 
  : x_size(map_size), y_size(map_size), map_resolution(map_resolution), obstacle_cost(min_obs_cost) {
  this->x_offset = -map_size / 2;
  this->y_offset = -map_size / 2;
  initializeMap();
}
/**
 * @brief Destructor destroys a GridPlanner.
 * 
 * This destructor is responsible for cleaning up the map used by the GridPlanner object.
 * It calls the resetMap() function to reset the stored map.
 */
GridPlanner::~GridPlanner() { resetMap(); }

/**
 * @brief Calculates the map index based on the given x and y coordinates. (This is also the Key in Closed list.)
 * 
 * @param x The x-coordinate.
 * @param y The y-coordinate.
 * @return The map index.
 */
inline int GridPlanner::getMapIndex(int x, int y) { return (y * y_size + x); }


/**
 * @brief Function to estimate the euclidean distance between two points
 *     formula: sqrt(dx^2 + dy^2)
 *  The Octile Distance dx+dy-min(dx, dy) deprecated in this version.
 * 
 * @param curr_x The x-coordinate of the current point.
 * @param curr_y The y-coordinate of the current point.
 * @param goal_x The x-coordinate of the goal point.
 * @param goal_y The y-coordinate of the goal point.
 * @return The estimated euclidean distance between these two points.
 */
auto GridPlanner::estimateEuclideanDistance(int curr_x, int curr_y, int goal_x, int goal_y) -> float {
  int delta_x = abs(curr_x - goal_x);
  int delta_y = abs(curr_y - goal_y);
  return sqrt(delta_x * delta_x + delta_y * delta_y);
}



//============================ Global Cost Map Helper Function ================================
//============================
/**
 * @brief Resets the map used by the grid planner. 
 *  This function clears the map, removing all previously stored data.
 * @return void
 */
void GridPlanner::resetMap() {
  this->map.clear();
  return;
}

/**
 * @brief Initializes the map for the grid planner.
 * 
 * This function resets the map and initializes it with the obstacle costs. 
 * The map is represented as a 2D grid with dimensions x_size and y_size.
 * Each cell in the grid is assigned an obstacle cost equal to half of the minimum obstacle cost 
 * specified in the GridPlanner object. (unexplored areas)
 * 
 * @return void
 */
void GridPlanner::initializeMap() {
  resetMap();
  std::vector<int8_t> data(this->x_size * this->y_size);
  std::fill(data.begin(), data.end(), this->obstacle_cost / 2);
  this->map = data;
  return;
}

/**
 * Check if the given coordinates (x, y) are within the map boundaries.
 *
 * @param x The x-coordinate.
 * @param y The y-coordinate.
 * @return True if the coordinates are within the map boundaries, false otherwise.
 */
auto GridPlanner::isInMap(int x, int y) -> bool {
  return (x >= 0 && x < x_size && y >= 0 && y < y_size);
}

/**
 * @brief Retrieves the map data.
 *
 * This function is used to retrieve the map data stored in the GridPlanner. 
 *  The map data will be put into the vector map_data.
 * 
 * @param map_data The vector to store the retrieved map.
 */
void GridPlanner::getMap(std::vector<int8_t>& map_data) {
  map_data = this->map;
  return;
}






/**
 * @brief Updates the global cost map with the given occupancy grid and origin point.
 * 
 * This function iterates over the grid and checks if each cell is within the map boundaries.
 * Each cell will be updated if it has not been updated before in terms of the local's map. 
 * Then, the cell will only update the obstacle cost when there is an obstacle in it.
 * 
 * For now the filtering function does not work well. 
 * 
 * @param grid The occupancy grid to update the map with.
 * @param origin_point The origin point in local planner's map, used to calculate the map coordinates.
 */
void GridPlanner::updateMap(const nav_msgs::OccupancyGrid& grid, geometry_msgs::Point& origin_point) {
  int x1, y1;

  for (int i = 2; i < grid.info.width - 3; i++)
    for (int j = 2; j < grid.info.height - 3; j++) {
      x1 = std::floor(origin_point.x / map_resolution) - x_offset;
      y1 = std::floor(origin_point.y / map_resolution) - y_offset;
      if (isInMap(x1 + i, y1 + j)) {
        if (this->map[(x1 + i) + ((y1 + j) * y_size)] == obstacle_cost / 2) {
          this->map[(x1 + i) + ((y1 + j) * y_size)] =
              (int8_t)grid.data.at(i + (j * grid.info.height));
        } else {
          if (grid.data.at(i + (j * grid.info.height)) != obstacle_cost / 2 &&
              grid.data.at(i + (j * grid.info.height)) != 0) {
            this->map[(x1 + i) + ((y1 + j) * y_size)] =
                (int8_t)grid.data.at(i + (j * grid.info.height));
          }
        }
      }
    }
}



void GridPlanner::printInfo() {
  ROS_INFO_STREAM("Map size: " << map.size());
  ROS_INFO_STREAM("Robot Position: " << start_node.x << " " << start_node.y);
  ROS_INFO_STREAM("x_offset: " << x_offset << " y_offset: " << y_offset);
}

/**
 * @brief Function to plan in a naive way
 */
auto GridPlanner::naivePlanner(const geometry_msgs::Pose& robot_pose, const geometry_msgs::Pose& target, geometry_msgs::PoseArray& plan) -> int {
  start_node.x = robot_pose.position.x / map_resolution - x_offset;
  start_node.y = robot_pose.position.y / map_resolution - x_offset;

  goal_node.x = target.position.x / map_resolution - x_offset;
  goal_node.y = target.position.y / map_resolution - x_offset;

  if (start_node.x < 0 || start_node.x >= x_size || start_node.y < 0 ||
      start_node.y >= x_size) {
    ROS_WARN("CANCELLING PLANNER BECAUSE ROBOT IS NOT IN THE GRID");
    return 0;
  }
  if (goal_node.x < 0 || goal_node.x >= x_size || goal_node.y < 0 ||
      goal_node.y >= x_size) {
    ROS_WARN("CANCELLING PLANNER BECAUSE GOAL IS NOT IN THE GRID");
    return 0;
  }

  // Setup start node, closed and open lists.
  CLOSED_LIST closed_list;
  OPEN_LIST open_list;

  Nodeptr start = std::make_shared<Node>(start_node.x, start_node.y);
  start->key = getMapIndex(start_node.x, start_node.y);
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
  while (!open_list.empty() && closed_list.count(getMapIndex(goal_node.x, goal_node.y)) == 0) {
    Nodeptr curr_node = open_list.top();
    open_list.pop();
    int curr_key = getMapIndex(curr_node->x, curr_node->y);
    if (closed_list.count(curr_key) == 0) {
      closed_list.insert({curr_key, curr_node});

      if (*curr_node == goal_node) {
        goal_key = curr_key;
        pathFound = true;
        ROS_INFO("Found Path");
        break;
      }

      for (int dir = 0; dir < 8; dir++) {
        int newx = curr_node->x + dX[dir];
        int newy = curr_node->y + dY[dir];
        int newt = curr_node->time - 1;

        bool expand = false;
        if (closed_list.count(getMapIndex(newx, newy)) == 0)
          expand = true;
        else {
          if (closed_list.at(getMapIndex(newx, newy))->time < newt)
            expand = true;
        }

        if (expand) {
          if (newx >= 0 && newx < x_size && newy >= 0 && newy < y_size) {
            if (((int)map[getMapIndex(newx, newy)] >= 0) && ((int)map[getMapIndex(newx, newy)] < obstacle_cost)) {  // if free
              float g_s_dash = curr_node->g + (int)map[getMapIndex(newx, newy)] + sqrt(dX[dir] * dX[dir] + dY[dir] * dY[dir]);
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

  int states_expanded = closed_list.size();
  int pathLength = 0;
  geometry_msgs::PoseArray best_path;
  best_path.poses.reserve(1000);

  geometry_msgs::Pose last_wp_n2, last_wp_n1;
  if (pathFound) {
    ROS_INFO("Backtracking..");
    pathLength = 0;
    Nodeptr backtrackNode = closed_list.at(goal_key);
    while (backtrackNode->parent != NULL) {
      geometry_msgs::Pose wp;
      if (pathLength <= 1) {
        wp.position.x = (backtrackNode->x + x_offset) * map_resolution;
        wp.position.y = (backtrackNode->y + y_offset) * map_resolution;
        wp.position.z = 0;
        best_path.poses.push_back(wp);
        if (pathLength == 0)
          last_wp_n2 = wp;
        else
          last_wp_n1 = wp;
        pathLength++;
      } else {
        wp.position.x = (backtrackNode->x + x_offset) * map_resolution;
        wp.position.y = (backtrackNode->y + y_offset) * map_resolution;
        wp.position.z = 0;

        float prev_dir = atan2(last_wp_n1.position.y - last_wp_n2.position.y, last_wp_n1.position.x - last_wp_n2.position.x);
        float curr_dir = atan2(wp.position.y - last_wp_n1.position.y, wp.position.x - last_wp_n1.position.x);
        float dist = sqrt(std::pow(wp.position.x - last_wp_n1.position.x, 2) + std::pow(wp.position.y - last_wp_n1.position.y, 2));
        if (abs(wrap2PI(prev_dir - curr_dir)) > 0.25 || dist > 8) {
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