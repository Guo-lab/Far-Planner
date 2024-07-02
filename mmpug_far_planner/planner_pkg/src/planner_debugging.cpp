#include "grid_planner.h"

using namespace std;


//============================ Debugging and Testing ================================
//============================
/**
 * @brief Log the information of the GridPlanner object.
 */
void GridPlanner::PrintInfo() {
    ROS_INFO_STREAM("Map size: " << map.size());
    ROS_INFO_STREAM("Robot Position: (" << start_node.x << ", " << start_node.y << ")");
    ROS_INFO_STREAM("x_offset: " << x_offset << " y_offset: " << y_offset);
}

