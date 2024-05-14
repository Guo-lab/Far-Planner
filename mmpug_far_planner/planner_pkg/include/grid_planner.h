#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <nav_msgs/OccupancyGrid.h>
#include <tf2/utils.h>


#include<queue>
#include<unordered_map>
#include<ros/ros.h>
#include<cmath>


#if !defined(MIN)
#define	MIN(A, B) ((A) < (B) ? (A) : (B))
#endif


struct Node {
    int key, time;
    int x, y, theta;
    float g, h, f;
    std::shared_ptr<Node> parent;

    Node(){}
    Node(int x, int y):x(x),y(y){}

    //Equals Comparision Operator for unordered map
    bool operator==(const Node &n) const { return n.x == x && n.y == y; }
};

// Less than operator for priority queue
struct CompareFValues {
    bool operator()(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2) {
        return n1->f > n2->f;
    }
};


typedef std::shared_ptr<Node> Nodeptr;

//Defining a custom closed list of type unordered_map to Store visited nodes.
typedef std::unordered_map<int, Nodeptr> CLOSED_LIST; 

//Defining an open list of type priority queue
typedef std::priority_queue<Nodeptr, std::vector<Nodeptr>, CompareFValues> OPEN_LIST;



class GridPlanner {
    public:
        GridPlanner(int map_size, float map_resolution, int min_obs_cost);
        GridPlanner(){};
        ~GridPlanner();
        
        int naivePlanner(const geometry_msgs::Pose& robot_pose, const geometry_msgs::Pose& target, geometry_msgs::PoseArray& plan);
        void updateMap(const nav_msgs::OccupancyGrid& grid, geometry_msgs::Point& corner1);
        void getMap(std::vector<int8_t>& map_data);
        void printInfo();
        
        int states_expanded;

    private:
        int x_size, y_size, x_offset, y_offset;
        int max_steps;        
        float map_resolution;
        float obstacle_cost;
        float robot_z;
        bool goal_updated;
        
        int dX[8] = {-1, -1, -1,  0,  0,  1, 1, 1};
        int dY[8] = {-1,  0,  1, -1,  1, -1, 0, 1};

        Node start_node, goal_node;

        std::vector<int8_t, std::allocator<int8_t>> map;

        float estimateOctileDistance(int curr_x, int curr_y, int goal_x, int goal_y);
        float estimateEuclideanDistance(int curr_x, int curr_y, int goal_x, int goal_y);

        int computeKey(int, int);
        int getMapIndex(int, int);

        bool reachedGoal(std::shared_ptr<Node>);

        inline float wrap2PI(float angle){
            angle = fmod(angle, 2 * M_PI);
            if (angle >= M_PI) {
                angle -= 2 * M_PI;
            }
            if(angle < -M_PI) {
                angle += 2 * M_PI;
            }
            return angle;
        }  

};