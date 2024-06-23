// #include <pluginlib/class_list_macros.h>
// #include "global_planner.h"

// PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

// namespace global_planner {

// GlobalPlanner::GlobalPlanner (){

// }

// GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
//     initialize(name, costmap_ros);
// }

// void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

// }

// bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

//         plan.push_back(start);

//         plan.push_back(goal);

//         return true;
//     }
// };

// #include <pluginlib/class_list_macros.h>
// #include "global_planner.h"

// PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

// namespace global_planner {

// GlobalPlanner::GlobalPlanner() : costmap_(nullptr), initialized_(false) {}

// GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : costmap_(nullptr), initialized_(false) {
//     initialize(name, costmap_ros);
// }

// void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
//     if (!initialized_) {
//         costmap_ = costmap_ros->getCostmap();
//         initialized_ = true;
//     }
// }

// bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
//     if (!initialized_) {
//         ROS_ERROR("GlobalPlanner has not been initialized");
//         return false;
//     }

//     plan.clear();

//     unsigned int start_x, start_y, goal_x, goal_y;
//     if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
//         !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
//         ROS_WARN("The start or goal is out of the costmap bounds");
//         return false;
//     }

//     std::vector<std::vector<double>> distances(costmap_->getSizeInCellsX(), std::vector<double>(costmap_->getSizeInCellsY(), std::numeric_limits<double>::infinity()));
//     std::vector<std::vector<geometry_msgs::Point>> predecessors(costmap_->getSizeInCellsX(), std::vector<geometry_msgs::Point>(costmap_->getSizeInCellsY(), geometry_msgs::Point()));

//     std::queue<std::pair<unsigned int, unsigned int>> q;
//     distances[start_x][start_y] = 0;
//     q.push({start_x, start_y});

//     const int dx[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
//     const int dy[8] = {0, 0, -1, 1, -1, 1, -1, 1};

//     while (!q.empty()) {
//         auto current = q.front();
//         q.pop();
//         unsigned int x = current.first;
//         unsigned int y = current.second;

//         for (int i = 0; i < 8; ++i) {
//             unsigned int nx = x + dx[i];
//             unsigned int ny = y + dy[i];
//             if (nx >= costmap_->getSizeInCellsX() || ny >= costmap_->getSizeInCellsY()) continue;

//             double new_cost = distances[x][y] + costmap_->getCost(nx, ny);
//             if (new_cost < distances[nx][ny]) {
//                 distances[nx][ny] = new_cost;
//                 predecessors[nx][ny].x = x;
//                 predecessors[nx][ny].y = y;
//                 q.push({nx, ny});
//             }
//         }
//     }

//     if (distances[goal_x][goal_y] == std::numeric_limits<double>::infinity()) {
//         ROS_WARN("No valid path found");
//         return false;
//     }

//     std::vector<geometry_msgs::PoseStamped> reverse_plan;
//     unsigned int cx = goal_x;
//     unsigned int cy = goal_y;

//     while (cx != start_x || cy != start_y) {
//         geometry_msgs::PoseStamped pose;
//         pose.header.stamp = ros::Time::now();
//         pose.header.frame_id = costmap_->getGlobalFrameID();
//         double wx, wy;
//         costmap_->mapToWorld(cx, cy, wx, wy);
//         pose.pose.position.x = wx;
//         pose.pose.position.y = wy;
//         pose.pose.position.z = 0.0;
//         pose.pose.orientation.w = 1.0;
//         reverse_plan.push_back(pose);

//         geometry_msgs::Point pred = predecessors[cx][cy];
//         cx = static_cast<unsigned int>(pred.x);
//         cy = static_cast<unsigned int>(pred.y);
//     }

//     geometry_msgs::PoseStamped start_pose = start;
//     start_pose.header.stamp = ros::Time::now();
//     reverse_plan.push_back(start_pose);

//     plan.assign(reverse_plan.rbegin(), reverse_plan.rend());
//     return true;
// }

// };  // namespace global_planner

#include <pluginlib/class_list_macros.h>
#include "global_planner.h"

// Includes the necessary pluginlib macros for registering the planner as a plugin, and includes the header file for this planner.

PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)
// Registers the GlobalPlanner class as a plugin, making it available to be used by the ROS navigation stack.

namespace global_planner
{

    GlobalPlanner::GlobalPlanner() : costmap_(nullptr), initialized_(false) {}
    // Default constructor sets the value of private class variables

    GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros) : costmap_(nullptr), initialized_(false)
    {
        initialize(name, costmap_ros);
    }

    void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {
            costmap_ = costmap_ros->getCostmap(); // This is the method of costmap_2D class
            global_frame_ = costmap_ros->getGlobalFrameID();
            initialized_ = true;
        }
    }

    bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
    {
        if (!initialized_)
        {
            ROS_ERROR("GlobalPlanner has not been initialized");
            return false;
        }

        plan.clear(); // clears any previous path

        unsigned int start_x, start_y, goal_x, goal_y;
        if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
            !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y))
        {
            ROS_WARN("The start or goal is out of the costmap bounds");
            return false;
        }
        // Converts the start and goal positions from world coordinates to map coordinates. If either conversion fails, logs a warning and returns false

        std::vector<std::vector<double>> distances(costmap_->getSizeInCellsX(), std::vector<double>(costmap_->getSizeInCellsY(), std::numeric_limits<double>::infinity()));

        std::vector<std::vector<geometry_msgs::Point>> predecessors(costmap_->getSizeInCellsX(), std::vector<geometry_msgs::Point>(costmap_->getSizeInCellsY(), geometry_msgs::Point()));

        /*
         Initializes two 2D vectors :

            distances : Stores the shortest distance to each cell,
                       initialized to infinity.predecessors : Stores the predecessor of each cell,
                     initialized to default geometry_msgs::Point.


        */

        std::queue<std::pair<unsigned int, unsigned int>> q;

        distances[start_x][start_y] = 0;
        q.push({start_x, start_y});

        const int dx[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
        const int dy[8] = {0, 0, -1, 1, -1, 1, -1, 1};

        while (!q.empty())
        {
            auto current = q.front();
            q.pop();
            unsigned int x = current.first;
            unsigned int y = current.second;

            for (int i = 0; i < 8; ++i)
            {
                unsigned int nx = x + dx[i];
                unsigned int ny = y + dy[i];
                if (nx >= costmap_->getSizeInCellsX() || ny >= costmap_->getSizeInCellsY())
                    continue;

                double new_cost = distances[x][y] + costmap_->getCost(nx, ny);
                if (new_cost < distances[nx][ny])
                {
                    distances[nx][ny] = new_cost;
                    predecessors[nx][ny].x = x;
                    predecessors[nx][ny].y = y;
                    q.push({nx, ny});
                }
            }
        }

        if (distances[goal_x][goal_y] == std::numeric_limits<double>::infinity())
        {
            ROS_WARN("No valid path found");
            return false;
        }

        std::vector<geometry_msgs::PoseStamped> reverse_plan;
        unsigned int cx = goal_x;
        unsigned int cy = goal_y;

        while (cx != start_x || cy != start_y)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = global_frame_;
            double wx, wy;
            costmap_->mapToWorld(cx, cy, wx, wy);
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            reverse_plan.push_back(pose);

            geometry_msgs::Point pred = predecessors[cx][cy];
            cx = static_cast<unsigned int>(pred.x);
            cy = static_cast<unsigned int>(pred.y);
        }

        geometry_msgs::PoseStamped start_pose = start;
        start_pose.header.stamp = ros::Time::now();
        reverse_plan.push_back(start_pose);

        plan.assign(reverse_plan.rbegin(), reverse_plan.rend());
        return true;
    }

}; // namespace global_planner
