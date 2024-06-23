// #include <ros/ros.h>
// #include <costmap_2d/costmap_2d_ros.h>
// #include <costmap_2d/costmap_2d.h>
// #include <nav_core/base_global_planner.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <angles/angles.h>
// #include <base_local_planner/world_model.h>
// #include <base_local_planner/costmap_model.h>

// using std::string;

// #ifndef GLOBAL_PLANNER_CPP
// #define GLOBAL_PLANNER_CPP

// namespace global_planner {

//     class GlobalPlanner : public nav_core::BaseGlobalPlanner {
//         public:

//         GlobalPlanner();
//         GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

//         void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
//         bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
//         std::vector<geometry_msgs::PoseStamped>& plan);
//     };
// };
// #endif

// #ifndef GLOBAL_PLANNER_CPP
// #define GLOBAL_PLANNER_CPP

// #include <ros/ros.h>
// #include <costmap_2d/costmap_2d_ros.h>
// #include <costmap_2d/costmap_2d.h>
// #include <nav_core/base_global_planner.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <angles/angles.h>
// #include <base_local_planner/world_model.h>
// #include <base_local_planner/costmap_model.h>
// #include <vector>
// #include <queue>
// #include <limits>

// using std::string;

// namespace global_planner {

//     class GlobalPlanner : public nav_core::BaseGlobalPlanner {
//         public:

//         GlobalPlanner();
//         GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

//         void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
//         bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
//         std::vector<geometry_msgs::PoseStamped>& plan);

//         private:
//         costmap_2d::Costmap2D* costmap_;
//         bool initialized_;
//     };
// };

// #endif

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <vector>
#include <queue>
#include <limits>

using std::string;

namespace global_planner
{ // Defines a namespace global_planner to avoid name conflicts with other parts of the program or other libraries.

    class GlobalPlanner : public nav_core::BaseGlobalPlanner
    // nav_core::BaseGlobalPlanner is the class in ros from which we are doing public inheritance
    {
    public:
        GlobalPlanner();
        GlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        // initializes the planner with a name and a costmap

        bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan);
        //   Generates a plan from 'start' to 'goal' , storing the resulting path in 'plan'

    private:
        costmap_2d::Costmap2D *costmap_;
        bool initialized_;
        std::string global_frame_;
    };
};

#endif
