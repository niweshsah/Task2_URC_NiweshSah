## Prerequisites

Before you can run the node, make sure you have the following installed:
- [ROS (Robot Operating System)](http://wiki.ros.org/ROS/Installation)
- [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)



**NOTE:** We are using ROS Noetic

## Installation

1.  **Make workspace (e.g. catkin_ws)**

     ```bash
    cd ~
    mkdir catkin_ws
    cd catkin_ws/
    
    ```

2.  **Clone the Repository**

    ```bash
    git clone <repository-url>
    ```

3. **Build the Package**

    Ensure you are in the workspace root directory (e.g., `catkin_ws`).

    ```bash
    catkin init
    catkin build
    ```
4. **Sourcing and setting the environment variables**:

     Type " nano ~/.bashrc " in terminal and add following lines at the end:

     ```bash
    source /opt/ros/noetic/setup.bash
     export TURTLEBOT3_MODEL=burger
     source ~/catkin_ws/devel/setup.bash
     ```

     Then, open terminal:
     ```bash
    source ~/.bashrc
    ```
        
6.  **Move the map.yaml at home directory**
    
    ```bash
    mv ~/catkin_ws/map.yaml ~/
    ```

## Launching Autonomous Navigation Nodes Using Custom Global Planner

To run the node, use the following command:

```bash
rosrun main_launch python_start_launches.py 
```
This will start 4 terminals:
1. Rosmaster
2. Gazebo world simulation with turtlebot3
3. RVIZ
4. Teleoperation node

## Initial Pose Estimation 

1. Click the 2D Pose Estimate button in the RViz menu.
2. Click on the map where the actual robot is located and drag the large green arrow toward the direction where the robot is facing.
3. Repeat step 1 and 2 until the LDS sensor data is overlayed on the saved map.
4. Use keyboard teleoperation node to precisely locate the robot on the map.
5. Move the robot back and forth a bit to collect the surrounding environment information and narrow down the estimated location of the TurtleBot3 on the map which is displayed with tiny green arrows.
6. Terminate the keyboard teleoperation node by entering Ctrl + C to the teleop node terminal in order to prevent different cmd_vel values are published from multiple nodes during Navigation.

## Set Navigation Goal
1. Click the 2D Nav Goal button in the RViz menu.
2. Click on the map to set the destination of the robot and drag the green arrow toward the direction where the robot will be facing.
.  This green arrow is a marker that can specify the destination of the robot.
.  The root of the arrow is x, y coordinate of the destination, and the angle θ is determined by the orientation of the arrow.
.  As soon as x, y, θ are set, TurtleBot3 will start moving to the destination immediately.


## Explanation of the Global Planner Algorithm

### Modified Breadth-First Search (BFS) for Path Planning

The modified BFS algorithm used for the global planner is designed to efficiently find the shortest path from the start to the goal position in a grid-based map. Here's a step-by-step explanation:

1. **Initialization**:
   - Initialize a queue and add the start node to it.
   - Mark the start node as visited.

2. **Exploration**:
   - While the queue is not empty, remove the front node from the queue.
   - For each neighbor of the current node:
     - If the neighbor is the goal node, terminate the search and reconstruct the path.
     - If the neighbor has not been visited and is not an obstacle, mark it as visited and add it to the queue.

3. **Path Reconstruction**:
   - Backtrack from the goal node to the start node to reconstruct the path using the parent pointers stored during the exploration phase.

### Comparison with Dijkstra's Algorithm

| Feature                      | Modified BFS                                   | Dijkstra's Algorithm                          |
|------------------------------|------------------------------------------------|-----------------------------------------------|
| **Type**                     | Uninformed search algorithm                    | Informed search algorithm                     |
| **Cost Consideration**       | Assumes uniform cost (all edges have the same cost) | Considers the actual cost of edges            |
| **Data Structure**           | Queue (FIFO)                                   | Priority queue (min-heap)                     |
| **Time Complexity**          | O(V + E)                                       | O((V + E) log V)                              |
| **Space Complexity**         | O(V)                                           | O(V)                                          |
| **Path Optimality**          | Guarantees shortest path for uniform cost grids| Guarantees shortest path for any cost grid    |
| **Usage Scenario**           | Suitable for grids with uniform cost           | Suitable for grids with varying edge costs    |

**Modified BFS** is efficient and straightforward for uniform cost grids, making it suitable for many robotic navigation tasks. It explores all nodes at the present depth level before moving on to nodes at the next depth level, ensuring the shortest path is found in an unweighted grid.

**Dijkstra's Algorithm** is more versatile as it accounts for varying edge costs, making it ideal for environments where the cost of movement between nodes differs. It uses a priority queue to explore nodes in the order of their cumulative cost from the start node.

## Creating a Custom Global Planner from Scratch

1. **Creating new package**

    Delete the existing global_planner package and make a new one using following command:
     ```bash
     cd ~/catkin_ws/src/
     rm -rf global_planner
     catkin_create_pkg global_planer roscpp nav_core nav_msgs tf tf2 std_msgs sensor_msgs geometry msgs 
    ```

2. **Build the workspace**

    ```bash
     cd ~/catkin_ws/
     catkin build
    ```

3. **Creating new folder**

    ```bash
     cd ~/catkin_ws/src/global_planner/
     mkdir plugins
    ```

     It is recommended that you put your global planner C++ codes inside this folder.

4. **Writing C++ code for Global Planner**

    Ensure all the command line commands are visible in the recording to help others follow along.

   

## Troubleshooting

If you encounter any issues, check the following:
- Ensure all dependencies are installed.
- Verify the nodes are being run in the correct environment.
- Check ROS logs for error messages.

```bash
roslaunch global_planner global_planner.launch
```

> **Note:** If you have any custom configurations or environment variables set, ensure they are properly configured before running the nodes.

## Contributing

If you would like to contribute, please fork the repository and create a pull request with your changes.

## License

Include information about the licensing of your project.

---

Feel free to modify this template according to your specific requirements and ensure all details relevant to your setup are included.
