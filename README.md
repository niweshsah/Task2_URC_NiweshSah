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


## Setting Up TurtleBot3

1. **Install TurtleBot3 Packages**

    ```bash
    sudo apt-get install ros-<ros-distro>-turtlebot3
    ```

2. **Set Up Environment Variables**

    Add the following lines to your `.bashrc` file:

    ```bash
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
    source ~/.bashrc
    ```

## Launching Nodes for Autonomous Navigation Using Custom Planner

### 1. Start the ROS Master

```bash
roscore
```

### 2. Launch TurtleBot3 Simulation (if using a simulator)

In a new terminal, run:

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### 3. Launch SLAM for Mapping and Localization

In another terminal, run:

```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

### 4. Launch the Global Planner Node

In another terminal, run:

```bash
rosrun global_planner global_planner_node
```

### 5. Launch RViz for Visualization

In another terminal, run:

```bash
roslaunch turtlebot3_slam turtlebot3_slam_rviz.launch
```

### 6. Send Navigation Goals

Use `rviz` to send navigation goals by setting the `2D Nav Goal` in RViz.

## Usage

The `global_planner` node will handle the path planning for the TurtleBot3. Use RViz to visualize the robot's environment and set navigation goals.

## Explanation of the Algorithm

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

## Creating a Demonstration Video

1. **Record the Terminal Commands**

    Use a screen recording tool (e.g., OBS Studio) to capture the terminal commands and the ROS nodes in action.

2. **Demonstrate the Algorithm**

    Show the following in your video:
    - Starting the ROS master.
    - Launching the TurtleBot3 simulation.
    - Launching the SLAM node.
    - Running the `global_planner` node.
    - Setting navigation goals in RViz.
    - The TurtleBot3 navigating autonomously to the goals.

3. **Include Command Line Commands**

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
