# Path-Planning-ROS
This package is developed as part of a ROS (Robot Operating System) project for path planning. It includes implementations of A* (A star), Dijkstra, and Greedy algorithms for path planning in robotic applications.

## Explanation 
In developing the path planning package, certain important assumptions have been made regarding the capabilities and environment of the robot. Firstly, it is assumed that the robot possesses a pre-existing map of its environment, allowing it to navigate effectively. Additionally, the robot is assumed to have accurate knowledge of its own location within this map, facilitating precise path planning. Another crucial assumption is that the environment in which the robot operates remains static and does not undergo any significant changes during the path planning process. Finally, the robot is assumed to have the ability to move in four cardinal directions (up, down, left, or right) within the map, provided that the corresponding spaces are unobstructed. These assumptions form the basis for the design and implementation of the path planning algorithms within the package. 

![rviz_robot_top_view](https://github.com/peakyquest/Path-Planning-ROS/assets/162409782/72b099d6-8bce-4c9f-9714-3f40e79a091e)



## Installation
To use this package, follow these steps:
1. Clone this repository into your ROS workspace's src directory:
```
git clone <repository_url>
```

2. Build the package using catkin_make:

```
cd <path_to_your_ros_workspace>
catkin_make
```

3. Launch the simulation world: 

```
roslaunch unit2_pp simulation_unit2.launch
```

4. Execute the path planning launch file: 

```
roslaunch unit2_pp unit2_exercise.launch
```

5. Choose which algorithm you want to use for path planning in the make_plan.py file:

**Example**  
```
from algorithms import dijkstra, a_star, greedy
path = greedy(start_index, goal_index, width, height, costmap, resolution, origin, viz)
```


## Example 
Following is the example of dijkstra algorithm:

![dijkstra_result](https://github.com/peakyquest/Path-Planning-ROS/assets/162409782/99db0d31-2d14-4299-9371-87d147f474b4)

