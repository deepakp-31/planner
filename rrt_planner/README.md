# 1. Package Summary
*rrt_planner* is a simple planner using rrt algorithm package. The package contains scripts for visualising the path. 

* *Maintainer(s)*:  Deepak P 

# 2. Nodes

## 2.1. rrt_planner_node
The rrt_planner_node publishes visualisation markers for visualising the path.

# 3. Package Installation & Dependencies
## 3.1. Prerequisites

- ROS noetic
- ubuntu focal 20.04

## 3.2. Install Dependencies
```bash
rosdep install --from-paths src --ignore-src -r -y 
```

# 4. Build
```bash
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
or 
```bash
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```
or Minimal build for rrt_planner
```bash
catkin build rrt_planner
```

# 5. Launch Planner

**1. Source the workspace**

After build, source the proper *setup.bash* file if it is not added in .bashrc

```bash
source ~/catkin_ws/devel/setup.bash
```

**2. Run ROS Master**
```bash
roscore
```

**3. Launch Planner visualisation**
Now run the following file to launch the visualization:

```bash
roslaunch rrt_planner planner.launch no_of_collision:=<number> goalx:=<number> goaly:=<number> startx:=<number> starty:=<number>

```
eg

```bash
roslaunch rrt_planner planner.launch no_of_collision:=20

```
OR
```bash
roslaunch rrt_planner planner.launch no_of_collision:=30 goalx:=6 goaly:=19 startx:=8 starty:=3
```

Please give goal & start x and y in range (0,20)

**4. Trigger the planner**
Send message to trigger the planner:

```bash
rostopic pub -1 /number std_msgs/Int64 "data: 100"

```



# 6. Notes

-  The launch would have to be relaunched everytime a trigger is sent. Some error is there in the subscriber node
- RRT is implemented and visualised in Rviz using visualisation markers.
- Obstacles are generated at random.
- Incomplete RRT* was implemented.
- Start and goal positions and number of obstacles could be sent as parameters via launch command else default value will be taken

