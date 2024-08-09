#ifndef RRT_PLANNER_HPP
#define RRT_PLANNER_HPP

// #include <iostream>
#include <ctime>

#include <random>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Int64.h>

#include "rrt_planner/visualisation.hpp"

namespace rrt_planner
{
struct Point2d
{
  int x;
  int y;
  int id;
  int parent_id;
  bool parent = false;
};

class RRT
{
public:
  RRT(const ros::NodeHandle& f_node_handle, int max_count, int delta_max, Point2d start, bool goalReached, Point2d goal,
      int goal_tolerance);

  // fills 20x20 2D array with random #s (1-100)
  void fill2DArray(int numRows);

  // prints array
  void print2DArray(int numRows);

  // Function to create collision points
  void generateCollisionPts(int num_collisionpts);

  // Function to check if collision exits
  bool isInCollision(Point2d& parent, Point2d& new_point);

  // Function to add point in space
  void addPoint(Point2d& pt, int val);

  // Euclidean distance
  double distance(Point2d& pt1, Point2d& pt2);

  // Find the nearest point
  int findParent(Point2d& pt);

  // Function to add point in space within the max_delta
  void updateNewPoint(Point2d& pt1, Point2d& parent, int val);

  bool isDuplicate(Point2d& new_node);

  // Function to add point in space
  void findRandomPoint();

  // To check if goal is reached
  bool isGoalReached();

  void generatePath();

  void printPath();

  // Print Node Vector
  void printNodes();

  // void rrtPlanner();

private:
  int m_ar[20][20];
  int m_delta_max = 4;
  int m_count;
  int m_max_count;
  std::vector<Point2d> m_nodes;
  bool m_goalReached;
  Point2d m_goal;
  int m_goal_tolerance;
  int m_iterations;
  std::vector<Point2d> m_collision_nodes;

  ros::NodeHandle m_node_handle;

  CVisualisation m_waypoint_visual;
};

}  // namespace rrt_planner
#endif