#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <ros/ros.h>
#include <std_msgs/Int64.h>

#include "rrt_planner/rrt_star_planner.hpp"

namespace rrt_planner
{
class CRRTPlanner
{
public:
  CRRTPlanner(const ros::NodeHandle& f_node_handle, int max_count, int delta_max, Point2d start, bool goalReached,
              Point2d goal, int goal_tolerance, int epsilon);

  void publishCounter();

  void callback_number(const std_msgs::Int64& msg);
  void rrtPlanner();

private:
  ros::NodeHandle m_node_handle;
  ros::Publisher pub = m_node_handle.advertise<std_msgs::Int64>("/number_count", 10);
  ros::Subscriber number_subscriber = m_node_handle.subscribe("/number", 1000, &CRRTPlanner::callback_number, this);
  int counter;

  RRT rrt;
  Point2d start;
  Point2d goal;
};

}  // namespace rrt_planner
#endif