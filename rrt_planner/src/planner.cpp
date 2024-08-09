#include "rrt_planner/planner.hpp"

namespace rrt_planner
{
CRRTPlanner::CRRTPlanner(const ros::NodeHandle& f_node_handle, int max_count, int delta_max, Point2d start,
                         bool goalReached, Point2d goal, int goal_tolerance, int epsilon)
  : m_node_handle(f_node_handle)
  , rrt(m_node_handle, max_count, delta_max, start, goalReached, goal, goal_tolerance, epsilon)
{
  ROS_INFO_STREAM("Constructor");
  counter = 0;

  // start.x = 6;
  // start.y = 10;
  // start.id = 1;
  // start.parent_id = -1;  // Parent id for starting node doesnt exist

  // goal.x = 10;
  // goal.y = 18;
  // goal.id = 0;
  // goal.parent_id = 0;

  // RRT rrt(5, 4, start, false, goal, 4);
}
// void CRRTPlanner::publishCounter()
// {
//   counter++;
//   std_msgs::Int64 new_msg;
//   new_msg.data = counter;
//   pub.publish(new_msg);
// }

void CRRTPlanner::rrtPlanner()
{
  srand((unsigned int)time(0));

  double no_of_collision;
  ros::param::get("/no_of_collision", no_of_collision);

  // ROS_INFO_STREAM("number: "<<no_of_collision);

  rrt.generateCollisionPts(no_of_collision);

  ROS_INFO_STREAM("array with collision points location");

  rrt.print2DArray(20);
  // rrt.fill2DArray(20);
  // print2DArray(ar, 20);

  // rrt.addPoint(start, ar, 100);
  // std::cout << "After start added\n";

  // for (int i = 1; i < 30; i++)
  // {
  //   rrt.findRandomPoint(start);
  // }

  rrt.generatePath();

  ROS_INFO_STREAM("Entire array filled with all points");
  rrt.print2DArray(20);

  // rrt.printNodes();
  ROS_INFO_STREAM("Path from goal to start");
  rrt.printPath();
}

void CRRTPlanner::callback_number(const std_msgs::Int64& msg)
{
  counter += msg.data;
  std_msgs::Int64 new_msg;
  new_msg.data = counter;
  pub.publish(new_msg);
  rrtPlanner();
}

}  // namespace rrt_planner