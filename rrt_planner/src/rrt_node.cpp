#include "rrt_planner/planner.hpp"

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "rrt_planner_node");
  ros::NodeHandle node_handle;

  // struct Point2d
  // {
  //   int x;
  //   int y;
  //   int id;
  //   int parent_id;
  //   bool parent = false;
  // };

  rrt_planner::Point2d start;

  ros::param::get("/startx", start.x);
  ros::param::get("/starty", start.y);
  // start.x = 2;
  // start.y = 4;
  start.id = 1;
  start.parent_id = -1;  // Parent id for starting node doesnt exist
  start.cost =0;
  start.cost_from_start =0;

  rrt_planner::Point2d goal;
  ros::param::get("/goalx", goal.x);
  ros::param::get("/goaly", goal.y);
  // goal.x = 10;
  // goal.y = 18;
  goal.id = 0;
  goal.parent_id = 0;

  rrt_planner::CRRTPlanner rrtplan(node_handle, 5, 4, start, false, goal, 4, 5);

  ros::Rate loop_rate(10);

  // while (ros::ok())
  // {
  //   ros::spinOnce();
  //   rrtplan.publishCounter();
  //   loop_rate.sleep();
  // }

  // ros::spinOnce();
  ros::spin();

  return 0;
}