#ifndef VISUALISATION_HPP
#define VISUALISATION_HPP

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

namespace rrt_planner
{
class CVisualisation
{
public:
  CVisualisation(const ros::NodeHandle& f_node_handle);

  void updateVisual(const geometry_msgs::Point& f_path_waypoint, int f_type_of_pt);

  void updateVisualPath(const geometry_msgs::Point& f_path_waypoint);

private:
  ros::NodeHandle m_node_handle;

  visualization_msgs::Marker m_line_strip;
  visualization_msgs::Marker m_rrt_waypoint;
  visualization_msgs::Marker m_rrt_waypoint_obstacle;

  ros::Publisher marker_pub = m_node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 4);

  ros::Time time_begin;
};

}  // namespace rrt_planner
#endif

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "visualisation_node");

//   ros::NodeHandle m_node_handle;

//   CVisualisation waypoint = CVisualisation(m_node_handle);

//   ros::spin();

//   return 0;
// }
