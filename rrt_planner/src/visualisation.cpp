#include "rrt_planner/visualisation.hpp"

namespace rrt_planner
{
CVisualisation::CVisualisation(const ros::NodeHandle& f_node_handle) : m_node_handle(f_node_handle)
{
  ROS_DEBUG_STREAM("Visualization");

  m_line_strip.header.frame_id = "my_frame";
  m_line_strip.header.stamp = ros::Time::now();

  // Any marker sent with the same namespace and id will overwrite the old one
  m_line_strip.ns = "path";
  m_line_strip.id = 1;  // id given at random

  // Type of marker
  m_line_strip.action = visualization_msgs::Marker::ADD;
  m_line_strip.type = visualization_msgs::Marker::LINE_STRIP;

  // Setting pose & sclae of the marker
  // A scale of [1,1,1] means the object will be 1m by 1m by 1m.
  // m_line_strip markers use only the x component of scale, for the line width
  m_line_strip.pose.orientation.w = 1.0;
  m_line_strip.scale.x = 0.05;

  // yellow color set for depicting path using m_line_strip
  m_line_strip.color.r = 1.0;
  m_line_strip.color.g = 1.0;
  m_line_strip.color.b = 0.0;
  m_line_strip.color.a = 1.0;

  // path will be visible for 1000 seconds
  m_line_strip.lifetime = ros::Duration(1000);

  m_rrt_waypoint.header.frame_id = "my_frame";
  m_rrt_waypoint.header.stamp = ros::Time();

  m_rrt_waypoint.ns = "waypoint";
  m_rrt_waypoint.id = 2;  // id given at random

  // Type of marker
  m_rrt_waypoint.type = visualization_msgs::Marker::POINTS;
  m_rrt_waypoint.action = visualization_msgs::Marker::ADD;

  // Setting pose & sclae of the marker
  // m_points markers use x and y scale for width/height respectively
  m_rrt_waypoint.pose.orientation.w = 1.0;
  m_rrt_waypoint.scale.x = 0.1;
  m_rrt_waypoint.scale.y = 0.1;
  m_rrt_waypoint.scale.z = 0.1;

  // Blue
  // m_rrt_waypoint.color.b = 1.0f;
  // m_rrt_waypoint.color.g = 0.0f;
  // m_rrt_waypoint.color.r = 0.0f;
  // m_rrt_waypoint.color.a = 1.0f;

  m_rrt_waypoint_obstacle.header.frame_id = "my_frame";
  m_rrt_waypoint_obstacle.header.stamp = ros::Time();

  m_rrt_waypoint_obstacle.ns = "obstacle";
  m_rrt_waypoint_obstacle.id = 3;  // id given at random

  // Type of marker
  m_rrt_waypoint_obstacle.type = visualization_msgs::Marker::POINTS;
  m_rrt_waypoint_obstacle.action = visualization_msgs::Marker::ADD;

  // Setting pose & sclae of the marker
  // m_points markers use x and y scale for width/height respectively
  m_rrt_waypoint_obstacle.pose.orientation.w = 1.0;
  m_rrt_waypoint_obstacle.scale.x = 0.1;
  m_rrt_waypoint_obstacle.scale.y = 0.1;
  m_rrt_waypoint_obstacle.scale.z = 0.1;

  m_line_strip.points.clear();
  m_rrt_waypoint.points.clear();
  m_rrt_waypoint_obstacle.points.clear();

  // Stroke path will be visible for 10 seconds
  m_rrt_waypoint.lifetime = ros::Duration(1000);

  time_begin = ros::Time::now();
}

void CVisualisation::updateVisual(const geometry_msgs::Point& f_path_waypoint, int f_type_of_pt)
{
  // ROS_INFO_STREAM("Waypoint");

  // if (f_type_of_pt == 0)
  // {
  //   m_rrt_waypoint.color.b = 1.0f;
  //   m_rrt_waypoint.color.g = 0.0f;
  //   m_rrt_waypoint.color.r = 0.0f;
  //   m_rrt_waypoint.color.a = 1.0f;
  // }

  if (f_type_of_pt == 1)
  {
    m_rrt_waypoint_obstacle.color.b = 0.0f;
    m_rrt_waypoint_obstacle.color.g = 0.0f;
    m_rrt_waypoint_obstacle.color.r = 1.0f;
    m_rrt_waypoint_obstacle.color.a = 1.0f;

    m_rrt_waypoint_obstacle.points.push_back(f_path_waypoint);

    marker_pub.publish(m_rrt_waypoint_obstacle);
  }

  if (f_type_of_pt == 2)
  {
    m_rrt_waypoint.color.b = 0.0f;
    m_rrt_waypoint.color.g = 1.0f;
    m_rrt_waypoint.color.r = 0.0f;
    m_rrt_waypoint.color.a = 1.0f;

    m_rrt_waypoint.points.push_back(f_path_waypoint);

    marker_pub.publish(m_rrt_waypoint);
  }

  // m_line_strip.points.push_back(f_path_waypoint);

  marker_pub.publish(m_line_strip);

  // ros::Duration(1.5).sleep();
}

void CVisualisation::updateVisualPath(const geometry_msgs::Point& f_path_waypoint)
{
  m_line_strip.points.push_back(f_path_waypoint);

  marker_pub.publish(m_line_strip);
}

}  // namespace rrt_planner