#include "rrt_planner/rrt_star_planner.hpp"

namespace rrt_planner
{
RRT::RRT(const ros::NodeHandle& f_node_handle, int max_count, int delta_max, Point2d start, bool goalReached,
         Point2d goal, int goal_tolerance, int epsilon)
  : m_node_handle(f_node_handle)
  , m_max_count(max_count)
  , m_delta_max(delta_max)
  , m_goalReached(goalReached)
  , m_goal(goal)
  , m_goal_tolerance(goal_tolerance)
  , m_waypoint_visual(m_node_handle)
  , m_epsilon(epsilon)
{
  fill2DArray(20);
  m_count = 1;
  m_nodes.push_back(start);
  addPoint(start, 1000);
  m_iterations = 50;
}

void RRT::fill2DArray(int numRows)
{
  for (int row = 0; row < numRows; row++)
  {
    for (int col = 0; col < 20; col++)
    {
      // ar[row][col] = rand() % 101;
      m_ar[row][col] = 0;
    }
  }
}

void RRT::print2DArray(int numRows)
{
  geometry_msgs::Point m_waypoint;
  for (int row = 0; row < numRows; row++)
  {
    for (int col = 0; col < 20; col++)
    {
      std::cout << m_ar[row][col] << "\t";
      m_waypoint.x = row + 1;
      m_waypoint.y = col + 1;

      if (m_ar[row][col] == -50000)
      {
        m_waypoint_visual.updateVisual(m_waypoint, 1);
      }
      else if (m_ar[row][col] > 10)
      {
        m_waypoint_visual.updateVisual(m_waypoint, 2);
      }
      // else
      // {
      //   m_waypoint_visual.updateVisual(m_waypoint, 0);
      // }
    }
    std::cout << std::endl;
  }
}

void RRT::generateCollisionPts(int num_collisionpts)
{
  int iterator = 0;

  Point2d collisionpt;

  while (iterator < num_collisionpts)
  {
    collisionpt.x = rand() % 20;
    collisionpt.y = rand() % 20;
    collisionpt.id = iterator;

    addPoint(collisionpt, -50000);

    m_collision_nodes.push_back(collisionpt);
    iterator++;
  }
}

bool RRT::isInCollision(Point2d& parent, Point2d& new_point)
{
  // Collision with obstacle
  for (auto collisionpt : m_collision_nodes)
  {
    if (atan2(parent.y - collisionpt.y, parent.x - collisionpt.x) ==
        atan2(collisionpt.y - new_point.y, collisionpt.x - new_point.x))
    {
      // std::cout << "Collision\n";
      return true;
    }
  }

  // Collision with nodes already present
  for (auto collisionpt : m_nodes)
  {
    if (atan2(parent.y - collisionpt.y, parent.x - collisionpt.x) ==
        atan2(collisionpt.y - new_point.y, collisionpt.x - new_point.x))
    {
      std::cout << "Collision\n";
      return true;
    }
  }

  return false;
}

void RRT::addPoint(Point2d& pt, int val)
{
  m_ar[pt.x][pt.y] = val;
}

int RRT::findParent(Point2d& pt)
{
  double dist1 = distance(m_nodes.front(), pt);

  double dist2;
  int parentID = m_nodes.front().id;
  // std::cout << "dist: " << dist << "\tparentID: " << parentID << std::endl;

  for (int iterator = 1; iterator < m_nodes.size(); iterator++)
  {
    dist2 = distance(m_nodes.at(iterator), pt);
    // dist2 = distance(m_nodes.front(), pt);
    // std::cout << "dist2: " << dist2 << std::endl;
    // std::cout << "m_nodes.at(iterator+1) x: " << m_nodes.at(iterator).x << "\ty: " << m_nodes.at(iterator).y
    // <<std::endl;
    if (dist2 < dist1)
    {
      // std::cout << "Updating"
      //           << "\tOld id: " << parentID << "\tdist1: " << dist1 << "\tdist2: " << dist2
      //           << "\t New id: " << m_nodes.at(iterator).id << std::endl;

      parentID = m_nodes.at(iterator).id;
      dist1 = dist2;
    }
  }
  return parentID;
  // pt.parent = true;
}

double RRT::distance(Point2d& pt1, Point2d& pt2)
{
  return std::sqrt(std::pow((pt2.x - pt1.x), 2) + std::pow((pt2.y - pt1.y), 2));
}

void RRT::updateNewPoint(Point2d& new_node, Point2d& parent, int val)
{
  Point2d updated_new_node;

  // updated_new_node.x = delta_max * (new_node.x - parent.x) /
  //       std::sqrt(std::pow((new_node.x - parent.x), 2) + std::pow((new_node.y - parent.y), 2));
  // updated_new_node.y = delta_max * (new_node.y - parent.y) /
  //       std::sqrt(std::pow((new_node.x - parent.x), 2) + std::pow((new_node.y - parent.y), 2));

  float theta = atan2(new_node.y - parent.y, new_node.x - parent.x);

  new_node.x = parent.x + m_delta_max * cos(theta);
  new_node.y = parent.y + m_delta_max * sin(theta);

  // std::cout << "new_node.x: " << new_node.x << "\tnew_node.y: " << new_node.y << std::endl;
  // std::cout << "parent.x: " << parent.x << "\tparent.y: " << parent.y << std::endl;
  // std::cout << "updated_new_node.x: " << updated_new_node.x << "\tupdated_new_node.y: " << updated_new_node.y
  // << std::endl;
  // addPoint(updated_new_node, val);
}

bool RRT::isDuplicate(Point2d& new_node)
{
  for (int iterator = 0; iterator < m_nodes.size(); iterator++)
  {
    // std::cout << " new_node.x= " << new_node.x << " new_node.y= " << new_node.y << "\n m_nodes.at(iterator).x "
    //           << m_nodes.at(iterator).x << " nm_nodes.at(iterator).y= " << m_nodes.at(iterator).y << std::endl;
    if (new_node.x == m_nodes.at(iterator).x && new_node.y == m_nodes.at(iterator).y)
    {
      // std::cout << "Collision \n";
      // std::cout << " new_node.id= " << new_node.id << " m_nodes.at(iterator).id= " << m_nodes.at(iterator).id
      //           << " new_node.x= " << new_node.x << " new_node.y= " << new_node.y << std::endl;
      return true;
    }
  }
  return false;
}

int RRT::computeTotalCost(int id)
{
  // int id = pt1.id;
  int total_cost = 0;

  ROS_INFO_STREAM("id"<<id);
  while (id != -1)
  {
    ROS_INFO("CHekc");
    total_cost += m_nodes.at(id).cost;

    ROS_INFO("CHekc2");

    id = m_nodes.at(id).x;
  }
  return total_cost;
}

void RRT::optimiseNode(Point2d& pt1)
{
  std::vector<Point2d> vicintiy_points;

  // Find all points in vicinity
  for (int iterator = 1; iterator < m_nodes.size(); iterator++)
  {
    if (distance(m_nodes.at(iterator), pt1) < m_epsilon)
    {
      vicintiy_points.push_back(m_nodes.at(iterator));
    }
  }

  // Choose the node which has the least cost from starting point
  if (vicintiy_points.size())
  {
    ROS_INFO_STREAM("id" << vicintiy_points.size());
    double new_pt_total_cost = computeTotalCost(vicintiy_points.front().id);
    double temp_new_pt_total_cost;

    // for (int iterator = 1; iterator < vicintiy_points.size(); iterator++)
    // {
    //   temp_new_pt_total_cost = computeTotalCost(vicintiy_points.at(iterator));

    //   if (temp_new_pt_total_cost < new_pt_total_cost)
    //   {
    //     pt1.parent_id = vicintiy_points.at(iterator).id;
    //     new_pt_total_cost = temp_new_pt_total_cost;
    //   }
    // }

    // // Rewire
    // // Iterate over vicintiy_points
    // // compute total cost except for pt1.parent_id
    // for (int iterator = 1; iterator < vicintiy_points.size(); iterator++)
    // {
    //   if (vicintiy_points.at(iterator).id != pt1.parent_id)
    //   {
    //     temp_new_pt_total_cost = computeTotalCost(vicintiy_points.at(iterator));

    //     if (temp_new_pt_total_cost > new_pt_total_cost + distance(vicintiy_points.at(iterator), pt1))
    //     {
    //       m_nodes.at(iterator).parent_id = pt1.id;
    //     }
    //   }
    // }
  }
}

int RRT::computeCost(Point2d& pt1)
{
  return distance(pt1, m_nodes.at(pt1.parent_id - 1));

  // pt1.cost_from_start = computeTotalCost(m_nodes.at(pt1.parent_id - 1).id);
}

void RRT::findRandomPoint()
{
  // 1. Find random point location
  Point2d rndompt;
  rndompt.x = rand() % 20;
  rndompt.y = rand() % 20;

  // std::cout << "rndompt x: " << rndompt.x << "\t y: " << rndompt.y << std::endl;

  // 2. Find nearest node to the random node
  if (!isDuplicate(rndompt))
  {
    rndompt.parent_id = findParent(rndompt);

    // std::cout << "rndompt.parent_id: " << rndompt.parent_id << std::endl;

    // std::cout << "rndompt.parent_id: " << rndompt.parent_id << "\tid: " << rndompt.id << std::endl;

    // 3. Get new node location wrt parent node
    // if (rndompt.parent)
    // std::cout << "m_nodes.at(0) " << m_nodes.at(rndompt.parent_id -1).x<< std::endl;

    // std::cout << "rndompt.id: " << rndompt.id << "\tparent: " << rndompt.parent_id << std::endl;

    // std::cout << "m_nodes.at(rndompt.parent_id - 1) ID " << m_nodes.at(rndompt.parent_id - 1).id << std::endl;

    if (m_delta_max < distance(rndompt, m_nodes.at(rndompt.parent_id - 1)))
    {
      updateNewPoint(rndompt, m_nodes.at(rndompt.parent_id - 1), rndompt.id * 100);
    }

    // rndompt.cost = computeCost(rndompt);

    // 4. Check for collision
    if (!isDuplicate(rndompt))
    {
      if (!isInCollision(m_nodes.at(rndompt.parent_id - 1), rndompt))
      {
        // Optimise the new node
        // optimiseNode(rndompt);

        // std::cout << "No Collision \n";
        m_count++;
        rndompt.id = m_count;
        rndompt.parent = true;
        m_nodes.push_back(rndompt);

        addPoint(rndompt, rndompt.id * 100);
      }
    }
  }
}

bool RRT::isGoalReached()
{
  // std::cout << " m_nodes.back().id= " << m_nodes.back().id << std::endl;
  if (distance(m_nodes.back(), m_goal) < m_goal_tolerance)
  {
    return true;
  }
  return false;
}

void RRT::generatePath()
{
  // while (!m_goalReached || m_count < m_iterations - 1)
  while (!m_goalReached)
  {
    // 1. Create random points
    findRandomPoint();

    // 2. check if goal is reached with the new random point
    if (isGoalReached())
    {
      std::cout << " Goal reached " << std::endl;
      m_goalReached = true;

      // update goal node wrt parent and add to queue
      m_count++;
      m_goal.parent_id = m_nodes.back().id;
      m_goal.id = m_count;
      m_goal.parent = true;
      m_nodes.push_back(m_goal);

      addPoint(m_goal, m_goal.id * -10000);
    }
  }
}

void RRT::printPath()
{
  // std::cout << "0: x: " << m_nodes.at(0).x << "\t y: " << m_nodes.at(0).y;
  // std::cout << "0: x: " << m_nodes.at(1).x << "\t y: " << m_nodes.at(1).y;

  geometry_msgs::Point m_waypoint;

  // m_waypoint.x = row;
  // m_waypoint.y = col;

  int i = m_nodes.back().id;
  while (i != -1)
  {
    std::cout << " Node: " << i << "at x" << m_nodes.at(i - 1).x << " y" << m_nodes.at(i - 1).y << std::endl;
    i = m_nodes.at(i - 1).parent_id;

    m_waypoint.x = m_nodes.at(i - 1).x + 1;
    m_waypoint.y = m_nodes.at(i - 1).y + 1;

    m_waypoint_visual.updateVisualPath(m_waypoint);
  }
}

void RRT::printNodes()
{
  for (auto node : m_nodes)
  {
    // std::cout << "node.x= " << node.x << " node.y= " << node.y << std::endl;
    std::cout << " node.id= " << node.id << " node.parent_id= " << node.parent_id << " node.parent= " << node.parent
              << std::endl;
  }
}

}  // namespace rrt_planner
