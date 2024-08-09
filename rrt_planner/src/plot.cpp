#include <iostream>
#include <ctime>

#include <random>
#include <vector>

// #include "rrt_planner/plot.hpp"
// using namespace std;

struct Point2d
{
  int x;
  int y;
  int id;
  int parent_id;
  bool parent = false;
};  // Structure variable

class RRT
{
public:
  RRT(int max_count, int delta_max, Point2d start, bool goalReached, Point2d goal, int goal_tolerance)
    : m_max_count(max_count)
    , m_delta_max(delta_max)
    , m_goalReached(goalReached)
    , m_goal(goal)
    , m_goal_tolerance(goal_tolerance)
  {
    fill2DArray(20);
    m_count = 1;
    m_nodes.push_back(start);
    addPoint(start, 1000);
    m_iterations = 50;
  }

  void fill2DArray(int numRows);
  void print2DArray(int numRows);

  void generateCollisionPts(int num_collisionpts);

  bool isInCollision(Point2d& parent, Point2d& new_point);

  // Function to add point in space within the max_delta
  void updateNewPoint(Point2d& pt1, Point2d& parent, int val);

  // Function to add point in space
  void addPoint(Point2d& pt, int val);

  // Euclidean distance
  double distance(Point2d& pt1, Point2d& pt2);

  bool isCollision(Point2d& new_node);

  // Function to add point in space
  void findRandomPoint();

  // Find the nearest point
  int findParent(Point2d& pt);

  // To check if goal is reached
  bool isGoalReached();

  void generatePath();

  void printPath();

  // Print Node Vector
  void printNodes();

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
};

void RRT::fill2DArray(int numRows)  // fills 20x20 2D array with random #s (1-100)
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

void RRT::print2DArray(int numRows)  // prints array
{
  for (int row = 0; row < numRows; row++)
  {
    for (int col = 0; col < 20; col++)
    {
      std::cout << m_ar[row][col] << "\t";
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
  for (auto collisionpt : m_collision_nodes)
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

bool RRT::isCollision(Point2d& new_node)
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
void RRT::findRandomPoint()
{
  // 1. Find random point location
  Point2d rndompt;
  rndompt.x = rand() % 20;
  rndompt.y = rand() % 20;

  // std::cout << "rndompt x: " << rndompt.x << "\t y: " << rndompt.y << std::endl;

  // 2. Find nearest node to the random node
  if (!isCollision(rndompt))
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

    // 4. Check for collision
    if (!isCollision(rndompt))
    {
      if (!isInCollision(m_nodes.at(rndompt.parent_id - 1), rndompt))
      {
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

  int i = m_nodes.back().id;
  while (i != -1)
  {
    std::cout << " Node: " << i << "at x" << m_nodes.at(i - 1).x << " y" << m_nodes.at(i - 1).y << std::endl;
    i = m_nodes.at(i - 1).parent_id;
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

int main()
{
  srand((unsigned int)time(0));

  // int max_distance;

  Point2d start;
  start.x = 0;
  start.y = 4;
  start.id = 1;
  start.parent_id = -1;  // Parent id for starting node doesnt exist

  Point2d goal;
  goal.x = 10;
  goal.y = 18;
  goal.id = 0;
  goal.parent_id = 0;

  RRT rrt(5, 4, start, false, goal, 4);

  rrt.generateCollisionPts(5);

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

  rrt.print2DArray(20);

  // rrt.printNodes();

  rrt.printPath();

  return 0;
}