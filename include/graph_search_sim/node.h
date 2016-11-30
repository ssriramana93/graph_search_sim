
#ifndef NODE_H
#define NODE_H

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <mutex>
#include <vector>
#include <map>

#include <graph_search_sim/graph_common.h>


class Node {
public:
  vertexName name_;
  double odomUnc_;

  double distance_;
  size_t visit_count_;
//  mapping::Odometry odomreadings;
//  nav_msgs::Odometry current_pose;
  ros::Time stamp;
  size_t position_;
  size_t previous_position_;
  Node();
  Node(vertexName name,double odomUnc,double distance,size_t visit_count);
};

#endif

