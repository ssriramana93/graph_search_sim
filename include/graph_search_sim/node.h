
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

  double distance_ = 0;
  size_t visit_count_ = 0;
//  mapping::Odometry odomreadings;
//  nav_msgs::Odometry current_pose;
  ros::Time stamp = ros::Time(0);
  size_t position_ = 0;
  size_t previous_offset_ = 0.0;
  Node();
  Node(vertexName name,double odomUnc,double distance,size_t visit_count);
};

#endif

