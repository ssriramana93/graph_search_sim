
#ifndef BRANCH_BASE_H
#define BRANCH_BASE_H


#include <ros/ros.h>
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <mutex>
#include <vector>
#include <map>

#include <graph_search_sim/graph.h>
#include <graph_search_sim/node.h>


class BranchBase {

public:
  typedef std::vector<std::shared_ptr<Node> > NodeList;


  NodeList node_list;
  std::map<vertexName,std::pair<size_t,std::shared_ptr<Node> > > visited_nodes;
  Graph map;
  double odom_unc_cost = 0.0,
         distance = 0.0,
         unexplored_penality = 1.0e+9,
         max_odom_cost = 0.0;
     //    unexplored_cost = 0.0;
  size_t total_visit_allowed = 2;
  double max_odom_unc = 100000000000000000000000000.0;



  double updateDistance(vertexName vertex);
//  virtual double updateOdom(vertexName vertex) = 0;
  //double updateOdom(vertexName vertex);
  virtual void updateOdom(vertexName vertex) = 0;

  ros::NodeHandle n_;

  bool please_kill_me = false;

  gtsam::NonlinearFactorGraph factor_graph_;
  mapping::Odometry odomreadings_;
  mapping::RelativePoseEstimates relative_poses_;
  mapping::optimization::Covariances pose_with_cov_;
  nav_msgs::Odometry current_position_;
  mapping::Timestamps timestamps_;
  factors::KeyGenerator key_generator_;
  gtsam::Values initial_guess_,solution_;
  size_t unexplored_nodes;

  BranchBase(ros::NodeHandle& n,Graph& graph,vertexName start_vertex);
  BranchBase(ros::NodeHandle& n,Graph& graph,Node& start_node);
  BranchBase(const BranchBase& branch_base);
  void updateVisited(vertexName name);
  void update(vertexName vertex);
  const double getTotalCost() const;
  const double getTrueCost() const;
  const double getunexploredcost() const;
  const std::shared_ptr<Node> getLatestNode() const;
  void printNodes();
  virtual ~BranchBase();

  NodeList nodeList() const{
    return node_list;
  }

  std::map<vertexName,std::pair<size_t,std::shared_ptr<Node> > > visitedNodes() const {
    return visited_nodes;
  }

  Graph Map() const{
    return map;
  }

  double odomUncCost() const{
    return odom_unc_cost;
  }

  double Distance() const{
    return distance;
  }

  double unexploredPenality() const{
    return unexplored_penality;
  }

  double unexploredCost() const{
    return getunexploredcost();
  }

  size_t totalVisitAllowed() const{
    return total_visit_allowed;
  }

  ros::NodeHandle NodeHandle() const{
    return n_;
  }

  size_t getPreviousDistance(vertexName v);
  const double getHeuristics() const;
};

#endif
