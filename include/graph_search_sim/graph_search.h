#ifndef GRAPH_SEARCH_H
#define GRAPH_SEARCH_H

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <mutex>
#include <vector>
#include <map>
#include <queue>

#include <graph_search_sim/graph.h>
#include <graph_search_sim/node.h>
#include <graph_search_sim/branch.h>

class GraphSearch {
  struct compareBranch {
      bool operator()(const Branch& lhs,const Branch& rhs) const {
        return lhs.getTotalCost() < rhs.getTotalCost();
      }
    };
public:
  ros::NodeHandle n_;
  Graph map_;
  vertexName start_name_;
  double best_score_ = 1e+10;
  std::shared_ptr<Branch> best_branch_;
  std::priority_queue<Branch,std::vector<Branch >,compareBranch> queue_;

  GraphSearch(ros::NodeHandle& n,Graph& map,vertexName start_name);
  void calcBestPath();

};

#endif

