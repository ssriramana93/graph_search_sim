#ifndef GRAPH_SEARCH_H
#define GRAPH_SEARCH_H

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <mutex>
#include <vector>
#include <map>
#include <fstream>
#include <queue>
#include <limits>

#include "gnuplot_i.hpp"
#include <graph_search_sim/graph.h>
#include <graph_search_sim/node.h>
#include <graph_search_sim/branch.h>

class GraphSearch {
  struct compareBranch {
      bool operator()(const Branch& lhs,const Branch& rhs) const {
        return (lhs.getTotalCost() > rhs.getTotalCost());
      }
    };
public:
  ros::NodeHandle n_;
  Graph map_;
  vertexName start_name_;
  double best_score_ = std::numeric_limits<double>::max();
  size_t best_iter_ = 0;
  std::shared_ptr<Branch> best_branch_;
  std::priority_queue<Branch,std::vector<Branch >,compareBranch> queue_;
  //std::stack<Branch> queue_;

  std::vector<double> collect_cost,collect_best;
  std::ofstream output_file;
  std::ostream_iterator<double> output_iterator;
  double getLowerBound(Branch& branch);
  Gnuplot gp,gp2;
  GraphSearch(ros::NodeHandle& n,Graph& map,vertexName start_name);
  void calcBestPath();
  void calcBestBnBPath();
  void plotResults();
  double getUpperBoundCost();

};

#endif

