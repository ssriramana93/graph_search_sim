#ifndef BRANCH_H
#define BRANCH_H

#include <graph_search_sim/branch_base.h>


class Branch: public BranchBase {


public:
  Branch(ros::NodeHandle& n,Graph& graph,vertexName start_vertex);
  Branch(ros::NodeHandle& n,Graph& graph,Node& start_node);
  Branch(const Branch& branch);
  //double updateOdom(vertexName vertex);
  void updateOdom(vertexName vertex);
//  double cov(Graph& map,std::pair<vertexName,vertexName>& edge,double value,bool close_loop,ros::Time lp_timestamp);
  double cov(Graph& map,std::pair<vertexName,vertexName>& edge,bool close_loop,ros::Time lp_timestamp);

  double simLoopClosure(vertexName name,NodeList& list);

};

#endif
