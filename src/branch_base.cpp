#include <graph_search_sim/branch_base.h>

BranchBase::BranchBase(ros::NodeHandle& n,Graph& graph,vertexName start_vertex): n_(n),map(graph) {
  std::shared_ptr<Node>  start_node= std::make_shared<Node>(start_vertex,0.0,0.0,0.0);

  updateVisited(start_vertex);
  node_list.push_back(start_node);
  unexplored_nodes = map.size();
  //unexplored_cost = unexplored_penality*map.size();
  //unexplored_cost = getunexploredcost();

}


BranchBase::BranchBase(ros::NodeHandle& n,Graph& graph,Node& start_node): n_(n),map(graph) {
  updateVisited(start_node.name_);
  node_list.push_back(std::make_shared<Node>(start_node));
  unexplored_nodes = map.size();

//  unexplored_cost = unexplored_penality*map.size();
 // unexplored_cost = getunexploredcost();


}

BranchBase::BranchBase(const BranchBase& branch_base):
    n_(branch_base.NodeHandle()),
    node_list(branch_base.nodeList()),
    visited_nodes(branch_base.visitedNodes()),
    map(branch_base.Map()),
    odom_unc_cost(branch_base.odomUncCost()),
    distance(branch_base.Distance()),
    unexplored_nodes(branch_base.unexplored_nodes),
   // unexplored_cost(branch_base.unexploredCost()),
    unexplored_penality(branch_base.unexploredPenality()),
    total_visit_allowed(branch_base.totalVisitAllowed()),
    please_kill_me(branch_base.please_kill_me),
    factor_graph_(branch_base.factor_graph_),
    odomreadings_(branch_base.odomreadings_),
    relative_poses_(branch_base.relative_poses_),
    pose_with_cov_(branch_base.pose_with_cov_),
    current_position_(branch_base.current_position_),
    timestamps_(branch_base.timestamps_),
    key_generator_(branch_base.key_generator_),
    initial_guess_(branch_base.initial_guess_),
    solution_(branch_base.solution_) {

}

void BranchBase::updateVisited(vertexName name) {
  if(!visited_nodes.size()) {

    visited_nodes.insert(std::make_pair(name,std::make_pair(0,std::make_shared<Node>())));
  }
  visited_nodes[name].first++;
}


void BranchBase::update(vertexName name) {
  if(!visited_nodes[name].first) {
    unexplored_nodes--;
  }

  updateVisited(name);
  map.updateEdgeCount(getLatestNode()->name_,name);


  distance += updateDistance(name);

  updateOdom(name);

 // double odomUnc = updateOdom(name);
 // odom_unc_cost += odomUnc;
  std::shared_ptr<Node> node_ptr = std::make_shared<Node>(name,0,distance,visited_nodes[name].first);

  node_ptr->stamp = *(timestamps_.rbegin());
  node_ptr->position_ = node_list.size();
  node_ptr->previous_offset_ = (double)getPreviousDistance(name);
  node_list.push_back(node_ptr);

  visited_nodes[name].second = node_ptr;
}

const double BranchBase::getunexploredcost() const{
 // return unexplored_penality*unexplored_nodes;

  return unexplored_penality*map.unvisited_edge_count_;
}

const double BranchBase::getTrueCost() const {
 // std::cout<<"OdomUnc: "<<odom_unc_cost<<"Dist: "<<distance<<"UnexploredCost: "<<getunexploredcost()<<std::endl;
  return(odom_unc_cost + distance + getunexploredcost());
}

const double BranchBase::getTotalCost() const {
  return (getTrueCost() + getHeuristics());
}

const std::shared_ptr<Node> BranchBase::getLatestNode() const {
  return(*(node_list.rbegin()));
}

void BranchBase::printNodes() {
  std::cout<<"[ ";
  for(auto const node: node_list) {
    std::cout<<node->name_<<" ";
  }
  std::cout<<"]"<<std::endl;
}

double BranchBase::updateDistance(vertexName name) {
  return (map.at(getLatestNode()->name_,name).weight);
}

size_t BranchBase::getPreviousDistance(vertexName v) {
  if(visited_nodes[v].first <= 1) return 10000;
  return node_list.size() - visited_nodes[v].second->position_;
}

const double BranchBase::getHeuristics() const {
  if(!node_list.size()) return(0);
  return /*map.unvisited_distance;*/-100*(double)(getLatestNode()->previous_offset_);
}

BranchBase:: ~BranchBase() {
}

/*double BranchBase::updateOdom(vertexName vertex) {

}*/


