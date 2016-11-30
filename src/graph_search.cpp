#include <graph_search_sim/graph_search.h>


GraphSearch::GraphSearch(ros::NodeHandle& n,Graph& map,vertexName start_name):n_(n),map_(map),start_name_(start_name) {

}

void GraphSearch::calcBestPath() {

  std::cout<<"Calc Best Path "<<std::endl;

  Node start_node(start_name_,0,0,1);
  Branch branch(n_,map_,start_node);
  queue_.push(branch);
  std::cout<<"Queue "<<std::endl;

  while(!queue_.empty()) {

    Branch current_branch(queue_.top());
    current_branch.printNodes();
    queue_.pop();
    auto latest_node = current_branch.getLatestNode();
    double current_true_cost = current_branch.getTotalCost(),current_heuristic_cost = current_branch.getHeuristics();

    double current_cost = current_true_cost + current_heuristic_cost;
    if (best_score_ > current_true_cost) {
      //best_score_ = current_cost;
      best_score_ = current_true_cost;
      best_branch_ = std::make_shared<Branch>(current_branch);
    }
    std::cout<<"cost: "<<current_cost<<std::endl;
    std::vector<vertexName> edges;
    map_.getEdges(latest_node->name_,edges);
    for(auto const iter: edges) {

      Branch new_branch(current_branch);

      new_branch.update(iter);

      if(new_branch.please_kill_me) continue;

      queue_.push(new_branch);
    }
  }
}
