#include <graph_search_sim/graph_search.h>


GraphSearch::GraphSearch(ros::NodeHandle& n,Graph& map,vertexName start_name):n_(n),map_(map),start_name_(start_name),output_file("./example.txt"),output_iterator(output_file, "\n"),gp("tee script.gp | gnuplot -persist"),gp2("tee script.gp | gnuplot -persist") {

}

void GraphSearch::calcBestPath() {

  std::cout<<"Calc Best Path "<<std::endl;

  Node start_node(start_name_,0,0,1);
  Branch branch(n_,map_,start_node);
  queue_.push(branch);
  std::cout<<"Queue "<<std::endl;
  size_t n_count = 0;

  while(!queue_.empty()) {
    n_count++;
    Branch current_branch(queue_.top());
    current_branch.printNodes();
    queue_.pop();
    auto latest_node = current_branch.getLatestNode();
    double current_true_cost = current_branch.getTrueCost(),current_heuristic_cost = current_branch.getHeuristics();

    double current_cost = current_true_cost + current_heuristic_cost;
    if (best_score_ > current_true_cost) {
      //best_score_ = current_cost;
      best_iter_ = n_count;
      best_score_ = current_true_cost;
      best_branch_ = std::make_shared<Branch>(current_branch);
    }
    collect_cost.push_back(current_true_cost);
    collect_best.push_back(best_score_);
    if(!(n_count%50)) {
   //   plotResults();
    }
    if(!(n_count%200)) {
   //   gp2.remove_tmpfiles();
    }
    std::cout<<"cost: "<<current_cost<<"\tHeuristic:\t"<<current_heuristic_cost<<"\tBest Score\t"<<best_score_<<"\tBest: ";
    best_branch_->printNodes();
    std::vector<vertexName> edges;
    map_.getEdges(latest_node->name_,edges);
    for(auto const iter: edges) {

      Branch new_branch(current_branch);

      new_branch.update(iter);

      if(new_branch.please_kill_me) continue;

      queue_.push(new_branch);
    }
  }
  plotResults();
}

double GraphSearch::getUpperBoundCost() {

  std::vector<vertexName> cpp_path;
  map_.getChinesePostmanPath("B",cpp_path);
  cpp_path.erase(cpp_path.begin());
  for(auto const iter:cpp_path) {
    std::cout<<iter<<"\t";
  }
  std::cout<<std::endl;
  Node start_node(*cpp_path.begin(),0,0,1);
  Branch branch(n_,map_,start_node);
  bool skip_first = false;
  for(auto const node:cpp_path) {
    if(!skip_first) {
      skip_first = true;
      continue;
    }
    branch.update(node);
    std::cout<<"TC:"<<branch.getTrueCost()<<std::endl;
  }
  return branch.getTrueCost();
}

double GraphSearch::getLowerBound(Branch& branch) {
  return branch.Distance() + branch.map.unvisited_distance + branch.max_odom_cost;
}

void GraphSearch::calcBestBnBPath() {

  std::cout<<"Calc Best Path "<<std::endl;

  Node start_node(start_name_,0,0,1);
  Branch branch(n_,map_,start_node);
  queue_.push(branch);
  std::cout<<"Queue "<<std::endl;
  size_t n_count = 0;
  double upper_bound  = getUpperBoundCost();
  std::cout<<"Upper Bound Cost"<<upper_bound<<std::endl;
  while(!queue_.empty()) {
    n_count++;
    Branch current_branch(queue_.top());
    current_branch.printNodes();
    queue_.pop();
    auto latest_node = current_branch.getLatestNode();
    double current_true_cost = current_branch.getTrueCost(),current_heuristic_cost = current_branch.getHeuristics();

    double current_cost = current_true_cost + current_heuristic_cost;

    if (best_score_ > current_true_cost) {
      best_iter_ = n_count;

      best_score_ = current_true_cost;
      best_branch_ = std::make_shared<Branch>(current_branch);
    }
    if(upper_bound > current_true_cost) {
     upper_bound = current_true_cost;
    }

    collect_cost.push_back(current_true_cost);
    collect_best.push_back(upper_bound);

    if(!(n_count%50)) {
   //   plotResults();
    }
    if(!(n_count%200)) {
   //   gp2.remove_tmpfiles();
    }
    std::cout<<"cost: "<<current_cost<<"\tHeuristic:\t"<<current_heuristic_cost<<"\tUpperBound\t"<<upper_bound<<"\tBest: "<<std::endl;

    best_branch_->printNodes();
    std::vector<vertexName> edges;
    map_.getEdges(latest_node->name_,edges);
    for(auto const iter: edges) {

      Branch new_branch(current_branch);

      new_branch.update(iter);

      double lower_bound = getLowerBound(new_branch);
      if(lower_bound > upper_bound) continue;
      std::cout<<"Lower Bound "<<lower_bound<<"\tUpperBound\t"<<upper_bound<<"\tCost: "<<new_branch.getTrueCost()<<"\tchild\t"<<iter<<"Total"<<new_branch.getTotalCost()<<std::endl;

      queue_.push(new_branch);
    }
  }
  plotResults();
}

void GraphSearch::plotResults() {
 // gp.set_grid();
 // gp.set_style("points").plot_x(collect_cost,"current_cost");
  gp2.replot();
  gp2.set_grid();
  gp2.set_style("bestscore").plot_x(collect_best,"current_best");
 // gp.remove_tmpfiles();
}
