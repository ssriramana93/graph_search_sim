#include <graph_search_sim/branch.h>
#include <aslam_demo/mapping/odometry_processing.h>

Branch::Branch(ros::NodeHandle& n,Graph& graph,vertexName start_vertex): BranchBase(n,graph,start_vertex) {

}

Branch::Branch(ros::NodeHandle& n,Graph& graph,Node& start_node): BranchBase(n,graph,start_node) {

}

Branch::Branch(const Branch& branch): BranchBase(branch) {

}

void Branch::updateOdom(vertexName name) {

//double Branch::updateOdom(vertexName name) {
  size_t count = visited_nodes[name].first;
  auto latest_node = getLatestNode();
  double odomUnc = 0.0;
  if(count > 1 /*and count <= total_visit_allowed*/) {
    if(count > total_visit_allowed)  {
      please_kill_me = true;
   //   return;
    }
    ros::Time lp_timestamp = visited_nodes[name].second->stamp;
    auto edge = std::make_pair(latest_node->name_,name);
  //  std::cout<<"LPcl"<<lp_timestamp<<"\t"<<*(timestamps_.rbegin())<<std::endl;

//    odom_unc_cost = simLoopClosure(name,node_list);
    odom_unc_cost = cov(map,edge,true,lp_timestamp);

    odomUnc = latest_node->odomUnc_;

  }
  else if(count == 1) {
    auto edge = std::make_pair(latest_node->name_,name);
  //  odomUnc = cov(map,edge,latest_node->odomUnc_,false,ros::Time::now());
    odom_unc_cost = cov(map,edge,false,ros::Time::now());

  }
//  else please_kill_me = true;

 // return odomUnc;
}

/*double Branch::cov(Graph& map,std::pair<vertexName,vertexName>& edge,double value) {
  double dist = map.at(edge.first,edge.second);
  return(value + (value + 1)*dist/10.0);
}*/


double Branch::cov(Graph& map,std::pair<vertexName,vertexName>& edge,bool close_loop,ros::Time lp_timestamp) {
  double time_tolerance = 1.e-02;
  gtsam::Vector sigmas(6);
  double sigma = 0.1;
 // sigmas<<0.1,0.1,0.1,0.1,0.1,0.1;

  sigmas<<sigma,sigma,sigma,sigma,sigma,sigma;

  mapping::Odometry odomreadings;
  mapping::Timestamps timestamps;
  mapping::RelativePoseEstimates relative_poses;
  gtsam::NonlinearFactorGraph odom_graph;
  nav_msgs::Odometry current_position;
  gtsam::Values initial_guess;


  if(odomreadings_.size())  {
    current_position = odomreadings_.rbegin()->second;
    //odomreadings.insert(std::make_pair(current_position.header.stamp,current_position));
  //  std::cout<<"St\t"<<key_generator_.generateKey(factors::key_type::Pose2,current_position.header.stamp)<<"\t("<<current_position.pose.pose.position.x<<","<<current_position.pose.pose.position.y<<")"<<std::endl;
    timestamps.insert(current_position.header.stamp);
    timestamps_.insert(current_position.header.stamp);

  }
  else {

    current_position.pose.pose.position.x = 0.0;
    current_position.pose.pose.position.y = 0.0;
    current_position.pose.pose.position.z = 0.0;
    ros::Time time = ros::Time::now() - ros::Duration(2.0);
    timestamps_.insert(time);
    timestamps.insert(time);
    current_position.header.stamp = time;
    odomreadings_.insert(std::make_pair(time,current_position));
    gtsam::Pose2 mean(0.0,0.0,0.0);
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise  = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector_(3, 0.0001, 0.0001, 0.0001));
    odom_graph.add(gtsam::PriorFactor<gtsam::Pose2>(key_generator_.generateKey(factors::key_type::Pose2,time),mean,priorNoise));
    (*node_list.begin())->stamp = time;
    visited_nodes[(*node_list.begin())->name_].second = (*node_list.begin());
    initial_guess_.insert(key_generator_.generateKey(factors::key_type::Pose2,time),gtsam::Pose2(0.0,0.0,0.0));
    initial_guess.insert(key_generator_.generateKey(factors::key_type::Pose2,time),gtsam::Pose2(0.0,0.0,0.0));

  }

  map.simulateOdomReadings(edge.first,edge.second,current_position,odomreadings,timestamps);

  relative_poses = mapping::odometry::computeRelativePoses(odomreadings,timestamps,sigmas,1.0,1.0);
  gtsam::KeySet keys;
  for(auto const time: timestamps) {
    if (close_loop && time == *(timestamps.rbegin())) {
     // ros::Time ptime = *std::prev(timestamps.rbegin());
      auto latest_key = /**keys.rbegin();*/key_generator_.generateKey(factors::key_type::Pose2,time);
      auto lp_key = key_generator_.generateKey(factors::key_type::Pose2,lp_timestamp);
   //   std::cout << "Closing loop\t" << lp_key<<"\t"<<lp_timestamp<<"\t"<<key_generator_.generateKey(factors::key_type::Pose2,lp_timestamp)<<"\t" << latest_key<<std::endl;

      auto rel_pose = *(relative_poses.rbegin());
      gtsam::noiseModel::Base::shared_ptr noise = gtsam::noiseModel::Gaussian::Covariance(rel_pose.cov, true);

    //  gtsam::noiseModel::Base::shared_ptr noise = gtsam::noiseModel::Gaussian::Covariance(rel_pose.cov, true);
   //   gtsam::NonlinearFactor::shared_ptr factorr(new factors::OdometryFactor(latest_key, lp_key,rel_pose,noise));
      odom_graph.add(gtsam::BetweenFactor<gtsam::Pose2>(latest_key, lp_key,gtsam::Pose2(0.0,0.0,0.0),noise));
    //  continue;

    }

    gtsam::Key new_key = key_generator_.generateKey(factors::key_type::Pose2,time);
  //  std::cout<<"Key Stamp\t"<<new_key<<"\t"<<time<<std::endl;
    keys.insert(new_key);
  }
  //std::cout<<"Sizes"<<relative_poses.size()<<"\t"<<timestamps.size()<<"\t"<<keys.size()<<std::endl;

  odom_graph.push_back(mapping::odometry::createOdometryFactors(relative_poses,time_tolerance,keys));
  factor_graph_.push_back(odom_graph);
  timestamps_.insert(timestamps.begin(),timestamps.end());
  odomreadings_.insert(std::next(odomreadings.begin()),odomreadings.end());
  for(auto const time:timestamps) {
    nav_msgs::Odometry odom = odomreadings_[time];

   // initial_guess.insert(key_generator_.generateKey(factors::key_type::Pose2,time),gtsam::Pose2(odom.pose.pose.position.x,odom.pose.pose.position.y,0.0));

    if(time == *timestamps.begin()) continue;
  //  initial_guess.insert(key_generator_.generateKey(factors::key_type::Pose2,time),gtsam::Pose2(odom.pose.pose.position.x,odom.pose.pose.position.y,0.0));

  //  if(*timestamps.rbegin() == time) {
   //   std::cout <<"WTF!!!!!!!!!"<<std::endl;
  //  }
  //  std::cout<<"timestamp: "<<time<<"\t"<<key_generator_.generateKey(factors::key_type::Pose2,time)<<std::endl;
    initial_guess_.insert(key_generator_.generateKey(factors::key_type::Pose2,time),gtsam::Pose2(odom.pose.pose.position.x,odom.pose.pose.position.y,0.0));
  }

  gtsam::LevenbergMarquardtParams parameters_; //@todo:parameters
// initial_guess_.print("IG");
  for(auto const iter:odomreadings_) {
 //   std::cout<<"odom\t"<<iter.first<<"\t("<<iter.second.pose.pose.position.x<<","<<iter.second.pose.pose.position.y<<")"<<std::endl;
  }
  printNodes();

// factor_graph_.print("FG");
/*if(!mapping::optimization::validateFactorGraph(factor_graph_,initial_guess_)) {
    //if(!mapping::optimization::validateFactorGraph(factor_graph_,initial_guess_)) {
      ROS_INFO_STREAM("Not Validated!!");

  }
  else {
    ROS_INFO_STREAM("Validated!!");

  }*/
  //solution_ = mapping::optimization::optimizeFactorGraph(factor_graph_,initial_guess_,parameters_);
//  solution_.print("IG");

  pose_with_cov_ = mapping::optimization::computeCovariances(factor_graph_,initial_guess_);
 /* if(!close_loop) {
  auto pose_with_cov = mapping::optimization::computeCovariances(factor_graph_,initial_guess);
  pose_with_cov_.insert(pose_with_cov.begin(),pose_with_cov.end());
}
  else {
    pose_with_cov_ = mapping::optimization::computeCovariances(factor_graph_,initial_guess_);
  }*/
  double total_det = 0.0,max_det = -10000,avg_det = 0.0;
  for (auto const iter:pose_with_cov_) {
    auto det = iter.second.determinant();
    total_det += det;
    if(max_det < det) {
      max_det = det;
    }
   }
  avg_det = total_det/pose_with_cov_.size();
 // initial_guess_.clear();
  solution_.clear();
  std::cout << "CCost: "<<total_det<<std::endl;
  return max_det;
}


double Branch::simLoopClosure(vertexName name,NodeList& list) {
  double cost = 0.0;
  for(auto &iter: list) {
    iter->odomUnc_ *= 0.8;
    cost += iter->odomUnc_;
  }
  return cost;
}

