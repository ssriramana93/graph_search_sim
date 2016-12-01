
#ifndef GRAPH_H
#define GRAPH_H


#include <vector>
#include <map>

#include <graph_search_sim/graph_common.h>



class Graph {

public:
  graphData data_;
  ros::NodeHandle n_;
  struct compareEdge {
    bool operator()(const std::pair<vertexName,vertexName> &lhs,const std::pair<vertexName,vertexName> &rhs) {
    //  bool result = !(((lhs.first == rhs.first) && (lhs.second == rhs.second)) || ((lhs.first == rhs.second) && (rhs.first == lhs.second)));
       auto arg1 = std::min<vertexName>(lhs.first,lhs.second),arg2 = std::max<vertexName>(lhs.first,lhs.second),arg3 = std::min<vertexName>(rhs.first,rhs.second),arg4 = std::max<vertexName>(rhs.first,rhs.second);
       return (std::tie(arg1,arg2) < std::tie(arg3,arg4));
     // return !(((lhs.first == rhs.first) && (lhs.second == rhs.second)) || ((lhs.first == rhs.second) && (rhs.first == lhs.second)));
    }
  };
  std::map<std::pair<vertexName,vertexName>,int,compareEdge> edge_count_;
  size_t unvisited_edge_count_ = 0;
  double unvisited_distance = 0;

  Graph(ros::NodeHandle& n):n_(n) {
  }

  Graph(ros::NodeHandle& n,graphData& data):n_(n) {
    setData(data);
    unvisited_edge_count_ = getUnvisitedEdgeCount();
  }

  Graph(const Graph& graph):n_(graph.n_),edge_count_(graph.edge_count_),unvisited_edge_count_(graph.unvisited_edge_count_),unvisited_distance(graph.unvisited_distance) {
    setData(graph.data());
  }

  void addNodeWithEdges(vertexName name,std::vector<vertexName> edges,std::vector<EdgeType> data) {
    std::map<vertexName,EdgeType> edge_list;

    for(size_t i = 0; i < edges.size(); i++ ) {
      if (edge_count_.find(std::make_pair(name,edges[i])) == edge_count_.end()) {
    /*    std::cout<<"("<<(!compareEdge()(std::make_pair("B","D"),std::make_pair("B","D")) and !compareEdge()(std::make_pair("B","D"),std::make_pair("B","D")))<<","<<(!compareEdge()(std::make_pair("D","B"),std::make_pair("B","D")) and !compareEdge()(std::make_pair("B","D"),std::make_pair("D","B")))<<")"<<std::endl;
        std::cout<<"("<<(!compareEdge()(std::make_pair("B","D"),std::make_pair("B","A")) and !compareEdge()(std::make_pair("B","A"),std::make_pair("B","D")))<<","<<(!compareEdge()(std::make_pair("D","B"),std::make_pair("B","D")) and !compareEdge()(std::make_pair("B","D"),std::make_pair("D","B")))<<")"<<std::endl;

        std::cout<<compareEdge()(std::make_pair("E","B"),std::make_pair("E","A"))<<std::endl;
        std::cout<<compareEdge()(std::make_pair("E","A"),std::make_pair("E","B"))<<std::endl;*/


        edge_count_.insert(std::make_pair(std::make_pair(name,edges[i]),0));
        unvisited_distance += data[i].weight;
      }
      edge_list.insert(std::make_pair(edges[i],data[i]));
    }
    data_.insert(std::make_pair(name,edge_list));
    printEdgeCount();
    unvisited_edge_count_ = getUnvisitedEdgeCount();
  }

  const size_t size() const {
    return (data_.size());
  }


  graphData data() const{
    return data_;
  }

  void printEdgeCount() {
    for(auto const iter:edge_count_) {
      std::cout<<"Pair:("<<iter.first.first<<","<<iter.first.second<<")"<<"->"<<iter.second<<std::endl;
    }
  }

  void setData(const graphData& data) {
    for (auto const vertex: data) {
      data_.insert(std::make_pair(vertex.first,vertex.second));
    }
  }

  bool checkElement(vertexName v) {
    return (data_.find(v) != data_.end());
  }

  bool checkEdge(vertexName v1,vertexName v2) {
    if (!checkElement(v1)) return false;
    return(data_.at(v1).find(v2) != data_.at(v1).end());
  }

 /* double at(vertexName v1,vertexName v2) {
    if (!checkEdge(v1,v2)) {
      throw std::runtime_error("Graph edge does not exist!!");
    }
    return (data_.at(v1)).at(v2).weight;
  }
*/

  EdgeType at(vertexName v1,vertexName v2) {
    if (!checkEdge(v1,v2)) {
      throw std::runtime_error("Graph edge does not exist!!");
    }
    return (data_.at(v1)).at(v2);
  }

  void getEdges(vertexName v,std::vector<vertexName>& edge_list) {
    if(!checkElement(v)) return;
    for(auto const iter: data_.at(v)) {
      edge_list.push_back(iter.first);
    }
  }

  size_t getUnvisitedEdgeCount() {
    size_t count = 0;
    for(auto const iter: edge_count_) {
      if(!iter.second) count++;
    }
    return count;
  }

  void updateEdgeCount(vertexName v1,vertexName v2) {
    auto edge_pair = std::make_pair(v1,v2);
    auto iter = edge_count_.find(edge_pair);
    if(iter == edge_count_.end()) {
      printEdgeCount();
      std::cout<<v1<<v2<<std::endl;
      throw std::runtime_error("Edge edge does not exist in count!!");
    }
    if(!iter->second) {
      unvisited_distance -= at(v1,v2).weight;
       --unvisited_edge_count_;
    }
    iter->second++;
  }

  void simulateOdomReadings(vertexName v1,vertexName v2,nav_msgs::Odometry odom,mapping::Odometry& odomreadings,mapping::Timestamps& timestamps) {

    EdgeType edge = at(v1,v2);
    double dist = edge.weight;
    double steps = 0.5,start = 0.0;
    gtsam::Pose2 start_pose;
    ros::Time time = odom.header.stamp;
    odomreadings.insert(std::make_pair(time,odom));
    timestamps.insert(odom.header.stamp);

    float one = 1;
    while((dist - start) >= steps) {

        time += ros::Duration(1.0);
        start += steps;


        odom.header.stamp = time;


        odom.pose.pose.position.x += edge.x_dir*steps;
        odom.pose.pose.position.y += edge.y_dir*steps;
        odomreadings.insert(std::make_pair(odom.header.stamp,odom));


        if(std::modf(start,&one) == 0.0) {
              timestamps.insert(odom.header.stamp);
         }
    }
  }


};

#endif
