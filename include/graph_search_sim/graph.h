
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

  std::vector<vertexName> odd_vertices_;

  Graph(ros::NodeHandle& n):n_(n) {
  }

  Graph(ros::NodeHandle& n,graphData& data,std::vector<vertexName>& odd_vertices):n_(n),odd_vertices_(odd_vertices) {
    setData(data);
    unvisited_edge_count_ = getUnvisitedEdgeCount();
  }

  Graph(const Graph& graph):n_(graph.n_),edge_count_(graph.edge_count_),unvisited_edge_count_(graph.unvisited_edge_count_),unvisited_distance(graph.unvisited_distance),odd_vertices_(graph.odd_vertices_) {
    setData(graph.data());
  }

  void addNodeWithEdges(vertexName name,std::vector<vertexName> edges,std::vector<EdgeType> data) {
    std::map<vertexName,EdgeType> edge_list;
    if(edges.size() % 2) {
          odd_vertices_.push_back(name);
       }
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

  void resetEdgeCount() {
    for(auto const &iter: edge_count_) {
      edge_count_[iter.first] = 0;
    }
    unvisited_edge_count_ = edge_count_.size();
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


  void getChinesePostmanPath(vertexName start_vertex,std::vector<vertexName>& cpp_path) {
    std::vector<vertexName> visited;
    std::vector<std::vector<std::pair<vertexName,vertexName> > > pairs_list;
    std::vector<std::pair<vertexName,vertexName> > min_pairs;
    double min_distance = 1000000000000000000;
    std::vector<std::pair<vertexName,vertexName> > pairs;
    std::vector<std::vector<vertexName> > paths,min_path;
   /* for(int i = 0; i < odd_vertices_.size()/2;i++) {
      size_t index = 0;
      for(int j = i+1; j < odd_vertices_.size();j++) {
        if(i == j) continue;
        pairs_list[index].push_back(std::make_pair(odd_vertices_[i],odd_vertices_[j]));
        ++index;
      }
    }*/
    std::sort(odd_vertices_.begin(),odd_vertices_.end());
    for(int i = 0; i < odd_vertices_.size();i++) {
          std::cout<<"P"<<odd_vertices_[i]<<"\t";

        }
    std::cout<<std::endl;
   // while(1);

    do {
      for(int i = 0; i < odd_vertices_.size()/2;i++) {
        std::cout<<"P"<<odd_vertices_[2*i]<<"\t";

      }
      std::cout<<std::endl;
      for(int i = 0; i < odd_vertices_.size()/2;i++) {
        std::cout<<"P"<<odd_vertices_[2*i]<<odd_vertices_[2*i+1]<<std::endl;
        pairs.push_back(std::make_pair(odd_vertices_[2*i],odd_vertices_[2*i+1]));
      }
      pairs_list.push_back(pairs);
      pairs.clear();
    } while(std::next_permutation(odd_vertices_.begin(),odd_vertices_.end()));

    for(auto const pairs: pairs_list) {
      double distance = 0;
     // std::cout<<"Size"<<pairs.size()<<std::endl;

      for(auto const pair: pairs) {
        auto v1 = pair.first;
        auto v2 = pair.second;
        std::vector<vertexName> path;
        djikstra(v1,v2,path);
        for(int k = 0;k < path.size() - 1;k++) {
             distance += at(path[k],path[k+1]).weight;
        }
     //   std::cout<<"D\t"<<distance<<std::endl;
        paths.push_back(path);
      }
      if(min_distance > distance) {
        min_distance = distance;
        min_pairs = pairs;
        min_path = paths;

      }
      paths.clear();
    }


    std::map<std::pair<vertexName,vertexName>,int,compareEdge> temp;
    for(auto const path: min_path) {
      for(int  i = 0;i < path.size()-1;i++ ) {
        if (temp.find(std::make_pair(path[i],path[i+1])) == temp.end()) {
          temp.insert(std::make_pair(std::make_pair(path[i],path[i+1]),0));
        }
      }
    }
    min_pairs.clear();
    for(auto const iter: temp) {
      min_pairs.push_back(iter.first);
    }
    std::cout<<"Size"<<pairs_list[1].size()<<"\t"<<min_path.size()<<std::endl;
      for(auto const iter:min_pairs) {
        std::cout<<"Pairs: "<<iter.first<<"\t"<<iter.second<<std::endl;
      }
    auto v = start_vertex;
    while(1) {
      cpp_path.push_back(v);
      std::vector<vertexName> edges;
      getEdges(v,edges);
      bool added = false;
      for(auto const iter:edges) {
        auto vp = std::make_pair(v,iter);
        for(auto const iter: min_pairs) {
          if(((vp.first == iter.first and vp.second == iter.second) || (vp.second == iter.first and vp.first == iter.second)) and !edge_count_[vp]) {
              updateEdgeCount(vp.first,vp.second);
              cpp_path.push_back(vp.second);
           //   cpp_path.push_back(vp.first);

              v = vp.first;
              added = true;
              break;
         }
        }

        if(!edge_count_[vp]) {
          updateEdgeCount(vp.first,vp.second);
          v = vp.second;
          added = true;
          break;
        }
        if (added) {
               break;
             }


      }
      if (added == false) {
        break;
      }
    }
    for(auto const iter:cpp_path) {
        std::cout<<iter<<"\t";
    }
    std::cout<<std::endl;
    resetEdgeCount();

    }



  void djikstra(vertexName v1,vertexName v2,std::vector<vertexName>& shortest_path) {
  //  std::cout<<"Djisktra entered\t"<<v1<<"\t"<<v2 <<std::endl;
    std::vector<bool> processed;
    std::vector<double> dist;
    std::map<vertexName,vertexName> parent;
    std::vector<vertexName> nodes;
    for (auto const iter: data_) {
      nodes.push_back(iter.first);
    }
    for (size_t i = 0;i < data_.size();i++) {
      processed.push_back(false);
      if(v1 == nodes[i]) {
        dist.push_back(0.0);
        continue;
      }
      dist.push_back(100000000.0);
    }


    while(1) {

      int least_node = 0,min_dist = 10000;
      for(int i=0;i < dist.size();i++) {
//        std::cout<<"D&P"<<dist[i]<<"\t"<<processed[i]<<std::endl;

        if(min_dist > dist[i] && !processed[i]) {
          least_node = i;
          min_dist = dist[i];
        }
      }
   //   std::cout<<"Stop 1"<<nodes[least_node] <<std::endl;

      if(v2 == nodes[least_node]) {
        break;
      }
      processed[least_node] = true;
 //     std::cout<<"Stop 2"<<v2<<std::endl;
      std::vector<vertexName> edges;
      getEdges(nodes[least_node],edges);
      for(int j=0;j < dist.size();j++) {
        if(!processed[j] && (std::find(edges.begin(),edges.end(),nodes[j]) != edges.end())) {
          if(dist[least_node] + at(nodes[least_node],nodes[j]).weight < dist[j]) {
            dist[j] = dist[least_node] + at(nodes[least_node],nodes[j]).weight;
            parent.insert(std::make_pair(nodes[j],nodes[least_node]));
          }
     //     std::cout<<"Node - Dist"<<nodes[j]<<"\t"<<dist[j]<<std::endl;

        }
      }
    }
    auto v = v2;
    while(v != v1) {
      shortest_path.push_back(v);
      v = parent.at(v);

    }
    shortest_path.push_back(v1);
  }

};


#endif
