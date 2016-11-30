#include <graph_search_sim/node.h>
#include <graph_search_sim/branch.h>
#include <graph_search_sim/graph_search.h>
#include <graph_search_sim/graph_common.h>


void creategraph(Graph& map) {
  std::cout<<"Create Graph"<<std::endl;
  vertexName name;
  std::vector<vertexName> edges;
  std::vector<EdgeType> weight;
  edges.push_back("B");
  weight.push_back(EdgeType("A","B",10.0,1.0,0.0));
  map.addNodeWithEdges("A",edges,weight);
  edges.clear();
  weight.clear();
  edges.push_back("A");
  edges.push_back("C");
  edges.push_back("D");
  weight.push_back(EdgeType("B","A",10.0,-1.0,0.0));
  weight.push_back(EdgeType("B","C",20.0,0.0,-1.0));
  weight.push_back(EdgeType("B","D",10.0,1.0,0.0));
  map.addNodeWithEdges("B",edges,weight);
  edges.clear();
   weight.clear();
  edges.push_back("B");
  edges.push_back("E");
  weight.push_back(EdgeType("C","B",20.0,0.0,1.0));
  weight.push_back(EdgeType("C","E",10.0,1.0,0.0));
  map.addNodeWithEdges("C",edges,weight);
  edges.clear();
  weight.clear();
  edges.push_back("B");
  edges.push_back("E");
  edges.push_back("F");
  weight.push_back(EdgeType("D","B",10.0,-1.0,0.0));
  weight.push_back(EdgeType("D","E",20.0,0.0,-1.0));
  weight.push_back(EdgeType("D","F",10.0,1.0,0.0));
  map.addNodeWithEdges("D",edges,weight);
  edges.clear();
  weight.clear();
  edges.push_back("C");
  edges.push_back("G");
  edges.push_back("D");
  weight.push_back(EdgeType("E","C",10.0,-1.0,0.0));
  weight.push_back(EdgeType("E","G",10.0,1.0,0.0));
  weight.push_back(EdgeType("E","D",20.0,0.0,1.0));
  map.addNodeWithEdges("E",edges,weight);
  edges.clear();
  weight.clear();
  edges.push_back("D");
  edges.push_back("G");
  weight.push_back(EdgeType("F","D",10.0,-1.0,0.0));
  weight.push_back(EdgeType("F","G",20.0,0.0,-1.0));
  map.addNodeWithEdges("F",edges,weight);
  edges.clear();
  weight.clear();
  edges.push_back("E");
  edges.push_back("F");
  weight.push_back(EdgeType("G","E",10.0,1.0,0.0));
  weight.push_back(EdgeType("G","F",20.0,0.0,1.0));
  map.addNodeWithEdges("G",edges,weight);
  edges.clear();
  weight.clear();

}

int main(int argc, char *argv[]) {
  ros::init(argc,argv,"node");
  ros::NodeHandle n;
  std::cout<<"Program Entered"<<std::endl;
  Graph map(n);
  std::cout<<"Map created"<<std::endl;
  creategraph(map);
  std::cout<<"Create Graph"<<std::endl;
  vertexName start_name = "A";
  std::cout<<"start name"<<std::endl;;
  GraphSearch gs(n,map,start_name);
  std::cout<<"Graph search inited"<<std::endl;
  gs.calcBestPath();
  std::cout<<"calcBestPath"<<std::endl;
  std::cout<<"BestScore"<<gs.best_score_<<std::endl;
  gs.best_branch_->printNodes();
  return 0;

}
