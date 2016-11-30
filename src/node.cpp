#include <graph_search_sim/node.h>

 Node::Node() {

 }

 Node::Node(vertexName name,double odomUnc,double distance,size_t visit_count):name_(name),odomUnc_(odomUnc),distance_(distance),visit_count_(visit_count) {

 }

