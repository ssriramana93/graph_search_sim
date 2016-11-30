#ifndef GRAPH_COMMON_H
#define GRAPH_COMMON_H


#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <nav_msgs/Odometry.h>
#include <gtsam/slam/PriorFactor.h>


#include <aslam_demo/mapping/map_processing.h>
#include <aslam_demo/mapping/probability_map.h>
#include <aslam_demo/factors/key_generator.h>
#include <aslam_demo/factors/laser_scan_factor.h>

#include <aslam_demo/mapping/csm_processing.h>
#include <aslam_demo/mapping/optimization_processing.h>
#include <aslam_demo/mapping/laserscan_processing.h>
#include <aslam_demo/mapping/odometry_processing.h>

#include <aslam_demo/aslam/aslam.h>
typedef std::string vertexName;

struct EdgeType {
  vertexName from;
  vertexName to;
  double weight;
  double x_dir;
  double y_dir;
  int visit_count = 0;


  EdgeType(vertexName f,vertexName t,double w,double x,double y) {
    from = f;
    to = t;
    weight = w;
    x_dir = x;
    y_dir = y;
  }
};
typedef std::map<vertexName,std::map<vertexName,EdgeType> > graphData;

#endif
