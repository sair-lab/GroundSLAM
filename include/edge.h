#ifndef EDGE_H_
#define EDGE_H_

#include <string>
#include <Eigen/Dense>
#include <Eigen/SparseCore>

#include "utils.h"


struct Edge{
  enum Type {
    Odom = 0,
    KCC = 1,
    Loop = 2,
    Others = 3,
  };

  int _edge_id;
  Type _type;
  int _from;
  int _to;
  Eigen::Vector3d _T;
  Eigen::Matrix3d _information;

  Edge();
  Edge(int edge_id, Type type, int from, int to, Eigen::Vector3d& T, Eigen::Matrix3d& information);
};

typedef std::shared_ptr<Edge> EdgePtr;

#endif  // EDGE_H_