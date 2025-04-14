#include "edge.h"


Edge::Edge(){
}
  
Edge::Edge(int edge_id, Type type, int from, int to, Eigen::Vector3d& T, Eigen::Matrix3d& information):
    _edge_id(edge_id), _type(type), _from(from), _to(to), _T(T), _information(information){
}