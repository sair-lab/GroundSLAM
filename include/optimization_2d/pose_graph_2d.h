#ifndef OPTIMIZATION_2D_POSE_GRAPH_2D_H_
#define OPTIMIZATION_2D_POSE_GRAPH_2D_H_

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <ceres/ceres.h>

#include "angle_local_parameterization.h"
#include "pose_graph_2d_error_term.h"
#include "types.h"


namespace ceres {
namespace optimization_2d {

void BuildOptimizationProblem(const std::vector<Constraint2d>& constraints,
                              std::map<int, Pose2d>* poses,
                              ceres::Problem* problem);

void BuildOptimizationProblemWithScale(const std::vector<Constraint2d>& constraints,
                              std::vector<ScaleData>& scale_data,
                              const std::vector<int>& scale_data_idx,
                              std::map<int, Pose2d>* poses,
                              ceres::Problem* problem);

bool SolveOptimizationProblem(ceres::Problem* problem);

}  // namespace optimization_2d
}  // namespace ceres

#endif  // OPTIMIZATION_2D_POSE_GRAPH_2D_H_