#pragma once

#include "vector"
#include "string"
#include <eigen3/Eigen/Core>
#include <optional>

namespace shr_utils {

    std::pair<std::vector<Eigen::MatrixXd>, std::vector<std::string>> load_meshes(const std::string &file_name);

    bool PointInTriangle(const Eigen::Vector2d &p, const Eigen::Vector2d &a, const Eigen::Vector2d &b,
                         const Eigen::Vector2d &c);

    bool PointInMesh(Eigen::Vector3d p, Eigen::MatrixXd verts, Eigen::MatrixXd verts2d);

    std::optional<int> pointInMeshes(const std::vector<Eigen::MatrixXd> &mesh_verts);

    std::optional<std::string> pointInMeshes(Eigen::Vector3d point,
                                             const std::pair<std::vector<Eigen::MatrixXd>, std::vector<std::string>> &mesh_data);

}