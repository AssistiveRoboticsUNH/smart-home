#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <stdexcept>
#include <cassert>


#include "shr_utils/geometry.hpp"

namespace shr_utils {

    std::pair<std::vector<Eigen::MatrixXd>, std::vector<std::string>> load_meshes(const std::string &file_name) {
        std::vector<Eigen::MatrixXd> out;
        std::vector<std::string> out2;
        Assimp::Importer importer;

        // Load the first mesh
        const aiScene *scene = importer.ReadFile(file_name, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
        if (!scene || !scene->mRootNode) {
            throw std::runtime_error("Failed to load mesh1.obj");
        }

        for (int i = 0; i < scene->mNumMeshes; i++) {
            const aiMesh *mesh = scene->mMeshes[i];

            std::vector<double> vert(mesh->mNumFaces * 3 * 3);
            int count = 0;
            for (int f = 0; f < mesh->mNumFaces; f++) {
                auto simp_ind = mesh->mFaces[f];
                for (int i = 0; i < simp_ind.mNumIndices; i++) {
                    auto ind = simp_ind.mIndices[i];
                    vert[count++] = mesh->mVertices[ind].x;
                    vert[count++] = mesh->mVertices[ind].y;
                    vert[count++] = mesh->mVertices[ind].z;
                }
            }

            Eigen::MatrixXd verts = Eigen::Map<Eigen::MatrixXd>(vert.data(), 3, vert.size() / 3);
            out.push_back(verts);
            out2.push_back(mesh->mName.data);
        }

        return {out, out2};
    }

    bool PointInTriangle(const Eigen::Vector2d &p, const Eigen::Vector2d &a, const Eigen::Vector2d &b,
                         const Eigen::Vector2d &c) {
        double u;
        double v;
        double w;

        auto v0 = b - a;
        auto v1 = c - a;
        auto v2 = p - a;
        float den = v0[0] * v1[1] - v1[0] * v0[1];
        v = (v2[0] * v1[1] - v1[0] * v2[1]) / den;
        w = (v0[0] * v2[1] - v2[0] * v0[1]) / den;
        u = 1.0f - v - w;

        return v >= 0 && v <= 1 && u >= 0 && u <= 1 && w >= 0 && w <= 1;
    }

    bool PointInMesh(Eigen::Vector3d p, Eigen::MatrixXd verts, Eigen::MatrixXd verts2d) {
        Eigen::Vector2d p2 = p.block(0, 0, 2, 1);
        int numInside = 0;
        for (int i = 0; i < verts2d.cols(); i += 3) {
            auto mid = 1.0 / 3.0 * (verts(2, i) + verts(2, i + 1) + verts(2, i + 2));
            if (p[2] - mid > 0) { // less, skip because my ray is up
                continue;
            }
            Eigen::Vector2d point1 = verts2d.col(i);
            Eigen::Vector2d point2 = verts2d.col(i + 1);
            Eigen::Vector2d point3 = verts2d.col(i + 2);
            if (PointInTriangle(p2, point1, point2, point3)) {
                numInside += 1;
            }

        }

        return ((numInside % 2) == 1); // odd means inside
    }

    std::optional<int> pointInMeshes(Eigen::Vector3d point, const std::vector<Eigen::MatrixXd> &mesh_verts) {
        for (int i = 0; i < mesh_verts.size(); i++) {
            auto verts = mesh_verts[i];
            Eigen::MatrixXd verts2d = verts.block(0, 0, 2, verts.cols());
            bool hit = shr_utils::PointInMesh(point, verts, verts2d);
            if (hit) {
                return i;
            }
        }
        return {};
    }

    std::optional<std::string> pointInMeshes(Eigen::Vector3d point,
                                             const std::pair<std::vector<Eigen::MatrixXd>, std::vector<std::string>> &mesh_data) {
        auto mesh_verts = mesh_data.first;
        auto mesh_names = mesh_data.second;
        for (int i = 0; i < mesh_verts.size(); i++) {
            auto verts = mesh_verts[i];
            Eigen::MatrixXd verts2d = verts.block(0, 0, 2, verts.cols());
            bool hit = shr_utils::PointInMesh(point, verts, verts2d);
            if (hit) {
                return mesh_names[i];
            }
        }
        return {};
    }
}