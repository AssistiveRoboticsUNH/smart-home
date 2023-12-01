#include <assimp/postprocess.h>
#include <filesystem>

//ros
#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "shr_utils/geometry.hpp"

namespace fs = std::filesystem;

int main(int argc, char *argv[]) {
    std::filesystem::path pkg_dir = ament_index_cpp::get_package_share_directory("shr_utils");
    auto file_path = pkg_dir / "test" / "data" / "mesh_test";

    fs::path folderPath(file_path);
//    for (const auto &entry: fs::directory_iterator(folderPath)) {
//        if (fs::is_regular_file(entry)) {
//            fs::path filePath = entry.path();
//            std::string mesh_file_1 = filePath.filename().string();
//            std::cout << " filename " << mesh_file_1 << " /n" << std::endl;
//            auto mesh_data = shr_utils::load_meshes(file_path / mesh_file_1);
//
//            Eigen::Vector3d dining_point = {4.5, -3.96, 0.5};
//            if (auto value = shr_utils::pointInMeshes(dining_point, mesh_data)) {
//                auto val = shr_utils::pointInMeshes(dining_point, mesh_data).value();
//                std::cout << " correct hit1  " << val << std::endl;
//
//            } else { std::cout << " wrong hit " << std::endl; }
//
//            Eigen::Vector3d dining_point2 = {3.96, -4.5, 0.5};
//            if (auto value = shr_utils::pointInMeshes(dining_point2, mesh_data)) {
//                auto val = shr_utils::pointInMeshes(dining_point2, mesh_data).value();
//                std::cout << " correct hit 2  " << val << std::endl;
//
//            } else { std::cout << " wrong hit2 " << std::endl; }
//
////            - 0.5830000042915344
////            - 3.765000104904175
//        }
//    }

    std::string mesh_file = "/home/olagh/smart-home/src/smart-home/external/particle_filter_mesh/config/labxf.obj";
    auto mesh_data_ = shr_utils::load_meshes(mesh_file);
//    //door no collision
//    // no hit
//    Eigen::Vector3d dining_point = {-0.92, -5.10, -0.5};
//    if (auto value = shr_utils::pointInMeshes(dining_point, mesh_data_)) {
//        auto val = shr_utils::pointInMeshes(dining_point, mesh_data_).value();
//        std::cout << " correct hit1  " << val << std::endl;
//
//    } else { std::cout << " wrong hit " << std::endl; }

    //bedroom
    Eigen::Vector3d dining_point2 = {1.700, 0.130, -0.5};
    if (auto value = shr_utils::pointInMeshes(dining_point2, mesh_data_)) {
        auto val = shr_utils::pointInMeshes(dining_point2, mesh_data_).value();
        std::cout << " correct hit 2  " << val << std::endl;
    } else { std::cout << " wrong hit2 " << std::endl; }

    // livingtoom door
    Eigen::Vector3d dining_point3 = {1.700, 0.130, -0.5};
    if (auto value = shr_utils::pointInMeshes(dining_point3, mesh_data_)) {
        auto val = shr_utils::pointInMeshes(dining_point3, mesh_data_).value();
        std::cout << " correct hit 2  " << val << std::endl;
    } else { std::cout << " wrong hit2 " << std::endl; }

    // kitchen door
    Eigen::Vector3d dining_point4 = {1.700, 0.130, -0.5};
    if (auto value = shr_utils::pointInMeshes(dining_point4, mesh_data_)) {
        auto val = shr_utils::pointInMeshes(dining_point4, mesh_data_).value();
        std::cout << " correct hit 2  " << val << std::endl;
    } else { std::cout << " wrong hit2 " << std::endl; }

    // bathroom door
    Eigen::Vector3d dining_point5 = {1.700, 0.130 , -0.5};
    if (auto value = shr_utils::pointInMeshes(dining_point5, mesh_data_)) {
        auto val = shr_utils::pointInMeshes(dining_point5, mesh_data_).value();
        std::cout << " correct hit 2  " << val << std::endl;
    } else { std::cout << " wrong hit2 " << std::endl; }

//    // rand  non halway
//    Eigen::Vector3d dining_point6= {-1.4516, 3.7272, -0.5};
//    if (auto value = shr_utils::pointInMeshes(dining_point6, mesh_data_)) {
//        auto val = shr_utils::pointInMeshes(dining_point6, mesh_data_).value();
//        std::cout << " correct hit 2  " << val << std::endl;
//    } else { std::cout << " wrong hit2 " << std::endl; }
//    // rand non
//    Eigen::Vector3d dining_point7 = {-3.2751, -3.5666, -0.5};
//    if (auto value = shr_utils::pointInMeshes(dining_point7, mesh_data_)) {
//        auto val = shr_utils::pointInMeshes(dining_point7, mesh_data_).value();
//        std::cout << " correct hit 2  " << val << std::endl;
//    } else { std::cout << " wrong hit2 " << std::endl; }

    return 0;

}
