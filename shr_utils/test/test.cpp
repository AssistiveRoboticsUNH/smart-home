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
    for (const auto &entry: fs::directory_iterator(folderPath)) {
        if (fs::is_regular_file(entry)) {
            fs::path filePath = entry.path();

            std::string mesh_file_1 = filePath.filename().string();

            std::cout << " filename " << mesh_file_1 << " /n" << std::endl;

            auto mesh_data = shr_utils::load_meshes(file_path / mesh_file_1);

//            Eigen::Vector3d couch_point = {1.012, 1.104, 0.5};
//            if (auto value = shr_utils::pointInMeshes(couch_point, mesh_data)) {
//                if (shr_utils::pointInMeshes(couch_point, mesh_data).value() == "bathroom"){
//                    std::cout << " correct hit" << std::endl;
//                }
//                else{
//                    std::cout << " wrong hit" << std::endl;
//                }
//            } else{std::cout << " wrong hit" << std::endl;}
//
//
//            Eigen::Vector3d living_room_point = {-2.24, 1.03, 0.5};
//            if (auto value = shr_utils::pointInMeshes(couch_point, mesh_data)) {
//                if (shr_utils::pointInMeshes(living_room_point, mesh_data).value() == "living_room"){
//                    std::cout << " correct hit" << std::endl;
//                }
//                else{
//                    std::cout << " wrong hit" << std::endl;
//                }
//            } else{std::cout << " wrong hit" << std::endl;}
//
//
//            Eigen::Vector3d kitchen_point = {4.3, 0.37, 0.5};
//            if (auto value = shr_utils::pointInMeshes(kitchen_point, mesh_data)) {
//                if (shr_utils::pointInMeshes(couch_point, mesh_data).value() == "kitchen"){
//                    std::cout << " correct hit" << std::endl;
//                }
//                else{
//                    std::cout << " wrong hit" << std::endl;
//                }
//            } else{std::cout << " wrong hit" << std::endl;}

            Eigen::Vector3d dining_point = { 4.5, -3.96, 0.5};
            if (auto value = shr_utils::pointInMeshes(dining_point, mesh_data)) {
                auto val = shr_utils::pointInMeshes(dining_point, mesh_data).value() ;
                    std::cout << " correct hit1  " << val << std::endl;

            } else{std::cout << " wrong hit " << std::endl;}

            Eigen::Vector3d dining_point2 = {  3.96, -4.5,0.5};
            if (auto value = shr_utils::pointInMeshes(dining_point2, mesh_data)) {
                auto val = shr_utils::pointInMeshes(dining_point2, mesh_data).value() ;
                std::cout << " correct hit 2  " << val << std::endl;

            } else{std::cout << " wrong hit2 " << std::endl;}
        }
    }
//    Eigen::Vector3d hallway_point = {-1.52, 1.49, 0.523606};
//    assert(shr_utils::pointInMeshes(kitchen_point, mesh_data).value() == "hallway");

    return 0;

}
