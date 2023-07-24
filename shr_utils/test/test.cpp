#include <assimp/postprocess.h>
#include <filesystem>

//ros
#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "shr_utils/geometry.hpp"

int main(int argc, char *argv[]) {
    std::filesystem::path pkg_dir = ament_index_cpp::get_package_share_directory("shr_utils");
    auto file_path = pkg_dir / "test" / "data";
    auto mesh_file_1 = (file_path / "cube.obj").string();
    auto mesh_data = shr_utils::load_meshes(mesh_file_1);

    Eigen::Vector3d couch_point = {-2.14135, -1.1564, 0.823606};
    assert(shr_utils::pointInMeshes(couch_point, mesh_data).value() == "couch");

    Eigen::Vector3d living_room_point = {-0.011147, 0.429234, 0.823606};
    assert(shr_utils::pointInMeshes(living_room_point, mesh_data).value() == "living_room");


    Eigen::Vector3d kitchen_point = {-1.29247, -1.12437, 0.823606};
    assert(shr_utils::pointInMeshes(kitchen_point, mesh_data).value() == "kitchen");

    return 0;

}
