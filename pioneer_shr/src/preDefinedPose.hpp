#include <geometry_msgs/Pose.h>
#include <string>

class PreDefinedPose {
public:
    PreDefinedPose(const geometry_msgs::Pose& pose, const std::string name)
            : pose(pose), name(name) {}

    geometry_msgs::Pose getPose() const { return pose; }

    std::string getName() const { return name; }

private:
    const geometry_msgs::Pose pose;
    const std::string name;
};
