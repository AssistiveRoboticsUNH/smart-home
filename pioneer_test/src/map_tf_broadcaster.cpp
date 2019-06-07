#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "map_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Transform transformSonar;

  const double pi = 3.14159265358979323846264338328 ;

  ros::Rate rate(60.0);
  while (node.ok()){
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    tf::Quaternion q;
    q.setEuler(0.0, 0.0, 0.0);    
    transform.setRotation( q );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "map"));
    rate.sleep();
  }
  return 0;
};

