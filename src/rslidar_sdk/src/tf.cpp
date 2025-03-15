#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, -0.17)),
        ros::Time::now(),"rslidar_m1", "rslidar_m1_207"));
      broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0.707, 0.707), tf::Vector3(0.0, 1.21, 1.35)),
        ros::Time::now(),"vehicle_frame", "rslidar_m1"));
    r.sleep();
  }
}
