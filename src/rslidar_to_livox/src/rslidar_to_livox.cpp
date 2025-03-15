#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <livox_ros_driver/CustomMsg.h>  // Livox 自定义点云格式

class RSLiDARToLivox
{
public:
    RSLiDARToLivox()
    {
        // 订阅 RSLiDAR 点云数据
        sub_ = nh_.subscribe("/rslidar_points_80", 10, &RSLiDARToLivox::pointCloudCallback, this);
        // 发布转换后的 Livox 点云数据
        pub_ = nh_.advertise<livox_ros_driver::CustomMsg>("/livox/lidar", 10);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        livox_ros_driver::CustomMsg livox_msg;
        livox_msg.header = msg->header;
        livox_msg.timebase = ros::Time::now().toNSec(); // 以当前时间为基准

        // 解析 PointCloud2 数据
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_intensity(*msg, "intensity");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity)
        {
            livox_ros_driver::CustomPoint point;
            point.offset_time = 0;  // 这里可以使用真实时间偏移
            point.x = *iter_x;
            point.y = *iter_y;
            point.z = *iter_z;
            point.reflectivity = *iter_intensity;
            point.tag = 0;  // 可根据需求设置
            point.line = 0; // 如果是多线雷达，可设置不同的 line 号
            
            livox_msg.points.push_back(point);
        }

        livox_msg.point_num = livox_msg.points.size();

        // 发布转换后的 Livox 点云数据
        pub_.publish(livox_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rslidar_to_livox_node");
    RSLiDARToLivox converter;
    ros::spin();
    return 0;
}
