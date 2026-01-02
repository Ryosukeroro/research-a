/**
*/

#include <map_to_pointcloud/map_to_pointcloud.hpp>

int main(int argc, char** argv)
{
    // 初期化
    ros::init(argc, argv, "map_to_pointcloud");
    ros::NodeHandle nh("");

    // 地図生成クラスのインスタンス化
    MapToPointCloudData ins(nh);

    // コールバック受信待機
    ros::spin();

    return (0);
}