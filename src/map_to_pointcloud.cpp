/**
* @file map_to_pointcloud.cpp
*/

#include <map_to_pointcloud/map_to_pointcloud.hpp>

MapToPointCloudData::MapToPointCloudData(ros::NodeHandle node)
{
    sensor_msgs::PointField field;
    std::string sub_topic_name;
    std::string pub_topic_name;
    ros::NodeHandle param_node("~");

    // パラメータ読み込み
    getParam(param_node, "map_topic_name", sub_topic_name, std::string("/map")); // 点群化する地図の取得トピック名
    getParam(param_node, "cloud_topic_name", pub_topic_name, sub_topic_name + std::string("_cloud"));
    getParam(param_node, "occupied_thresh", occupied_thresh_, 50.0); // 占有してると判断する確率のしきい値
    getParam(param_node, "global_frame", global_frame_, std::string("lidar")); // 変換後の点群データのフレーム名
    getParam(param_node, "sensor_height", sensor_height_, 0.35); // 点群センサーの擬似的な高さ
    getParam(param_node, "publish_times", publish_times_, 0); // 配信回数

    // サブスクライバ定義
    sub_map = node.subscribe(sub_topic_name, ROS_QUEUE_SIZE_1, &MapToPointCloudData::recvMapCB, this);

    // パブリッシャ定義
    pub_cloud = node.advertise<sensor_msgs::PointCloud2>(pub_topic_name.c_str(), 10, true);
    

}

MapToPointCloudData::~MapToPointCloudData()
{
    // 何もしない
}

void MapToPointCloudData::recvMapCB(const nav_msgs::OccupancyGridPtr msg)
{
    unsigned int map_size = msg->data.size(); // 地図のサイズ
    std::vector<int8_t> map_data = msg->data; // 地図データ
    map_cell_width_= msg->info.width;
    map_cell_height_ = msg->info.height;
    origin_x_ = msg->info.origin.position.x;
    origin_y_ = msg->info.origin.position.y;
    resolution_ = msg->info.resolution;

    std::vector<PointXYZ> cloud_data;
    cloud_data.reserve(map_size);

    // 地図データをクロールして閾値以上のセル座標を抜き出す
    for(unsigned int idx = 0; idx < map_size; ++idx)
    {
        // 閾値判定
        if( map_data[idx] == UNKNOWN_COST_GRIDMAP || // UNKNOWN = -1
            map_data[idx] == FREESPACE_COST_GRIDMAP || // FREESPACE = 0
            map_data[idx] < occupied_thresh_ ) // 閾値となる占有確率以下
        {
            continue;
        }

        unsigned int cell_x, cell_y; // 該当セルの座標値
        double x, y; // 原点からの距離

        indexToCells(idx, cell_x, cell_y); // セル座標を求める
        mapToWorld(cell_x, cell_y, x, y); // 地図座標系からワールド座標系に変換

        // データの格納
        cloud_data.emplace_back((float)x, (float)y, sensor_height_);
    }

    // 配信する点群データの作成
    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header = msg->header;
    cloud_msg.header.frame_id = global_frame_; // パラメータで設定したフレーム (例: "lidar")

    // 2. 「中身はx, y, z のfloatだよ」という説明書を作る
    sensor_msgs::PointField field_x, field_y, field_z;
    field_x.name = "x"; field_x.offset = 0; field_x.datatype = sensor_msgs::PointField::FLOAT32; field_x.count = 1;
    field_y.name = "y"; field_y.offset = 4; field_y.datatype = sensor_msgs::PointField::FLOAT32; field_y.count = 1;
    field_z.name = "z"; field_z.offset = 8; field_z.datatype = sensor_msgs::PointField::FLOAT32; field_z.count = 1;

    cloud_msg.fields.push_back(field_x);
    cloud_msg.fields.push_back(field_y);
    cloud_msg.fields.push_back(field_z);

    // 3. データサイズなどの設定
    int n_points = cloud_data.size();
    cloud_msg.height = 1;
    cloud_msg.width = n_points;
    cloud_msg.is_bigendian = false;
    cloud_msg.point_step = sizeof(PointXYZ); // 構造体のサイズ (12byte)
    cloud_msg.row_step = cloud_msg.point_step * n_points;
    cloud_msg.is_dense = true;

    // 4. データを流し込む (これが makeShared の代わりの作業!)
    cloud_msg.data.resize(n_points * sizeof(PointXYZ));

    if (n_points > 0) {
        // vectorの中身をそのままメッセージのdata領域にコピー
        memcpy(&cloud_msg.data[0], &cloud_data[0], cloud_msg.data.size());
    }

    // 5. 送信!
    pub_cloud.publish(cloud_msg);

#ifndef DEBUG
     ROS_INFO("!!!!!Pub PointCloud Data!!!!!");
#endif






}