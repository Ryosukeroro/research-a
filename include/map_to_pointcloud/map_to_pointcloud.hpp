/**
* @file map_to_pointcloud.hpp
 */

#ifndef MAP_TO_POINTCLOUD_H_
#define MAP_TO_POINTCLOUD_H_

#include <ros/ros.h> // 基本ライブラリ
#include <nav_msgs/OccupancyGrid.h> // グリッドマップ
#include <sensor_msgs/PointCloud2.h> // 配信するデータ型
#include <sensor_msgs/PointField.h>

#include "utilities.h"

// 占有率定義
#define FREESPACE_COST_GRIDMAP 0
#define UNKNOWN_COST_GRIDMAP   -1

/**
*/
class MapToPointCloudData
{
    public:

        /**
         */
        MapToPointCloudData();

        /**
        */
        ~MapToPointCloudData();

        /**
        */
        void recvMapCB(const nav_msgs::OccupancyGridPtr msg);

        /**
        */
       inline void mapToWorld(unsigned int map_x, unsigned int map_y, double& world_x, double& world_y) const
       {
            world_x = origin_x_ + (map_x + 0.5) * resolution_;
            world_y = origin_y_ + (map_y + 0.5) * resolution_;
       }

        /**
        */
        inline unsigned int getIndex(unsigned int mx, unsigned int my) const
        {
            return my * map_cell_width_ + mx;
        }

        /**
        */
        inline void indexToCells(unsigned int index, unsigned int& mx, unsigned int& my) const
        {
            my = index / map_cell_width_;
            mx = index - (my * map_cell_width_);
        }

    private:
        ros::Publisher pub_cloud; // 変換後の点群データのパブリッシャ
        ros::Subscriber sub_map; // 変換する地図のサブスクライバ

        double occupied_threth_; // 完全占有として判断する占有確率(0~100%)の閾値
        std::string global_frame_; // 地図のフレーム名
        unsigned int map_cell_width_; // 地図の幅[cell]
        unsigned int map_cell_height_; // 地図の高さ[cell]
        int publish_times_; // 配信回数
        double origin_x_; // 原点のX座標
        double origin_y_; // 原点のY座標
        double resolution_; // 解像度
        double sensor_height_; // 点群センサーの擬似的な高さ
};

#endif
