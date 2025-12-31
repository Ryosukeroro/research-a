#ifndef POINT2_H
#define POINT2_H

// ROSのPointCloud2にそのまま流し込める構造体
// float型を使うのがROSのデファクトスタンダードです
struct PointXYZ {
    float x;
    float y;
    float z; //飾り

    // コントラクタ (使うときは x, y だけ指定すれば z=0 になるようにする)
    PointXYZ(float _x, float _y, float _z = 0.0f) : x(_x), y(_y), z(_z) {}
    PointXYZ() : x(0), y(0), z(0) {}
};

#endif