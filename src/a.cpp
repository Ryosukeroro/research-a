#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <thread>
#include <mutex>
#include <queue>
#include "point.h"

/*定義*/
#define MAX_iteration 30

/*収束判定の閾値*/
#define EPS  0.001

/*微小変位*/
#define delta 1.0e-7

/*学習率*/
#define learning_rate 1.2

std::vector<Point> target;

FILE* gnuplot_pipe = nullptr;

void plot(const std::vector<Point>& source) {
    if (!gnuplot_pipe) return;

    fprintf(gnuplot_pipe, "set size ratio 1\n");
    fprintf(gnuplot_pipe, "set xrange [-10:10]\n");
    fprintf(gnuplot_pipe, "set yrange [-10:10]\n");
    
    // プロットのためのデータを送信
    fprintf(gnuplot_pipe, "plot '-' with points pointtype 7 pointsize 0.5 lc rgb 'red' title 'Source points', ");
    fprintf(gnuplot_pipe, "'-' with points pointtype 7 pointsize 0.5 lc rgb 'blue' title 'File points'\n");

    // レーザースキャンの点群をプロット
    for (const auto& point : source) {
        fprintf(gnuplot_pipe, "%f %f\n", point.x, point.y);
    }
    fprintf(gnuplot_pipe, "e\n");

    // ファイルからの点群をプロット
    for (const auto& point : target) {
        fprintf(gnuplot_pipe, "%f %f\n", point.x, point.y);
    }
    fprintf(gnuplot_pipe, "e\n");

    fflush(gnuplot_pipe);
}

std::vector<Point> readPointsFromFile(const std::string& file_path) {
    std::vector<Point> points;
    std::ifstream infile(file_path);
    if (!infile.is_open()) {
        ROS_ERROR("Failed to open file: %s", file_path.c_str());
        return points;
    }

    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        Point point;
        if (iss >> point.x >> point.y) {
            points.push_back(point);
        }
    }

    infile.close();
    return points;
}

void plotAsync(const std::vector<Point>& source) {
    std::thread plot_thread([source]() {
        plot(source);
    }
    );
    plot_thread.detach(); //メインスレッドから切り離して非同期で実行

}

float distance(const Point& points, const Point& point){
    return sqrt((points.x - point.x) * (points.x - point.x) + (points.y - point.y) * (points.y - point.y));

}

int findClosestPoint(const Point& point, const std::vector<Point>& target){
    int Index = -1;
    float minDist = std::numeric_limits<float>::max();
    for(size_t i = 0; i < target.size(); ++i){
        float dist = distance(target[i], point);
        if(dist < minDist){
            minDist = dist;
            Index = i;
        }
    }
    return Index;

}

float diffx(Point Target, Point SOurce){
    float fx_delta = (Target.x - (SOurce.x + delta)) * (Target.x - (SOurce.x + delta)) + (Target.y - SOurce.y) * (Target.y - SOurce.y);
    float fx = (Target.x - SOurce.x) * (Target.x - SOurce.x) + (Target.y - SOurce.y) * (Target.y - SOurce.y);
    return (fx_delta - fx) / delta;
}

float diffy(Point Target, Point SOurce){
    float fx_delta = (Target.x - SOurce.x) * (Target.x - SOurce.x) + (Target.y - (SOurce.y + delta)) * (Target.y - (SOurce.y + delta));
    float fx = (Target.x - SOurce.x) * (Target.x - SOurce.x) + (Target.y - SOurce.y) * (Target.y - SOurce.y);
    return (fx_delta - fx) / delta;
}

float difftheta(Point Target, Point SOurce){
    float fx_delta = (Target.x - ((SOurce.x)* cos(delta*M_PI/180)-(SOurce.y)* sin(delta*M_PI/180)))* (Target.x - ((SOurce.x)* cos(delta*M_PI/180)-(SOurce.y)* sin(delta*M_PI/180))) + (Target.y - ((SOurce.x) * (sin(delta*M_PI/180)) + (SOurce.y) * cos(delta*M_PI/180))) * (Target.y - ((SOurce.x) * (sin(delta*M_PI/180)) + (SOurce.y) * cos(delta*M_PI/180)));
    float fx = (Target.x - SOurce.x) * (Target.x - SOurce.x) + (Target.y - SOurce.y) * (Target.y - SOurce.y);
    return (fx_delta - fx) / delta;
}



void icp_scan_matching( const std::vector<Point>& Source){//, const std::vector<Point>& target){
   std::vector<Point> transformed_source = Source;
   float previous_error_sum = std::numeric_limits<float>::max(); // 前回の誤差を最大値で初期化
   for(int iter = 0; iter <= MAX_iteration; ++iter){
    std::vector<Point> target_closest;
    float error_sum = 0;
    double gradDx = 0;
    double gradDy = 0;
    double gradTheta = 0;
    float dx = 0;
    float dy = 0;
    float dth = 0;
    float dtheta = 0;
    for(const auto& Source : transformed_source){
         int index = findClosestPoint(Source,target);
         target_closest.push_back(target[index]);
         Point error = {target[index].x - Source.x, target[index].y - Source.y};
         Point Target = {target[index].x, target[index].y};
         Point SOurce = {Source.x, Source.y};
         error_sum += error.x * error.x + error.y * error.y;
         //std::cout << "error_sum: " << error_sum << std::endl;
         gradDx += diffx(Target, SOurce);
         gradDy += diffy(Target, SOurce);
         gradTheta += difftheta(Target, SOurce);


        //std::cout << "gradTheta: " << gradTheta << std::endl;
    }
    int num_points =Source.size(); // Sourceの点の数を取得

    dx = (-gradDx / num_points) * learning_rate;
    dy = (-gradDy / num_points) * learning_rate;
    dth = (-gradTheta / num_points) * learning_rate;

    // std::cout << "dy: " << dy << std::endl;
     //std::cout << "dth: " << dy << std::endl;

     for(auto& Source : transformed_source){
    //Source.x += dx;
    float x_new = Source.x * cos(dth) - Source.y * sin(dth);
    float y_new = Source.x * sin(dth) + Source.y * cos(dth);
    Source.x = x_new + dx;
    Source.y = y_new + dy;
}

/*収束条件のチェック*/
if(std::abs(previous_error_sum - error_sum) < EPS){
std::cout << "Converged after " << iter << "iterations." << std::endl;
break;
}
previous_error_sum = error_sum;//前回の誤差を更新
   }
   plotAsync(transformed_source);
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{auto start_time = std::chrono::high_resolution_clock::now();
    int count = scan->ranges.size();
    //ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    //ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

    std::vector<Point> points;

    for (int i = 0; i < count; i++) {
        float angle = scan->angle_min + scan->angle_increment * i;
        float range = scan->ranges[i];

        if (range >= scan->range_min && range <= scan->range_max) {
            float x = range * cos(angle);
            float y = range * sin(angle);

            points.push_back({x, y});
            //ROS_INFO("%.2f \t\t %.3f \t\t %.3f \t %.3f", RAD2DEG(angle), range, x, y);
        }
    }

    //std::vector<Point> file_points = readPointsFromFile("/home/ryosuke/catkin_ws/src/research/scan.txt");
 icp_scan_matching(points);
  auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "ICP algorithm completed in " << duration.count() << " milliseconds." << std::endl;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "Lidar_node_client4");
    ros::NodeHandle n;

    // 起動時にファイルから点群を読み込む
    target = readPointsFromFile("/home/ryosuke/catkin_ws/src/research/scan.txt");
    


    gnuplot_pipe = popen("gnuplot -persist", "w");
    if (!gnuplot_pipe) {
        ROS_ERROR("Could not open pipe to gnuplot");
        return 1;
    }

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    ros::spin();

    pclose(gnuplot_pipe);
    return 0;
}
