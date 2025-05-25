#pragma once
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <functional>
#include <future>
#include <iostream>
#include <libserial/SerialPort.h>
#include <memory>
#include <mutex>
#include <numeric>
#include <onnx/onnxruntime_cxx_api.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <ppnc/predictor_api.h>
#include <queue>
#include <spdlog/spdlog.h>
#include <string>
#include <sys/time.h>
#include <thread>
#include <unordered_map>
#include <vector>

#define COLSIMAGE 320    // 图像的列数
#define ROWSIMAGE 240    // 图像的行数
#define COLSIMAGEIPM 320 // IPM图像的列数
#define ROWSIMAGEIPM 240 // IPM图像的行数
#define PWMSERVOMAX 1900 // 舵机PWM最大值（左）1840
#define PWMSERVOMID 1500 // 舵机PWM中值 1520
#define PWMSERVOMIN 1100 // 舵机PWM最小值（右）1200

#define LABEL_BATTERY 0    // AI标签：充电站
#define LABEL_BLOCK 1      // AI标签：障碍物
#define LABEL_BRIDGE 2     // AI标签：坡道
#define LABEL_BURGER 3     // AI标签：汉堡
#define LABEL_CAR 4        // AI标签：道具车
#define LABEL_COMPANY 5    // AI标签：公司
#define LABEL_CONE 6       // AI标签：锥桶
#define LABEL_CROSSWALK 7  // AI标签：斑马线
#define LABEL_PEDESTRIAN 8 // AI标签：行人
#define LABEL_SCHOOL 9     // AI标签：学校

#define using_kernel_num 7
#define using_resample_dist 0.05
#define PI 3.141593
#define pixel_per_meter 133

/**
 * @brief 场景类型（路况）
 *
 */
enum Scene {
  NormalScene = 0, // 基础赛道
  CrossScene,      // 十字道路
  RingScene,       // 环岛道路
  BridgeScene,     // 坡道区
  ObstacleScene,   // 障碍区
  CateringScene,   // 快餐店
  LaybyScene,      // 临时停车区
  ParkingScene,    // 停车区
  StopScene        // 停车（结束）
};

/**
 * @brief Get the Scene object
 *
 * @param scene
 * @return string
 */
std::string getScene(Scene scene) {
  switch (scene) {
  case Scene::NormalScene:
    return "Normal";
  case Scene::CrossScene:
    return "Crossroad";
  case Scene::RingScene:
    return "Ring";
  case Scene::BridgeScene:
    return "Bridge";
  case Scene::ObstacleScene:
    return "Obstacle";
  case Scene::CateringScene:
    return "Catering";
  case Scene::LaybyScene:
    return "Layby";
  case Scene::ParkingScene:
    return "Parking";
  case Scene::StopScene:
    return "Stop";
  default:
    return "Error";
  }
}

/**
 * @brief 构建二维坐标
 *
 */
struct POINT {
  int x = 0;
  int y = 0;
  float slope = 0.0f;

  POINT(){};
  POINT(int x, int y) : x(x), y(y){};
};

/**
 * @brief 角度
 *
 */
struct Angle {
  float angle[500];
  int real_size;
};

/**
 * @brief 存储图像至本地
 *
 * @param image 需要存储的图像
 */
void savePicture(cv::Mat &image) {
  // 存图
  std::string name = ".jpg";
  static int counter = 0;
  counter++;
  std::string imgPath = "./samples/";
  name = imgPath + std::to_string(counter) + ".jpg";
  cv::imwrite(name, image);
}

//--------------------------------------------------[公共方法]----------------------------------------------------
/**
 * @brief int集合平均值计算
 *
 * @param arr 输入数据集合
 * @return double
 */
double average(std::vector<int> vec) {
  if (vec.size() < 1)
    return -1;

  double sum = 0;
  for (size_t i = 0; i < vec.size(); i++) {
    sum += vec[i];
  }

  return (double)sum / vec.size();
}

/**
 * @brief int集合数据方差计算
 *
 * @param vec Int集合
 * @return double
 */
double sigma(std::vector<int> vec) {
  if (vec.size() < 1)
    return 0;

  double aver = average(vec); // 集合平均值
  double sigma = 0;
  for (size_t i = 0; i < vec.size(); i++) {
    sigma += (vec[i] - aver) * (vec[i] - aver);
  }
  sigma /= (double)vec.size();
  return sigma;
}

/**
 * @brief 赛道点集的方差计算
 *
 * @param vec
 * @return double
 */
double sigma(std::vector<POINT> vec) {
  if (vec.size() < 1)
    return 0;

  double sum = 0;
  for (size_t i = 0; i < vec.size(); i++) {
    sum += vec[i].y;
  }
  double aver = (double)sum / vec.size(); // 集合平均值

  double sigma = 0;
  for (size_t i = 0; i < vec.size(); i++) {
    sigma += (vec[i].y - aver) * (vec[i].y - aver);
  }
  sigma /= (double)vec.size();
  return sigma;
}

/**
 * @brief 阶乘计算
 *
 * @param x
 * @return int
 */
int factorial(int x) {
  int f = 1;
  for (int i = 1; i <= x; i++) {
    f *= i;
  }
  return f;
}

// 由两点生成离散直线
std::vector<POINT> points2line(POINT a, POINT b, int dist) {
  std::vector<POINT> result;
  double dx = b.x - a.x;
  double dy = b.y - a.y;
  double dn = sqrt(dx * dx + dy * dy);

  int steps = static_cast<int>(std::ceil(dn / dist));

  for (int i = 0; i <= steps; i++) {
    POINT p;
    p.x = a.x + dx * (i / static_cast<double>(steps));
    p.y = a.y + dy * (i / static_cast<double>(steps));
    result.push_back(p);
  }
  return result;
}

/**
 * @brief 贝塞尔曲线
 *
 * @param dt
 * @param input
 * @return std::vector<POINT>
 */
std::vector<POINT> Bezier(double dt, std::vector<POINT> input) {
  std::vector<POINT> output;

  double t = 0;
  while (t <= 1) {
    POINT p;
    double sumX = 0.0;
    double sumY = 0.0;
    int i = 0;
    int n = input.size() - 1;
    while (i <= n) {
      double k = factorial(n) / (factorial(i) * factorial(n - i)) * pow(t, i) *
                 pow(1 - t, n - i);
      sumX += k * input[i].x;
      sumY += k * input[i].y;
      i++;
    }
    p.x = sumX;
    p.y = sumY;
    output.push_back(p);
    t += dt;
  }
  return output;
}

auto formatDouble2String(double val, int fixed) {
  auto str = std::to_string(val);
  return str.substr(0, str.find(".") + fixed + 1);
}

/**
 * @brief 点到直线的距离计算
 *
 * @param a 直线的起点
 * @param b 直线的终点
 * @param p 目标点
 * @return double
 */
double distanceForPoint2Line(POINT a, POINT b, POINT p) {
  double ab_distance =
      sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
  double ap_distance =
      sqrt((a.x - p.x) * (a.x - p.x) + (a.y - p.y) * (a.y - p.y));
  double bp_distance =
      sqrt((p.x - b.x) * (p.x - b.x) + (p.y - b.y) * (p.y - b.y));

  double half = (ab_distance + ap_distance + bp_distance) / 2;
  double area = sqrt(half * (half - ab_distance) * (half - ap_distance) *
                     (half - bp_distance));

  return (2 * area / ab_distance);
}

/**
 * @brief 两点之间的距离
 *
 * @param a
 * @param b
 * @return double
 */
double distanceForPoints(POINT a, POINT b) {
  return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

std::vector<cv::Point2f> convertPointsToCvPoints(std::vector<POINT> &edge) {
  std::vector<cv::Point2f> cvPoints;
  for (int i = 0; i < edge.size(); i++) {
    cvPoints.emplace_back(cv::Point2f(edge[i].y, edge[i].x));
  }
  return cvPoints;
} // POINT to Point2f

std::vector<POINT>
convertCvPointsToPoints(const std::vector<cv::Point2f> &cvPoints) {
  std::vector<POINT> edge;
  for (int i = 0; i < cvPoints.size(); ++i) {
    POINT p(0, 0);
    p.y = static_cast<int>(cvPoints[i].x);
    p.x = static_cast<int>(cvPoints[i].y);
    edge.push_back(p);
  }
  return edge;
} // Point2f to POINT

cv::Mat auto_init_ipm_mat() {
  int offsety = 20;
  std::vector<cv::Point2f> srcPoints = {
      cv::Point2f(93, 76), cv::Point2f(214, 75), cv::Point2f(52, 153),
      cv::Point2f(259, 146)};
  std::vector<cv::Point2f> dstPoints = {
      cv::Point2f(130, 120 + offsety), cv::Point2f(190, 120 + offsety),
      cv::Point2f(130, 180 + offsety), cv::Point2f(190, 180 + offsety)};
  cv::Mat M = cv::getPerspectiveTransform(
      srcPoints, dstPoints);

  return M;
}

int clip(int x, int low, int up) { return x > up ? up : x < low ? low : x; }

std::vector<POINT> blur_points(std::vector<POINT> edge_input, int kernel) {
  std::vector<POINT> output_edge;
  // output_edge. = edge_input.real_size;

  int half = kernel / 2;
  for (int i = 0; i < edge_input.size(); i++) {
    POINT output(0, 0);
    for (int j = -half; j <= half; j++) {
      output.y += edge_input[clip(i + j, 0, edge_input.size() - 1)].y *
                  (half + 1 - abs(j));
      output.x += edge_input[clip(i + j, 0, edge_input.size() - 1)].x *
                  (half + 1 - abs(j));
    }
    output.y /= (2 * half + 2) * (half + 1) / 2;
    output.x /= (2 * half + 2) * (half + 1) / 2;
    output_edge.push_back(output);
  }

  return output_edge;
}

std::vector<POINT> resample_points(std::vector<POINT> input_edge, float dist) {
  std::vector<POINT> output_edge;

  int remain = 0; // len = 0;
  for (int i = 0; i < input_edge.size() - 1; i++) {
    float x0 = input_edge[i].y;
    float y0 = input_edge[i].x;
    float dx = input_edge[i + 1].y - x0;
    float dy = input_edge[i + 1].x - y0;
    float dn = sqrt(dx * dx + dy * dy);
    dx /= dn;
    dy /= dn;

    while (remain < dn) {
      POINT p(0, 0);
      x0 += dx * remain;
      p.y = x0;
      y0 += dy * remain;
      p.x = y0;
      output_edge.push_back(p);

      // len++;
      dn -= remain;
      remain = dist;
    }
    remain -= dn;
  }
  return output_edge;
}

Angle get_angle(std::vector<POINT> input_edge, int dist) {
  Angle angle_output;

  angle_output.real_size = input_edge.size();
  for (int i = 0; i < input_edge.size(); i++) {
    if (i <= 0 || i >= input_edge.size() - 1) {
      angle_output.angle[i] = 0;
      continue;
    }
    float dx1 = input_edge[i].y -
                input_edge[clip(i - dist, 0, input_edge.size() - 1)].y;
    float dy1 = input_edge[i].x -
                input_edge[clip(i - dist, 0, input_edge.size() - 1)].x;
    float dn1 = sqrtf(dx1 * dx1 + dy1 * dy1);
    float dx2 = input_edge[clip(i + dist, 0, input_edge.size() - 1)].y -
                input_edge[i].y;
    float dy2 = input_edge[clip(i + dist, 0, input_edge.size() - 1)].x -
                input_edge[i].x;
    float dn2 = sqrtf(dx2 * dx2 + dy2 * dy2);
    float c1 = dx1 / dn1;
    float s1 = dy1 / dn1;
    float c2 = dx2 / dn2;
    float s2 = dy2 / dn2;
    angle_output.angle[i] = atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);
    angle_output.angle[i] = 180 / PI * angle_output.angle[i];
  }

  return angle_output;
}

// float angleBetweenPoints(const POINT& p1, const POINT& p2, const POINT& p3) {
//     float dx1 = p2.x - p1.x;
//     float dy1 = p2.y - p1.y;
//     float dx2 = p3.x - p2.x;
//     float dy2 = p3.y - p2.y;
//     float dot = dx1 * dx2 + dy1 * dy2;
//     float cross = dx1 * dy2 - dy1 * dx2;
//     float angle = atan2f(cross, dot);
//     return fabs(angle);
// }

std::vector<POINT> track_leftline(std::vector<POINT> edge_in, int approx_num,
                                  float dist) {
  std::vector<POINT> edge_out;
  for (int i = 0; i < edge_in.size(); i++) {
    POINT p(0, 0);
    float dx = edge_in[clip(i + approx_num, 0, edge_in.size() - 1)].y -
               edge_in[clip(i - approx_num, 0, edge_in.size() - 1)].y;
    float dy = edge_in[clip(i + approx_num, 0, edge_in.size() - 1)].x -
               edge_in[clip(i - approx_num, 0, edge_in.size() - 1)].x;

    float dn = sqrt(dx * dx + dy * dy);
    dx /= dn;
    dy /= dn;

    p.y = edge_in[i].y - dy * dist;
    p.x = edge_in[i].x + dx * dist;
    edge_out.push_back(p);
  }

  return edge_out;
}

std::vector<POINT> track_rightline(std::vector<POINT> edge_in, int approx_num,
                                   float dist) {
  std::vector<POINT> edge_out;
  for (int i = 0; i < edge_in.size(); i++) {
    POINT p(0, 0);
    float dx = edge_in[clip(i + approx_num, 0, edge_in.size() - 1)].y -
               edge_in[clip(i - approx_num, 0, edge_in.size() - 1)].y;
    float dy = edge_in[clip(i + approx_num, 0, edge_in.size() - 1)].x -
               edge_in[clip(i - approx_num, 0, edge_in.size() - 1)].x;
    float dn = sqrt(dx * dx + dy * dy);
    dx /= dn;
    dy /= dn;
    p.y = edge_in[i].y + dy * dist;
    p.x = edge_in[i].x - dx * dist;
    edge_out.push_back(p);
  }

  return edge_out;
}

cv::Mat draw_boundary_ipm(std::vector<POINT> left_edge,
                          std::vector<POINT> right_edge, cv::Mat invMat) {
  // cout << "左边线:" << left_edge.real_size;
  left_edge = blur_points(left_edge, using_kernel_num);
  right_edge = blur_points(right_edge, using_kernel_num);
  left_edge = resample_points(left_edge, using_resample_dist);
  right_edge = resample_points(right_edge, using_resample_dist);
  std::vector<cv::Point2f> pointsToTransform_left =
                               convertPointsToCvPoints(left_edge),
                           pointsToTransform_right =
                               convertPointsToCvPoints(right_edge),
                           transformedPoints;

  cv::Size imageSize(320, 240);
  cv::Mat only_boundary = cv::Mat::zeros(imageSize, CV_8UC1);
  // invcv::Mat_auto=invcv::Mat_auto.inv();
  cv::perspectiveTransform(pointsToTransform_left, transformedPoints, invMat);
  for (size_t i = 0; i < transformedPoints.size(); i++) {
    cv::circle(only_boundary, transformedPoints[i], 0,
               cv::Scalar(255, 255, 255), 2);
  }
  left_edge = convertCvPointsToPoints(transformedPoints);

  cv::perspectiveTransform(pointsToTransform_right, transformedPoints, invMat);
  for (size_t i = 0; i < transformedPoints.size(); i++) {
    cv::circle(only_boundary, transformedPoints[i], 0,
               cv::Scalar(255, 255, 255), 2);
  }

  std::vector<POINT> center_line = track_leftline(left_edge, 5, 27);
  Angle angle_pts = get_angle(center_line, 10);
  std::vector<cv::Point2f> pointsToTransform_center =
      convertPointsToCvPoints(center_line);
  // perspectiveTransform(pointsToTransform_center, transformedPoints,
  // invcv::Mat);
  for (size_t i = 0; i < pointsToTransform_center.size(); i++) {
    cv::circle(only_boundary, pointsToTransform_center[i], 0,
               cv::Scalar(255, 255, 255), 2);
    // imshow("11", only_boundary);
    // cout << angle_pts.angle[i] << endl;
    // waitKey(500);
  }
  // imshow("仅边界", only_boundary);

  return only_boundary;
}