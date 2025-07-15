#pragma once
#include "common.hpp"
#include "detection.hpp"
#include "recognition/tracking.hpp"

class Catering {
public:
  Catering(int travelTime = 10, int turningTime = 50, int stopTime = 100)
      : travelTime(travelTime), turningTime(turningTime), stopTime(stopTime) {}
  bool stopEnable = false; // 停车使能标志
  bool noRing = false;     // 用来区分环岛路段
  bool burgerLeft = true;  // 汉堡在左侧

  bool process(Tracking &track, cv::Mat &image,
               std::vector<PredictResult> predict) {
    if (cateringEnable) {
      if (!stopEnable && turning) {
        for (size_t i = 0; i < predict.size(); i++) {
          if (predict[i].type == LABEL_BURGER && predict[i].score > 0.6) {
            burgerFound = true; // 发现汉堡标志
            break;
          } else {
            burgerFound = false; // 未发现汉堡标志
          }
        }

      }

      counterSession++;
      // 超时退出区域
      if (counterSession > (turningTime + travelTime + stopTime)) {
        counterRec = 0;
        counterSession = 0;
        cateringEnable = false;
        turning = true;                                       // 转向标志
        stopEnable = false;                                   // 停车使能
        noRing = false;                                       // 区分环岛
      } else if (counterSession > (turningTime + travelTime)) // 驶入餐饮区
        stopEnable = true;                                    // 停车使能
      else if (counterSession > turningTime)                  // 进入岔路
        turning = false; // 关闭转向标志
      // if ((burgerFound && burgerLeft && track.stdevRight > 30) ||
      //     (burgerFound && !burgerLeft && track.stdevLeft > 30)) {
      if (burgerFound) {
        return true;
      } else {
        return false;
      }
 
    } else {
      for (size_t i = 0; i < predict.size(); i++) {
        if (predict[i].type == LABEL_BURGER && predict[i].score > 0.6 &&
            (predict[i].y + predict[i].height) > ROWSIMAGE * 0.3) {
          counterRec++;
          noRing = true;
          burgerFound = true; // 发现汉堡标志
          if (predict[i].x < COLSIMAGE / 2) {
            burgerLeft = true;
            spdlog::info("found");
          } else {
            burgerLeft = false;
          }
          break;
        } else {
          burgerFound = false; // 未发现汉堡标志
        }
      }

      if (counterRec) {
        counterSession++;
        if (counterRec >= 3 && counterSession < 8) {
          counterRec = 0;
          counterSession = 0;
          cateringEnable = true; // 检测到汉堡标志
          if ((burgerFound && burgerLeft && track.stdevRight > 100) ||
              (burgerFound && !burgerLeft && track.stdevLeft > 100)) {
            return true;
          } else {
            return false;
          }

        } else if (counterSession >= 8) {
          counterRec = 0;
          counterSession = 0;
        }
      }

      return false;
    }
  }

  /**
   * @brief 识别结果图像绘制
   *
   */
  void drawImage(Tracking track, cv::Mat &image) {
    // 赛道边缘
    for (size_t i = 0; i < track.pointsEdgeLeft.size(); i++) {
      circle(image,
             cv::Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 1,
             cv::Scalar(0, 255, 0), -1); // 绿色点
    }
    for (size_t i = 0; i < track.pointsEdgeRight.size(); i++) {
      circle(image,
             cv::Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x),
             1, cv::Scalar(0, 255, 255), -1); // 黄色点
    }

    if (cateringEnable)
      putText(image, "[1] Burger - ENABLE", cv::Point(10, 10),
              cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
  }

private:
  uint16_t counterSession = 0; // 图像场次计数器
  uint16_t counterRec = 0;     // 汉堡标志检测计数器
  bool cateringEnable = false; // 岔路区域使能标志
  bool turning = true;         // 转向标志
  bool burgerFound = false;    // 汉堡检测标志
  int turningTime = 50;        // 转弯时间 25帧
  int travelTime = 10; // 行驶时间 10帧 在斜线路段的行驶时间
  int stopTime = 100;  // 停车时间 25帧
};