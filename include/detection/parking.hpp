#pragma once
#include "common.hpp"
#include "detection.hpp"
#include "recognition/tracking.hpp"

class Parking {
public:
  /**
   * @brief 停车步骤
   *
   */
  enum ParkStep {
    none = 0, // 未知状态
    enable,   // 停车场使能
    turning,  // 入库转向
    stop,     // 停车
    trackout  // 出库
  };

  ParkStep step = ParkStep::none; // 停车步骤
  bool parkLeft = false;

  Parking(int turn, int carstop) : turningTIme(turn), stopTime(carstop) {}

  bool process(Tracking &track, cv::Mat &image,
               std::vector<PredictResult> predict) {
    counterSession++;
    if (step != ParkStep::none && counterSession > 500) // 超时退出
    {
      counterRec = 0;
      counterSession = 0;
      step = ParkStep::none; // 退出状态.
      startTurning = false;  // 恢复状态
      garageFirst = true;    // 进入一号车库
      lineY = 0;             // 直线高度
      ptA = cv::Point(0, 0); // 清空线段的两个端点
      ptB = cv::Point(0, 0);
      spdlog::info("[parking] 超时退出停车场");
    }
    switch (step) {
    case ParkStep::none: // AI未识别
    {
      for (size_t i = 0; i < predict.size(); i++) {
        if ((predict[i].type == LABEL_BATTERY) && predict[i].score > 0.4) {
          counterRec++;
          if (predict[i].x < COLSIMAGE / 2) {
            parkLeft = true;
          } else {
            parkLeft = false;
          }
          break;
        }
      }
      if (counterRec) // 检测到一帧后开始连续监测AI标志是否满足条件
      {
        if (counterRec >= 3 && counterSession < 12) {
          counterRec = 0;
          counterSession = 0;
          step = ParkStep::enable; // 检测到停车场标志
          spdlog::info("[parking] 进入停车场");
          return true;
        } else if (counterSession >= 8) {
          counterRec = 0;
          counterSession = 0;
        }
      }
      return false;
      break;
    }
    case ParkStep::enable: // 停车场使能
    {
      carY = ROWSIMAGE;     // 充电车高度
      batteryY = ROWSIMAGE; // 充电站标识高度

      for (size_t i = 0; i < predict.size(); i++) {
        if (predict[i].type == LABEL_CAR && predict[i].score > 0.6) {
          carY = (predict[i].y + predict[i].height) / 2; // 计算智能车的中心高度
        } else if ((predict[i].type == LABEL_BATTERY) &&
                   predict[i].score > 0.6) {
          batteryY = predict[i].y + predict[i].height; // 标识牌底部高度
        }
      }

      if (1) {
        garageFirst = true; // 进入一号车库
        // lineY = std::min(midY1, midY2); // 获取距离最远的线控制车入库
        step = ParkStep::turning; // 开始入库
        counterSession = 0;
        spdlog::info("[parking] 1号车库");
      } else if (0) {
        garageFirst = false; // 进入二号车库
        // lineY = std::min(midY1, midY2); // 获取距离最远的线控制车入库
        step = ParkStep::turning; // 开始入库
        counterSession = 0;
        spdlog::info("[parking] 2号车库");
      } else {
        counterSession = 0;
        step = ParkStep::turning; // 开始入库
        // lineY = std::min(midY1, midY2); // 获取距离最远的线控制车入库
        spdlog::info("[parking] 1号车库");
      }
    }
    case ParkStep::turning: // 入库转向
    {
      // 右侧车库 从中间开始找赛道边缘的跳变点 作为最近的第一条横线中的点
      for (int i = 10; i < track.pointsEdgeRight.size(); i++) {
        // 跳变点增加约束，这里去了平均值，将各个跳变点约束在相应直线附近
        if (track.pointsEdgeRight[i].y - track.pointsEdgeRight[i - 1].y > 20) {
          // track.pointsEdgeRight[i-2].y - track.pointsEdgeRight[i+2].y > 20) {
          splitpointdown[1].x = track.pointsEdgeRight[i - 5].x;
          int sum = 0;
          for (int k = 0; k < 5; k++) {
            sum += track.pointsEdgeRight[i - k].y;
          }
          splitpointdown[1].y = sum / 5;

          splitpointdown[0].x = track.pointsEdgeRight[i + 5].x;
          sum = 0;
          for (int k = 0; k < 5; k++) {
            sum += track.pointsEdgeRight[i + k].y;
          }
          splitpointdown[0].y = sum / 5;
        }
        if (track.pointsEdgeRight[i].y - track.pointsEdgeRight[i - 1].y < -20) {
          splitpointup[0].x = track.pointsEdgeRight[i].x;
          int sum = 0;
          for (int k = 0; k < 5; k++) {
            sum += track.pointsEdgeRight[i + k].y;
          }
          splitpointup[0].y = sum / 6;

          splitpointup[1].x = track.pointsEdgeRight[i - 5].x;
          sum = 0;
          for (int k = 0; k < 5; k++) {
            sum += track.pointsEdgeRight[i - k].y;
          }
          splitpointup[1].y = sum / 5;
          splitpointfound = true;
          break;
        }
      }
      // 控制转弯时机
      if (splitpointfound) {
        if (!startTurning) {
          counterSession = 0;
          startTurning = true; // 已经开始转弯
        }
        spdlog::info("[parking] 控制转弯");
        // 当两个补线点都在右侧
        if (splitpointdown[1].y > ((COLSIMAGE / 2) - 80) &&
            splitpointup[0].y > ((COLSIMAGE / 2) - 80)) {
          // 入库补线
          track.pointsEdgeLeft.clear();
          track.pointsEdgeRight.clear();
          POINT start = POINT(ROWSIMAGE - 31, 30);
          POINT end = POINT((splitpointdown[1].x + splitpointup[0].x) / 2,
                            (splitpointdown[1].y + splitpointup[0].y) / 2);
          POINT middle =
              POINT((start.x + end.x) * 0.5, (start.y + end.y) * 0.1);
          std::vector<POINT> input = {start, middle, end};
          track.pointsEdgeLeft = Bezier(0.02, input);

          std::vector<POINT> lineWithSplitPoint =
              points2line(end,
                          POINT((splitpointdown[0].x + splitpointup[1].x) / 2,
                                (splitpointdown[0].y + splitpointup[1].y) / 2),
                          5);
          track.pointsEdgeLeft.insert(track.pointsEdgeLeft.end(),
                                      lineWithSplitPoint.begin(),
                                      lineWithSplitPoint.end()); // 补直线
        }
        // 入库轨迹
        pathsEdgeLeft.push_back(track.pointsEdgeLeft);
        pathsEdgeRight.push_back(track.pointsEdgeRight);
      }
      if (counterSession > turningTIme && startTurning) // 开始停车状态
      {
        spdlog::info("[parking] 开始停车");
        step = ParkStep::stop; // 开始停车
      }
      break;
    }
    case ParkStep::stop: // 停车
    {
      if (counterSession > stopTime) {
        step = ParkStep::trackout; // 开始倒车
        spdlog::info("停车切倒车");
      }
      break;
    }
    case ParkStep::trackout: // 出库
    {
      spdlog::info("[parking] 开始倒车");
      if (pathsEdgeLeft.empty() || pathsEdgeRight.empty()) {
        counterRec = 0;
        counterSession = 0;
        step = ParkStep::none; // 退出状态.
        startTurning = false;  // 恢复状态
        garageFirst = true;    // 进入一号车库
        lineY = 0;             // 直线高度
        ptA = cv::Point(0, 0); // 清空线段的两个端点
        ptB = cv::Point(0, 0);
        spdlog::info("[parking] 退出停车场");
      }
      track.pointsEdgeLeft = pathsEdgeLeft[pathsEdgeLeft.size() - 1];
      track.pointsEdgeRight = pathsEdgeRight[pathsEdgeRight.size() - 1];
      pathsEdgeLeft.pop_back();
      pathsEdgeRight.pop_back();

      if (counterSession > 40 &&
          (pathsEdgeLeft.size() < 1 || pathsEdgeRight.size() < 1)) {
        counterRec = 0;
        counterSession = 0;
        step = ParkStep::none; // 退出状态.
        startTurning = false;  // 恢复状态
        garageFirst = true;    // 进入一号车库
        lineY = 0;             // 直线高度
        ptA = cv::Point(0, 0); // 清空线段的两个端点
        ptB = cv::Point(0, 0);
        pathsEdgeRight.clear();
        pathsEdgeLeft.clear();
        spdlog::info("[parking] 已退出停车场");
      }
      break;
    }
    }

    return true;
  }

  /**
   * @brief 识别结果图像绘制
   *
   */
  void drawImage(Tracking track, cv::Mat &image) {
    cv::circle(image, cv::Point(splitpointdown[0].y, splitpointdown[0].x), 3,
               cv::Scalar(0, 255, 0), -1);
    cv::circle(image, cv::Point(splitpointup[1].y, splitpointup[0].x), 3,
               cv::Scalar(0, 255, 0), -1);
    for (auto &i : horizontalLines) {
      cv::line(image, cv::Point(i[0], i[1]), cv::Point(i[2], i[3]),
               cv::Scalar(0, 0, 255), 1);
    }

    if (step != ParkStep::none)
      putText(image, "[1] BATTERY - ENABLE", cv::Point(10, 10),
              cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
  }

private:
  std::vector<cv::Vec4i> lines;
  std::vector<cv::Vec4i> horizontalLines;
  uint16_t counterSession = 0; // 图像场次计数器
  uint16_t counterRec = 0;     // 加油站标志检测计数器
  bool garageFirst = true;     // 进入一号车库
  int lineY = 0;               // 直线高度
  bool startTurning = false;   // 开始转弯
  int carY = ROWSIMAGE;        // 充电车高度
  int batteryY = ROWSIMAGE;    // 充电站标识高度
  std::vector<std::vector<POINT>> pathsEdgeLeft; // 记录入库路径
  std::vector<std::vector<POINT>> pathsEdgeRight;
  cv::Point ptA = cv::Point(0, 0); // 记录线段的两个端点
  cv::Point ptB = cv::Point(0, 0);
  // 跳变点 0： 跳变点对上方点 1：跳变点对下方点
  POINT splitpointdown[2] = {{0, 0}, {0, 0}};
  POINT splitpointup[2] = {{0, 0}, {0, 0}};
  bool splitpointfound = false;

  int turningTIme = 21; // 转弯时间 21帧
  int stopTime = 100;   // 停车时间 40帧
  // float swerveTime = 0.2; // 转向时机 0.2 （转弯线出现在屏幕上方0.2处）
};