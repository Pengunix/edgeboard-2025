#pragma once
#include "common.hpp"
#include "detection.hpp"
#include "recognition/tracking.hpp"

class Layby {
public:
  bool stopEnable = false; // 停车使能标志

  bool process(Tracking &track, cv::Mat &image, std::vector<PredictResult> predict) {
    if (laybyEnable) // 进入临时停车状态
    {
      cv::Mat edges;
      cv::Mat blurred;

      curtailTracking(track, leftEnable); // 缩减优化车道线（双车道→单车道）
      // 直线检测
      GaussianBlur(image, blurred, cv::Size(3, 3), 0); // 添加高斯模糊预处理
      Canny(blurred, edges, 30, 150, 3); // 调整Canny参数，使用3x3 Sobel算子

      // 霍夫变换检测直线
      std::vector<cv::Vec4i> lines;
      HoughLinesP(edges, lines, 1, // rho
                  CV_PI / 180,     // theta
                  25,              // threshold：降低阈值
                  40,              // minLineLength：减小最小线段长度
                  20);             // maxLineGap：增大间隙容忍度

      // 存储合并后的线段
      mergedLines.clear();

      // 对检测到的线段进行排序（按y坐标）
      sort(lines.begin(), lines.end(), [](const cv::Vec4i &a, const cv::Vec4i &b) {
        return (a[1] + a[3]) / 2 < (b[1] + b[3]) / 2;
      });

      // 找到左边界最左侧的点
      int leftMostX = COLSIMAGE;
      for (size_t i = 0; i < track.pointsEdgeLeft.size(); i++) {
        if (track.pointsEdgeLeft[i].x < leftMostX) {
          leftMostX = track.pointsEdgeLeft[i].x;
        }
      }

      // 合并距离相近的线段
      for (const cv::Vec4i &line : lines) {
        cv::Point pt1(line[0], line[1]);
        cv::Point pt2(line[2], line[3]);

        // 计算线段中点的y坐标
        int midY = (pt1.y + pt2.y) / 2;

        double slope =
            abs(static_cast<double>(pt2.y - pt1.y) / (pt2.x - pt1.x + 1e-5));

        if (slope > 0.5 || midY <= 30 || midY >= 200 || pt1.x < leftMostX ||
            pt2.x < leftMostX)
          continue; // 跳过非水平线;跳过切行图像

        bool shouldMerge = false;
        // 检查是否有距离相近的已存在线段
        for (cv::Vec4i &merged : mergedLines) {
          int y1 = (merged[1] + merged[3]) / 2; // 已有线段的平均y值
          int y2 = (pt1.y + pt2.y) / 2;         // 当前线段的平均y值

          if (abs(y1 - y2) < 20) // 如果y方向距离小于20像素
          {
            // 合并并延长线段
            int minX = std::min(std::min(merged[0], merged[2]), std::min(line[0], line[2]));
            int maxX = std::max(std::max(merged[0], merged[2]), std::max(line[0], line[2]));
            int avgY = (y1 + y2) / 2; // 使用平均y值

            // 更新为合并后的线段
            merged[0] = minX;
            merged[1] = avgY;
            merged[2] = maxX;
            merged[3] = avgY;

            shouldMerge = true;
            break;
          }
        }

        if (!shouldMerge) {
          mergedLines.push_back(line);
        }
      }

      counterSession++;
      if (counterSession > stopTime) // 结束临时停车状态
      {
        counterRec = 0;
        counterSession = 0;
        laybyEnable = false;
        stopEnable = false;               // 停车使能
        searchingLine = false;            // 搜索直线标志
      } else if (mergedLines.size() == 2) // 检测到两条直线
      {
        searchingLine = true; // 搜索直线标志
        spdlog::info("[layby] 检测到两条线");
      } else if (searchingLine && mergedLines.size() == 1 &&
                 mergedLines[0][1] > moment && counterRec == 0) {
        spdlog::info("[layby] 开始临时停车");
        stopEnable = true; // 停车使能
        searchingLine = false;
      }
      return true;
    }

    else // 检测标志
    {
      for (size_t i = 0; i < predict.size(); i++) {
        if (((predict[i].type == LABEL_SCHOOL ||
              predict[i].type == LABEL_COMPANY) &&
             predict[i].score > 0.6) &&
            (predict[i].y + predict[i].height) > ROWSIMAGE * 0.1) {
          counterRec++;
          if (predict[i].x < COLSIMAGE / 2) // 标识牌在左侧
            leftEnable = true;
          else
            leftEnable = false;
          break;
        }
      }

      if (counterRec) {
        counterSession++;
        if (counterRec >= 4 && counterSession < 8) {
          counterRec = 0;
          counterSession = 0;
          laybyEnable = true; // 检测到标识牌子
          return true;
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
      circle(image, cv::Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x),
             1, cv::Scalar(0, 255, 0), -1); // 绿色点
    }
    for (size_t i = 0; i < track.pointsEdgeRight.size(); i++) {
      circle(image,
             cv::Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1,
             cv::Scalar(0, 255, 255), -1); // 黄色点
    }

    // 绘制合并后的结果
    for (const cv::Vec4i &line : mergedLines) {
      cv::line(image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]),
               cv::Scalar(0, 0, 255), 2);
    }

    if (laybyEnable)
      putText(image, "[1] Layby - ENABLE", cv::Point(COLSIMAGE / 2 - 30, 10),
              cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
  }

  /**
   * @brief 缩减优化车道线（双车道→单车道）
   *
   * @param track
   * @param left
   */
  void curtailTracking(Tracking &track, bool left) {
    if (left) // 向左侧缩进
    {
      if (track.pointsEdgeRight.size() > track.pointsEdgeLeft.size())
        track.pointsEdgeRight.resize(track.pointsEdgeLeft.size());

      for (size_t i = 0; i < track.pointsEdgeRight.size(); i++) {
        track.pointsEdgeRight[i].y =
            (track.pointsEdgeRight[i].y + track.pointsEdgeLeft[i].y) / 2;
      }
    } else // 向右侧缩进
    {
      if (track.pointsEdgeRight.size() < track.pointsEdgeLeft.size())
        track.pointsEdgeLeft.resize(track.pointsEdgeRight.size());

      for (size_t i = 0; i < track.pointsEdgeLeft.size(); i++) {
        track.pointsEdgeLeft[i].y =
            (track.pointsEdgeRight[i].y + track.pointsEdgeLeft[i].y) / 2;
      }
    }
  }

private:
  uint16_t counterSession = 0; // 图像场次计数器
  uint16_t counterRec = 0;     // 标识牌检测计数器
  bool laybyEnable = false;    // 临时停车区域使能标志
  bool leftEnable = true;      // 标识牌在左侧
  bool searchingLine = false;  // 搜索直线标志
  std::vector<cv::Vec4i> mergedLines;   // 合并后的线段用于绘制
  int moment = 110; // 停车时机，屏幕上方的像素值，值越大越越晚停车
  int stopTime = 40; // 停车时间 40帧
};