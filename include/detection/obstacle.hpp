#include "common.hpp"
#include "detection.hpp"

class Obstacle {
public:
  enum Step {
    None = 0,
    Start, // 识别到爆炸物
    Inside,
    End
  };

  Step step = Step::None;
  PredictResult resultObs; // 避障目标锥桶
  cv::Rect pedestrain_rect;
  bool findFlag = false;
  bool enable = false; // 场景检测使能标志
  bool left = true;
  bool pedstrain = false; // 行人检测标志
  int block_stage = 1;
  int first = 0;
  bool last_left = true;
  int passblock = 0;
  int counterExit = 0;
  int colorcnt[4] = {0};
  // TODO(me) merge pedstrain mode to full cone mode
  bool process(cv::Mat imageRGB, Tracking &track,
               std::shared_ptr<Detection> detect, uint64_t frameTime,
               float speed, int DisLeaving, const std::vector<int> &RoadWidth,
               float UpScale, float block_scale, float distance_block,
               int ObstacleMode) {

    if (step == Step::None) {
      pedstrain = false;
      // for (size_t i = 0; i < detect->results.size(); i++) {
      //   if ((detect->results[i].type == LABEL_PEDESTRIAN) &&
      //       detect->results[i].score > 0.6) {
      //     first = 0;
      //     step = Step::Start;
      //     counterExit = 0;
      //     pedstrain = true;
      //   }
      // }
      if (hsvSearch(imageRGB, track)) {
        first = 0;
        step = Step::Start;
        counterExit = 0;
      }
    }

    if (step == Step::Start) {
      // spdlog::info("Obstacle start");
      findFlag = false;
      // 锥桶检测
      cv::Mat hsv;
      cv::cvtColor(imageRGB, hsv, cv::COLOR_BGR2HSV);
      // 定义黄色的HSV范围
      cv::Scalar lowerYellow(15, 100, 60);
      cv::Scalar upperYellow(35, 255, 255);
      // 红色
      cv::Scalar lowerRed(18, 135, 121);
      cv::Scalar upperRed(180, 255, 255);

      // 紫色
      cv::Scalar lowerPurple(117, 28, 51);
      cv::Scalar upperPurple(180, 163, 196);

      cv::Mat Mask = cv::Mat::zeros(imageRGB.size(), CV_8UC1);
      cv::Mat tmpMask = cv::Mat::zeros(imageRGB.size(), CV_8UC1);
      inRange(hsv, lowerYellow, upperYellow, tmpMask);
      cv::bitwise_or(Mask, tmpMask, Mask);
      inRange(hsv, lowerRed, upperRed, tmpMask);
      cv::bitwise_or(Mask, tmpMask, Mask);
      inRange(hsv, lowerPurple, upperPurple, tmpMask);
      cv::bitwise_or(Mask, tmpMask, Mask);

      cv::Mat kernel =
          cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
      cv::morphologyEx(Mask, Mask, cv::MORPH_OPEN, kernel);

      std::vector<std::vector<cv::Point>> contours;
      findContours(Mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      // cv::Rect Pedestrain;
      // for (const auto &result : detect->results) {
      //   if (result.type == LABEL_PEDESTRIAN && result.score > 0.5) {
      //     Pedestrain =
      //         cv::Rect(result.x, result.y, result.width, result.height);
      //     pedstrain = true;
      //   }
      // }

      int x_Near = 0; // 行坐标
      if (!contours.empty()) {
        // 选取距离最近的锥桶
        cv::Rect Cone;
        for (const auto &contour : contours) {
          if (cv::contourArea(contour) < 20)
            continue; // 过滤小锥桶
          cv::Rect Tempcone = boundingRect(contour);
          if (Tempcone.y > ROWSIMAGE * UpScale &&
              Tempcone.y + Tempcone.height < ROWSIMAGE - DisLeaving &&
              Tempcone.y > x_Near) {
            int row =
                track.pointsEdgeLeft.size() - (Tempcone.y - track.rowCutUp);
            if (row < 0) // 太远不管
            {
              continue;
            }
            int disLeft =
                Tempcone.x + Tempcone.width -
                track.pointsEdgeLeft[row].y; // 障碍右侧到左边线的水平距离
            int disRight = track.pointsEdgeRight[row].y -
                           Tempcone.x; // 右边线到障碍左侧的水平距离
            if (disLeft >= 0 && disRight >= 0) {
              findFlag = true;
              Cone = Tempcone;
              x_Near = Tempcone.y;
            }
          }
        }
        // 锥桶与行人比较，选距离最近的赋值给Cone
        // if (pedstrain) {
        //   if (Pedestrain.y > ROWSIMAGE * UpScale &&
        //       Pedestrain.y + Pedestrain.height < ROWSIMAGE - DisLeaving &&
        //       Pedestrain.y > x_Near) {
        //     if (!findFlag) {
        //       findFlag = true;
        //       Cone = Pedestrain;
        //       x_Near = Pedestrain.y;
        //     } else {
        //       int row =
        //           track.pointsEdgeLeft.size() - (Pedestrain.y -
        //           track.rowCutUp);
        //       if (row > 0) // 太远不管
        //       {
        //         int disLeft =
        //             Pedestrain.x + Pedestrain.width -
        //             track.pointsEdgeLeft[row].y; //
        //             行人右侧到左边线的水平距离
        //         int disRight = track.pointsEdgeRight[row].y -
        //                        Pedestrain.x; // 右边线到行人左侧的水平距离
        //         if (disLeft >= 0 && disRight >= 0 &&
        //             (Pedestrain.y + Pedestrain.height) <
        //                 (Cone.y + Cone.height)) {
        //           findFlag = true;
        //           Cone = Pedestrain;
        //           x_Near = Pedestrain.y;
        //         }
        //       }
        //     }
        //   }
        // }
        if (findFlag) {
          resultObs.type = LABEL_CONE;
          resultObs.x = Cone.x;
          resultObs.y = Cone.y;
          resultObs.height = Cone.height;
          resultObs.width = Cone.width;
        } else {
          counterExit++;
          if (counterExit > 50) {
            step = Step::None;
            return false;
          }
        }
      }
      if (ObstacleMode == 0) {
        if (findFlag) {
          // 障碍物方向判定（左/右）
          int row =
              track.pointsEdgeLeft.size() - (resultObs.y - track.rowCutUp);
          int disLeft =
              resultObs.x + resultObs.width -
              track.pointsEdgeLeft[row].y; // 障碍右侧到左边线的水平距离
          int disRight = track.pointsEdgeRight[row].y -
                         resultObs.x; // 右边线到障碍左侧的水平距离
          if (disLeft <= disRight)    //[1] 障碍物在赛道内， 且靠左
          {
            left = false;
          } else if (disLeft > disRight) //[2] 障碍物在赛道内，且靠右
          {
            left = true;
          }
          if (first == 0) {
            last_left = left;
            first = 1;
          } else if (first == 1 && last_left != left) {
            step = Step::Inside;
            first = 0;
          }
        }
        if (first == 1)
          curtailTracking(track, left, RoadWidth);

      } else if (ObstacleMode == 1) {
        if (findFlag)
          step = Step::Inside;
      }
    }

    if (step == Step::Inside) {
      spdlog::info("Obstacle inside");
      findFlag = false;
      // 锥桶检测
      cv::Mat hsv;
      cvtColor(imageRGB, hsv, cv::COLOR_BGR2HSV);
      // 定义黄色的HSV范围
      // 定义黄色的HSV范围
      cv::Scalar lowerYellow(15, 100, 60);
      cv::Scalar upperYellow(35, 255, 255);
      // 红色
      cv::Scalar lowerRed(18, 135, 121);
      cv::Scalar upperRed(180, 255, 255);

      // 紫色
      cv::Scalar lowerPurple(117, 28, 51);
      cv::Scalar upperPurple(180, 163, 196);

      cv::Mat Mask = cv::Mat::zeros(imageRGB.size(), CV_8UC1);
      cv::Mat tmpMask = cv::Mat::zeros(imageRGB.size(), CV_8UC1);
      inRange(hsv, lowerYellow, upperYellow, tmpMask);
      cv::bitwise_or(Mask, tmpMask, Mask);
      inRange(hsv, lowerRed, upperRed, tmpMask);
      cv::bitwise_or(Mask, tmpMask, Mask);
      inRange(hsv, lowerPurple, upperPurple, tmpMask);
      cv::bitwise_or(Mask, tmpMask, Mask);

      cv::Mat kernel =
          cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
      cv::morphologyEx(Mask, Mask, cv::MORPH_OPEN, kernel);

      std::vector<std::vector<cv::Point>> contours;
      findContours(Mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      int x_Near = 0;
      if (!contours.empty()) {
        // 选取距离最近的锥桶
        cv::Rect Cone;
        for (const auto &contour : contours) {
          if (cv::contourArea(contour) < 20)
            continue; // 过滤小锥桶
          cv::Rect Tempcone = boundingRect(contour);
          if (Tempcone.y > ROWSIMAGE * UpScale &&
              Tempcone.y + Tempcone.height < ROWSIMAGE - DisLeaving &&
              Tempcone.y > x_Near) {
            int row =
                track.pointsEdgeLeft.size() - (Tempcone.y - track.rowCutUp);
            if (row < 0) // 太远不管
            {
              continue;
            }
            int disLeft =
                Tempcone.x + Tempcone.width -
                track.pointsEdgeLeft[row].y; // 障碍右侧到左边线的水平距离
            int disRight = track.pointsEdgeRight[row].y -
                           Tempcone.x; // 右边线到障碍左侧的水平距离
            if (disLeft >= 0 && disRight >= 0) {
              findFlag = true;
              Cone = Tempcone;
              x_Near = Tempcone.y;
            }
          }
        }
        if (findFlag) {
          resultObs.type = LABEL_CONE;
          resultObs.x = Cone.x;
          resultObs.y = Cone.y;
          resultObs.height = Cone.height;
          resultObs.width = Cone.width;
        }
      }
      if (!findFlag) // 障碍之间未识别到
      {
        spdlog::info("Obstacle inside and not found others");
        // 情况二 障碍与赛道边线无间隔
        for (int i = 40; i < track.pointsEdgeLeft.size(); i += 2) {
          if (track.pointsEdgeLeft[i].y < track.pointsEdgeLeft[i - 5].y - 5 /**/
              && track.pointsEdgeLeft[i].x > ROWSIMAGE * block_scale) {
            distance = 0;
            step = Step::End;
            passblock = 0;
            left = false;
            break;
          } else if (track.pointsEdgeRight[i].y >
                         track.pointsEdgeRight[i - 5].y + 5 /**/
                     && track.pointsEdgeLeft[i].x > ROWSIMAGE * block_scale) {
            spdlog::info("Obstacle end");
            distance = 0;
            step = Step::End;
            passblock = 0;
            left = true;
            break;
          }
        }
        // 单车道补线
        if (step != Step::End) {
          curtailTracking(track, left, RoadWidth);
        }
      } else {
        spdlog::info("Obstacle inside and found");
        // 障碍物方向判定（左/右）
        int row = track.pointsEdgeLeft.size() - (resultObs.y - track.rowCutUp);
        int disLeft = resultObs.x + resultObs.width -
                      track.pointsEdgeLeft[row].y; // 障碍右侧到左边线的水平距离
        int disRight = track.pointsEdgeRight[row].y -
                       resultObs.x; // 右边线到障碍左侧的水平距离
        if (disLeft > 0 && disRight > 0 &&
            disLeft <= disRight) //[1] 障碍物在赛道内， 且靠左
        {
          left = false;
          std::vector<POINT> points(4); // 三阶贝塞尔曲线
          points[0] = track.pointsEdgeLeft[row / 2];
          points[1] = {resultObs.y + resultObs.height,
                       resultObs.x + resultObs.width};
          points[2] = {(resultObs.y + resultObs.height + resultObs.y) / 2,
                       resultObs.x + resultObs.width};
          if (resultObs.y >
              track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].x)
            points[3] = track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1];
          else
            points[3] = {resultObs.y, resultObs.x + resultObs.width};

          track.pointsEdgeLeft.resize((size_t)row / 2); // 删除错误路线

          std::vector<POINT> repair = Bezier(0.01, points); // 重新规划车道线
          for (int i = 0; i < repair.size(); i++) {
            repair[i].y += 40;
            // if(repair[i].y>COLSIMAGE) repair[i].y=COLSIMAGE-1;
            track.pointsEdgeLeft.push_back(repair[i]);
          }
        } else if (disLeft > 0 && disRight > 0 &&
                   disLeft > disRight) //[2] 障碍物在赛道内，且靠右
        {
          left = true;
          std::vector<POINT> points(4); // 三阶贝塞尔曲线
          points[0] = track.pointsEdgeRight[row / 2];
          points[1] = {resultObs.y + resultObs.height,
                       resultObs.x - resultObs.width};
          points[2] = {(resultObs.y + resultObs.height + resultObs.y) / 2,
                       resultObs.x - resultObs.width};
          if (resultObs.y >
              track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x)
            points[3] = track.pointsEdgeRight[track.pointsEdgeRight.size() - 1];
          else
            points[3] = {resultObs.y, resultObs.x};

          track.pointsEdgeRight.resize((size_t)row / 2);    // 删除错误路线
          std::vector<POINT> repair = Bezier(0.01, points); // 重新规划车道线
          for (int i = 0; i < repair.size(); i++) {
            repair[i].y -= 40;
            // if(repair[i].y<0) repair[i].y=0;
            track.pointsEdgeRight.push_back(repair[i]);
          }
        }
      }
    }

    if (step == Step::End) {
      distance += speed * frameTime / 1000;
      if (distance > distance_block) {
        first = 0;
        step = Step::None;
        return false;
      }
      curtailTracking(track, left, RoadWidth);
    }

    if (step == Step::None)
      return false;
    else
      return true;
  }

  /**
   * @brief 图像绘制禁行区识别结果
   *
   * @param img 需要叠加显示的图像
   */
  void drawImage(cv::Mat &img) {
    if (findFlag) {
      putText(img, "[2] Obstacle- " + std::to_string(step), cv::Point(30, 10),
              cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
      cv::Rect rect(resultObs.x, resultObs.y, resultObs.width,
                    resultObs.height);
      cv::rectangle(img, rect, cv::Scalar(0, 0, 255), 1);
    }
  }

private:
  float distance = 0;

  /**
   * @brief 缩减优化车道线（双车道→单车道）
   *
   * @param track
   * @param left
   */
  void curtailTracking(Tracking &track, bool left,
                       const std::vector<int> &RoadWidth) {
    if (left) {
      if (track.pointsEdgeRight.size() > track.pointsEdgeLeft.size())
        track.pointsEdgeRight.resize(track.pointsEdgeLeft.size());

      for (int i = 0; i < track.pointsEdgeRight.size(); i++) {
        track.pointsEdgeRight[i].y =
            track.pointsEdgeLeft[i].y + RoadWidth[i] / 2;
      }
      // 向右侧缩进
    } else {
      if (track.pointsEdgeRight.size() < track.pointsEdgeLeft.size())
        track.pointsEdgeLeft.resize(track.pointsEdgeRight.size());

      for (int i = 0; i < track.pointsEdgeLeft.size(); i++) {
        track.pointsEdgeLeft[i].y =
            track.pointsEdgeRight[i].y - RoadWidth[i] / 2;
      }
    }
  }
  bool hsvSearch(cv::Mat image, Tracking track) {
    // 图像边长压缩比例为2：1
    // 将图像从BGR转换为HSV颜色空间，以便更容易提取红色
    int red_c = 0, purple_c = 0, yellow_l = 0, yellow_r = 0, yellow_c = 0;
    cv::Mat hsv_img;
    cvtColor(image, hsv_img, cv::COLOR_BGR2HSV);
    int bound = (track.pointsEdgeLeft.size() > 30) ? 30 : 0;

    for (int i = bound; i < track.pointsEdgeLeft.size(); i += 2) {
      if (track.pointsEdgeLeft[i].x < 3 ||
          track.pointsEdgeRight[i].x > COLSIMAGE - 3)
        continue; // 跳过无效点
      for (int y = 0; y < COLSIMAGE; y += 2) {
        cv::Vec3b hsv = hsv_img.at<cv::Vec3b>(track.pointsEdgeLeft[i].x, y);
        // 找红色
        if (((hsv[0] >= 170 && hsv[0] <= 180) ||
             (hsv[0] >= 0 && hsv[0] <= 10)) &&
            hsv[1] >= 44 && hsv[2] >= 85 && hsv[1] <= 231 && hsv[2] <= 255) {
          if (y < track.pointsEdgeRight[i].y && y > track.pointsEdgeLeft[i].y)
            red_c++;
          continue;
        }
        // 找紫色
        if ((hsv[0] >= 92 && hsv[0] <= 147) && hsv[1] >= 18 && hsv[2] >= 91 &&
            hsv[1] <= 115 && hsv[2] <= 255) {
          if (y < track.pointsEdgeRight[i].y && y > track.pointsEdgeLeft[i].y)
            purple_c++;
          continue;
        }
        if ((hsv[0] >= 15 && hsv[0] <= 35) && hsv[1] >= 100 && hsv[2] >= 60 &&
            hsv[1] <= 255 && hsv[2] <= 255) {
          if (y < track.pointsEdgeLeft[i].y)
            yellow_l++;
          else if (y > track.pointsEdgeRight[i].y)
            yellow_r++;
          else if (y > track.pointsEdgeLeft[i].y &&
                   y < track.pointsEdgeRight[i].y) {
            yellow_c++;
          }
        }
      }
    }
    colorcnt[0] = red_c;
    colorcnt[1] = yellow_l;
    colorcnt[2] = yellow_c;
    colorcnt[3] = yellow_r;
    // params[0]=red;params[1]=yellow_l;params[2]=yellow_c;params[3]=yellow_r;
    // if (yellow_l >= 20 && yellow_c <= 3 && yellow_r >= 20) {
    //   RescueFlag = true;
    //   return;
    // }

    if (yellow_c >= 20 || red_c >= 20 || purple_c >= 20) {
      spdlog::info("Obstacle found");
      spdlog::info("Color counts - Red: {}, purple c: {}, Yellow Center: {}, "
                   "Yellow Right: {}",
                   red_c, purple_c, yellow_c, yellow_r);
      return true;
    }
    return false;
  }
};
