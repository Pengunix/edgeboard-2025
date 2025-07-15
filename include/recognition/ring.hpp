#pragma once
#include "recognition/tracking.hpp"
enum RingStep {
  None = 0, // 未知类型
  Verifing,
  Waiting,
  Entering, // 入环
  Inside,   // 环中
  Exiting,  // 出环
  Finish    // 环任务结束
};

/**
 * @brief 环岛类型
 *
 */
enum RingType {
  RingNone = 0, // 未知类型
  RingLeft,     // 左入环岛
  RingRight     // 右入环岛
};

class Ring {
public:
  RingStep ringStep = RingStep::None; // 环岛处理阶段

  RingType ringType = RingType::RingLeft; // 环岛类型
  uint8_t ringNum = 0;
  // uint16_t counterShield = 0; // 环岛检测屏蔽计数器：屏蔽车库误检测
  bool R100 = false;

  /**
   * @brief 环岛识别初始化|复位
   *
   */
  void reset(void) {
    RingType ringType = RingType::RingLeft; // 环岛类型
    RingStep ringStep = RingStep::None;     // 环岛处理阶段
    int rowRepairLine = 0;                  // 用于环补线的点（行号）
    int colRepairLine = 0;                  // 用于环补线的点（列号）
    // counterSpurroad = 0;                    // 岔路计数器
    // counterShield = 0;
  }
  /**
   * @brief 环岛识别与行径规划
   *
   * @param track 基础赛道识别结果
   * @param imagePath 赛道路径图像
   */
  bool ringRecognition(Tracking &track, uint8_t ringSum, uint8_t *ringEnter,
                       const std::vector<int> &RoadWidth) {
    if (ringStep == RingStep::None) {
      // 左环
      if (track.stdevRight < 15 && track.stdevLeft > 20) {
        uint8_t right_sum = 0;  // 右边线数目
        uint8_t left_cnt = 0;   // 判断增减趋势
        uint8_t left_state = 0; // 0:增, 1 2:减
        uint8_t left_edge = 0;  // 左丢线数目
        // 0左丢线计数 1内环凸起计数 2内环左凹计数
        int cnt[3] = {0, 0, 0};
        for (int i = 2; i < track.widthBlock.size(); i += 2) {
          // 若当前点在上个点的左边或贴近边缘视作丢线
          if (track.pointsEdgeRight[i].y <= track.pointsEdgeRight[i - 2].y &&
              track.pointsEdgeRight[i - 2].y < COLSIMAGE - 3) {
            right_sum++;
          } else {
            right_sum = 0;
          }

          if (left_state == 0) {
            if (track.pointsEdgeLeft[i].y <= 3) // 丢线
            {
              cnt[0]++; // 25
            }
            // 左边点存在向右凸起
            if (track.pointsEdgeLeft[i - 2].y <= 3 &&
                track.pointsEdgeLeft[i].y <= 20 &&
                track.pointsEdgeLeft[i].y > 3) {
              left_state++;
            }
          } else if (left_state == 1) {
            if (track.pointsEdgeLeft[i].y >
                track.pointsEdgeLeft[i - 2].y + 1) // 左边线向中间增
            {
              cnt[1]++; // 5
            }
            // 内环边线向左偏
            if (track.pointsEdgeLeft[i].y < track.pointsEdgeLeft[i - 2].y - 1) {
              left_state++;
            }
          } else if (left_state == 2) {
            if (track.pointsEdgeLeft[i].y <
                track.pointsEdgeLeft[i - 2].y - 1) // 左边线向左减
            {
              cnt[2]++;
            }
          }
        }
        // printf("%d %d %d %d\n", left_state,left_cnt,right_sum,left_edge );
        // printf("%d %d %d %d\n", cnt[0], cnt[1], cnt[2], left_edge);
        // params[0]=left_state;params[1]=left_cnt;params[2]=right_sum;params[3]=left_edge;
        if (right_sum > 60 &&
            ((cnt[0] > 5 && cnt[0] < 20 && cnt[1] > 10 && cnt[1] < 45 &&
              cnt[2] > 0 && track.stdevLeft < 70) ||
             (cnt[0] > 25 && cnt[1] > 10 && cnt[2] > 3 &&
              track.stdevLeft > 80))) {
          if ((cnt[0] > 5 && cnt[0] < 20 && cnt[1] > 10 && cnt[1] < 45 &&
               cnt[2] > 0 && track.stdevLeft < 70)) {
            R100 = true;
          } else {
            R100 = false;
          }
          ringStep = RingStep::Verifing;
          ringType = RingType::RingLeft;
        }
      }

      // 右环
      if (track.stdevLeft < 15 && track.stdevRight > 20) {
        // std::cout << ringStep << std::endl;
        uint8_t left_sum = 0;
        uint8_t right_cnt = 0;
        uint8_t right_state = 0;
        uint8_t right_edge = 0;
        int cnt[3] = {0, 0, 0};
        for (int i = 2; i < track.widthBlock.size(); i += 2) {
          //  if(track.pointsEdgeLeft[i].y - track.pointsEdgeLeft[i-2].y > 10)
          //  {left_sum=0;break;}

          if (track.pointsEdgeLeft[i].y >= track.pointsEdgeLeft[i - 2].y &&
              track.pointsEdgeLeft[i].y > 3) {
            left_sum++;
          } else
            left_sum = 0;

          if (right_state == 0) {
            if (track.pointsEdgeRight[i].y >= COLSIMAGE - 3) // 丢线
            {
              cnt[0]++;
            }
            if (track.pointsEdgeRight[i - 2].y >= COLSIMAGE - 3 &&
                track.pointsEdgeRight[i].y >= COLSIMAGE - 20 &&
                track.pointsEdgeRight[i].y < COLSIMAGE - 3) {
              right_state++;
            }
          } else if (right_state == 1) {
            if (track.pointsEdgeRight[i].y <
                track.pointsEdgeRight[i - 2].y - 1) // 右边线向中间增
            {
              cnt[1]++;
            }
            if (track.pointsEdgeRight[i].y >
                track.pointsEdgeRight[i - 2].y + 1) {
              right_state++;
            }
          } else if (right_state == 2) {
            if (track.pointsEdgeRight[i].y >
                track.pointsEdgeRight[i - 2].y + 1) // 右边线向右边减
            {
              cnt[2]++;
            }
          }
        }
        if (left_sum > 60 &&
            ((cnt[0] > 5 && cnt[0] < 20 && cnt[1] > 10 && cnt[1] < 45 &&
              cnt[2] > 0 && track.stdevRight < 70) ||
             (cnt[0] > 25 && cnt[1] > 10 && cnt[2] > 3 &&
              track.stdevRight > 80))) {
          if ((cnt[0] > 5 && cnt[0] < 20 && cnt[1] > 10 && cnt[1] < 45 &&
               cnt[2] > 0 && track.stdevRight < 70)) {
            R100 = true;
          } else {
            R100 = false;
          }
          ringStep = RingStep::Verifing;
          ringType = RingType::RingRight;
        }
      }
    }

    else if (ringStep == RingStep::Verifing) {
      spdlog::info("环岛 Verifying");
      if (ringType == RingType::RingLeft) {
        if (track.stdevRight > 15) {
          ringStep = RingStep::None;
          return false;
        }
        int fixline = 0;
        int state = 0;
        int cnt[3] = {0, 0, 0};
        for (int i = 2; i < track.pointsEdgeLeft.size(); i++) {
          if (state == 0) {
            if (track.pointsEdgeLeft[i].y >
                track.pointsEdgeLeft[i - 2].y) // 左边线向中间增
            {
              cnt[0]++;
            }
            if (track.pointsEdgeLeft[i].y < track.pointsEdgeLeft[i - 2].y)
              state++;
          }
          if (state == 1) {
            if (track.pointsEdgeLeft[i].y <
                track.pointsEdgeLeft[i - 2].y) // 左边线向中间减
            {
              // 3
              cnt[1]++;
            }
            if (track.pointsEdgeLeft[i].y <= 3)
              state++;
          }
          if (state == 2) // 丢线
          {
            if (track.pointsEdgeLeft[i].y <= 3)
              cnt[2]++;
          }
        }
        // printf("%d %d %d\n", cnt[0], cnt[1], cnt[2]);
        if (!R100 && cnt[0] > 5 && cnt[1] > 3 && cnt[2] >= 2)
          ringStep = RingStep::Waiting;
        if (R100 && cnt[0] > 40 && cnt[1] > 20 && cnt[2] >= 1)
          ringStep = RingStep::Inside;

        if (ringStep == RingStep::Verifing) {
          for (int i = 0; i < track.pointsEdgeLeft.size(); i++) {
            track.pointsEdgeLeft[i].y =
                track.pointsEdgeRight[i].y - RoadWidth[i];
          }
        }
      } else if (ringType == RingType::RingRight) {
        if (track.stdevLeft > 15) {
          ringStep = RingStep::None;
          return false;
        }
        int fixline = 0;
        int state = 0;
        int cnt[3] = {0, 0, 0};
        for (int i = 2; i < track.pointsEdgeRight.size(); i++) {
          if (state == 0) {
            if (track.pointsEdgeRight[i].y <
                track.pointsEdgeRight[i - 2].y) // 右边线向中间减
            {
              cnt[0]++;
            }
            if (track.pointsEdgeRight[i].y > track.pointsEdgeRight[i - 2].y)
              state++;
          }
          if (state == 1) {
            if (track.pointsEdgeRight[i].y > track.pointsEdgeRight[i - 2].y) {
              cnt[1]++;
            }
            if (track.pointsEdgeRight[i].y >= COLSIMAGE - 3)
              state++;
          }
          if (state == 2) {
            if (track.pointsEdgeRight[i].y >= COLSIMAGE - 3)
              cnt[2]++;
          }
        }
        // printf("%d %d %d\n", cnt[0], cnt[1], cnt[2]);
        if (!R100 && cnt[0] > 5 && cnt[1] > 3 && cnt[2] >= 2)
          ringStep = RingStep::Waiting;
        if (R100 && cnt[0] > 40 && cnt[1] > 20 && cnt[2] >= 1)
          ringStep = RingStep::Inside;

        if (ringStep == RingStep::Verifing) {
          for (int i = 0; i < track.pointsEdgeRight.size(); i++) {
            track.pointsEdgeRight[i].y =
                track.pointsEdgeLeft[i].y + RoadWidth[i];
          }
        }
      }
    }

    else if (ringStep == RingStep::Waiting) {
      spdlog::info("环岛 Waiting");
      if (ringType == RingType::RingLeft) {
        // 这里改过, 20
        if (track.stdevRight > 50) {
          ringStep = RingStep::None;
          return false;
        }
        int fixline = 0;
        int whitecnt = 0;
        bool search = true;
        POINT endpoint = POINT(0, 0);

        for (int i = 0; i < track.pointsEdgeLeft.size(); i++) {
          if (track.pointsEdgeLeft[i].y < 200 &&
              track.pointsEdgeLeft[i].y > track.pointsEdgeLeft[fixline].y)
            fixline = i;
          if (search && whitecnt > 5 && track.pointsEdgeLeft[i].y >= 3)
            search = false;
          if (search && track.pointsEdgeLeft[i].x < 85 &&
              track.pointsEdgeLeft[i].y < 3) {
            endpoint = track.pointsEdgeLeft[i];
            whitecnt++;
          }
        }
        _ringPoint = endpoint;
        spdlog::info("endpoint x {}, y {}", endpoint.x, endpoint.y);

        if (!search && endpoint.x > ringEnter[ringNum]) {
          ringStep = RingStep::Entering;
          EnteringCounter = 0;
        }

        if (ringStep == RingStep::Waiting) {
          for (int i = 0; i < track.pointsEdgeLeft.size(); i++) {
            track.pointsEdgeLeft[i].y =
                track.pointsEdgeRight[i].y - RoadWidth[i];
          }
        }
      } else if (ringType == RingType::RingRight) {
        if (track.stdevLeft > 20) {
          ringStep = RingStep::None;
          return false;
        }
        int fixline = 0;
        int whitecnt = 0;
        bool search = true;
        POINT endpoint = POINT(0, COLSIMAGE - 1);

        for (int i = 0; i < track.pointsEdgeRight.size(); i++) {
          if (track.pointsEdgeRight[i].y >= 120 &&
              track.pointsEdgeRight[i].y < track.pointsEdgeRight[fixline].y)
            fixline = i;
          if (search && whitecnt > 5 &&
              track.pointsEdgeRight[i].y <= COLSIMAGE - 4)
            search = false;
          if (search && track.pointsEdgeRight[i].x < 85 &&
              track.pointsEdgeRight[i].y > COLSIMAGE - 4) {
            endpoint = track.pointsEdgeRight[i];
            whitecnt++;
          }
        }
        _ringPoint = endpoint;
        spdlog::info("endpoint x {}, y {}", endpoint.x, endpoint.y);
        if (!search && endpoint.x > ringEnter[ringNum]) {
          ringStep = RingStep::Entering;
          EnteringCounter = 0;
        }

        if (ringStep == RingStep::Waiting) {
          for (int i = 0; i < track.pointsEdgeRight.size(); i++) {
            track.pointsEdgeRight[i].y =
                track.pointsEdgeLeft[i].y + RoadWidth[i];
          }
        }
      }
    } else if (ringStep == RingStep::Entering) {
      spdlog::info("环岛 Entering");
      if (ringType == RingType::RingLeft) {
        //     EnteringCounter ++;  //入环计数器
        //   printf("%d %d\n", track.pointsEdgeRight.size(),
        //   track.pointsEdgeRight[10].y);
        // TODO(me) 这里环岛结束补线太过激进
        /* || track.pointsEdgeRight.size() < 140*/
        /*|| EnteringCounter > 40*/

        // 这里改过
        if ((track.pointsEdgeRight.size() < 180 &&
             track.pointsEdgeRight[10].y < COLSIMAGE - 20 && track.pointsEdgeRight[30].y < COLSIMAGE - 50 && track.stdevRight > 100)) {
          ringStep = RingStep::Inside;
        }

        // 补线
        if (ringStep == RingStep::Entering) {
          int whitecnt = 0;
          POINT endpoint = POINT(0, 0);
          for (int i = 0; i < track.pointsEdgeLeft.size(); i++) {
            if (whitecnt > 50 && track.pointsEdgeLeft[i].y >= 3)
              break;
            if (track.pointsEdgeLeft[i].x < 160 &&
                track.pointsEdgeLeft[i].y < 3) {
              endpoint = track.pointsEdgeLeft[i];
              whitecnt++;
            }
          }
          POINT start = POINT(ROWSIMAGE - 31, COLSIMAGE - 31);
          POINT end = POINT(endpoint.x, endpoint.y);
          //  POINT end = POINT(track.pointsEdgeLeft[i].x,
          //  track.pointsEdgeLeft[i].y);
          POINT middle =
              POINT((start.x + end.x) * 0.3, (start.y + end.y) * 0.7);
          std::vector<POINT> input = {start, middle, end};
          track.pointsEdgeRight = Bezier(0.02, input);

          track.pointsEdgeLeft.clear();
          for (int i = 0; i < track.pointsEdgeRight.size(); i++) {
            track.pointsEdgeLeft.push_back(
                POINT(track.pointsEdgeRight[i].x, 0));
          }
        }
      } else if (ringType == RingType::RingRight) {
        // EnteringCounter ++;
        //    printf("%d %d\n", track.pointsEdgeLeft.size(),
        //    track.pointsEdgeLeft[10].y);
        if ((track.pointsEdgeLeft.size() < 180 &&
             track.pointsEdgeLeft[10].y >
                 20) /*|| track.pointsEdgeLeft.size() < 140*/
            /* || EnteringCounter > 40*/) {
          ringStep = RingStep::Inside;
        }

        if (ringStep == RingStep::Entering) {
          int whitecnt = 0;
          POINT endpoint = POINT(0, COLSIMAGE - 1);

          for (int i = 0; i < track.pointsEdgeRight.size(); i++) {
            if (whitecnt > 50 && track.pointsEdgeRight[i].y <= COLSIMAGE - 4)
              break;
            if (track.pointsEdgeRight[i].x < 160 &&
                track.pointsEdgeRight[i].y > COLSIMAGE - 4) {
              endpoint = track.pointsEdgeRight[i];
              whitecnt++;
            }
          }

          POINT start = POINT(ROWSIMAGE - 31, 30);
          POINT end = POINT(endpoint.x, endpoint.y);
          POINT middle =
              POINT((start.x + end.x) * 0.3, (start.y + end.y) * 0.3);
          std::vector<POINT> input = {start, middle, end};
          track.pointsEdgeLeft = Bezier(0.02, input);

          track.pointsEdgeRight.clear();
          for (int i = 0; i < track.pointsEdgeLeft.size(); i++) {
            track.pointsEdgeRight.push_back(
                POINT(track.pointsEdgeLeft[i].x, COLSIMAGE - 1));
          }
        }
      }
    } else if (ringStep == RingStep::Inside) {
      //  printf("Inside\n");
      if (ringType == RingType::RingLeft) {
        if (R100) {
          static bool In = false;
          if (!In && track.stdevRight > 28)
            In = true;
          if (In && track.stdevRight > 1 && track.stdevRight < 10) {
            In = false;
            ringStep = RingStep::None;
            ringType = RingType::RingNone;
            ringNum = (ringNum + 1) % ringSum;
          }
        } else {
          uint8_t right_cnt = 0;
          uint8_t right_state = 0;

          for (int i = 2; i < track.pointsEdgeRight.size(); i += 2) {
            if (!right_state) {
              if (track.pointsEdgeRight[i].y > track.pointsEdgeRight[i - 2].y) {
                right_cnt++;
                right_state = 1;
              }
            } else {
              if (track.pointsEdgeRight[i].y > track.pointsEdgeRight[i - 2].y ||
                  track.pointsEdgeRight[i].y > COLSIMAGE - 5) {
                right_cnt++;
              }
            }
          }
          if (right_cnt > 20) {
            ringStep = RingStep::Exiting;
          }
        }
        //    printf("%d %d\n",right_state,  right_cnt);
      } else if (ringType == RingType::RingRight) {
        if (R100) {
          static bool In = false;
          if (!In && track.stdevLeft > 28)
            In = true;
          if (In && track.stdevLeft > 1 && track.stdevLeft < 10) {
            In = false;
            ringStep = RingStep::None;
            ringType = RingType::RingNone;
            ringNum = (ringNum + 1) % ringSum;
          }
        } else {
          uint8_t left_cnt = 0;
          uint8_t left_state = 0;

          for (int i = 2; i < track.pointsEdgeLeft.size(); i += 2) {
            if (!left_state) {
              if (track.pointsEdgeLeft[i].y < track.pointsEdgeLeft[i - 2].y) {
                left_cnt++;
                left_state = 1;
              }
            } else {
              if (track.pointsEdgeLeft[i].y < track.pointsEdgeLeft[i - 2].y ||
                  track.pointsEdgeLeft[i].y < 4) {
                left_cnt++;
              }
            }
          }
          if (left_cnt > 20) {
            ringStep = RingStep::Exiting;
          }
        }
        //    printf("%d %d\n",left_state,  left_cnt);
      }
    } else if (ringStep == RingStep::Exiting) {
      spdlog::info("环岛Exiting");
      if (ringType == RingType::RingLeft) {
        int whitecnt = 0;
        int stage = 0;
        bool left_leave = false;
        for (int i = 2; i < track.pointsEdgeRight.size(); i += 2) {
          if (track.pointsEdgeRight[i].y > COLSIMAGE - 4)
            whitecnt++; // 右丢线
          if (stage == 0 &&
              track.pointsEdgeRight[i].y < track.pointsEdgeRight[i - 2].y)
            stage++; //
          if (stage == 1 &&
              track.pointsEdgeRight[i].y > track.pointsEdgeRight[i - 2].y) {
            stage++;
            break;
          }
        }
        for (int i = 2; i < track.pointsEdgeLeft.size(); i += 2) {
          if (!left_leave && track.pointsEdgeLeft[i].y > 3 &&
              track.pointsEdgeLeft[i - 2].y < 3)
            left_leave = true;
        }

        if (stage == 1 && track.stdevRight < 50 && track.stdevRight > 10 && track.stdevLeft < 500 &&
            (whitecnt < 10 || left_leave)) {
          ringStep = RingStep::None;
          ringType = RingType::RingNone;
          ringNum = (ringNum + 1) % ringSum;
        }

        if (ringStep == RingStep::Exiting) {
          POINT endpoint = POINT(120, 0); // tag
          for (int i = track.pointsEdgeLeft.size() - 1; i >= 0; i--) {
            if (track.pointsEdgeLeft[i].y < 3) {
              endpoint.x = track.pointsEdgeLeft[i].x;
              break;
            }
          }

          POINT start = POINT(ROWSIMAGE - 30, COLSIMAGE - 31); // tag
          POINT end = POINT(endpoint.x, endpoint.y);
          POINT middle =
              POINT((start.x + end.x) * 0.4, (start.y + end.y) * 0.6);
          std::vector<POINT> input = {start, middle, end};
          track.pointsEdgeRight = Bezier(0.02, input);

          track.pointsEdgeLeft.clear();
          for (int i = 0; i < track.pointsEdgeRight.size(); i++) {
            track.pointsEdgeLeft.push_back(
                POINT(track.pointsEdgeRight[i].x, 0));
          }
        }
      } else if (ringType == RingType::RingRight) {
        int whitecnt = 0;
        int stage = 0;
        int right_leave = false;
        for (int i = 2; i < track.pointsEdgeLeft.size(); i += 2) {
          if (track.pointsEdgeLeft[i].y < 3)
            whitecnt++;
          if (stage == 0 &&
              track.pointsEdgeLeft[i].y > track.pointsEdgeLeft[i - 2].y)
            stage++;
          if (stage == 1 &&
              track.pointsEdgeLeft[i].y < track.pointsEdgeLeft[i - 2].y) {
            stage++;
            break;
          }
        }

        for (int i = 2; i < track.pointsEdgeRight.size(); i += 2) {
          if (!right_leave && track.pointsEdgeRight[i].y < COLSIMAGE - 3 &&
              track.pointsEdgeRight[i - 2].y > COLSIMAGE - 3)
            right_leave = true;
        }

        if (stage == 1 && track.stdevLeft < 50 && track.stdevLeft > 10 && track.stdevRight < 500 &&
            (whitecnt < 10 || right_leave)) {
          ringStep = RingStep::None;
          ringType = RingType::RingNone;
          ringNum = (ringNum + 1) % ringSum;
        }

        if (ringStep == RingStep::Exiting) {
          POINT endpoint = POINT(120, COLSIMAGE - 1); // tag
          for (int i = track.pointsEdgeRight.size() - 1; i >= 0; i--) {
            if (track.pointsEdgeRight[i].y > COLSIMAGE - 4) {
              endpoint.x = track.pointsEdgeRight[i].x;
              break;
            }
          }

          POINT start = POINT(ROWSIMAGE - 30, 30); // tag
          POINT end = POINT(endpoint.x, endpoint.y);
          POINT middle =
              POINT((start.x + end.x) * 0.4, (start.y + end.y) * 0.4);
          std::vector<POINT> input = {start, middle, end};
          track.pointsEdgeLeft = Bezier(0.02, input);

          track.pointsEdgeRight.clear();
          for (int i = 0; i < track.pointsEdgeLeft.size(); i++) {
            track.pointsEdgeRight.push_back(
                POINT(track.pointsEdgeLeft[i].x, COLSIMAGE - 1));
          }
        }
      }
    }

    if (ringStep == RingStep::None) {
      return false;
    } else {
      // spdlog::info("环岛 边线点集长度 left {}, right {}",
      //  track.pointsEdgeLeft.size(), track.pointsEdgeRight.size());
      return true;
    }
  }

  void drawimg(cv::Mat &image) {
    if (ringStep == RingStep::None) {
      return;
    }
    cv::circle(image, cv::Point(_ringPoint.y, _ringPoint.x), 5,
               cv::Scalar(0, 0, 255), -1);
    cv::putText(image, "Ring Step: " + std::to_string(ringStep),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 255, 0), 1);
    cv::putText(image, "Ring Type: " + std::to_string(ringType),
                cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 255, 0), 1);
    cv::putText(image, "Ring Num: " + std::to_string(ringNum),
                cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 255, 0), 1);
    // spdlog::info("环岛 边线点集长度 left {}, right {}",
    // track.pointsEdgeLeft.size(), track.pointsEdgeRight.size());
  }

private:
  float distance = 0;
  // uint16_t counterSpurroad = 0; // 岔路计数器
  // 临时测试用参数
  POINT _ringPoint = POINT(0, 0);

  /**
   * @brief 环岛运行步骤/阶段
   *
   */

  int rowRepairLine = 0; // 用于环补线的点（行号）
  int colRepairLine = 0; // 用于环补线的点（列号）

  uint8_t EnteringCounter = 0;
};
