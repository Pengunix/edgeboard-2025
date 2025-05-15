#pragma once
#include "common.hpp"

class Crossroad {
public:
  bool crossstraight = true;
  bool last_left_down = false, last_right_down = false;
  cv::Mat inv_Mat = auto_init_ipm_mat();
  /**
   * @brief 初始化
   *
   */
  void reset(void) {
    crossroadType = CrossroadType::None; // 十字道路类型
  }

  /**
   * @brief 十字道路识别与图像处理
   *
   * @param track 赛道识别结果
   * @param imagePath 输入图像
   */
  bool crossRecognition(Tracking &track) {
    int left_up = 0, left_stage = 0;
    int left_down = 0, left_down_1 = 0;
    int right_up = 0, right_stage = 0;
    int right_down = 0, right_down_1 = 0;

    // 当track.pointsEdgeRight.size()小于10时，size - 10会下溢
    if (track.pointsEdgeLeft.size() <= 10) {
      return false;
    }
    for (int i = 10; i < (track.pointsEdgeLeft.size() - 10); i += 2) {
      // if(!left_down && left_down_1!=0 && track.pointsEdgeLeft[i - 5].y -
      // track.pointsEdgeLeft[i].y >= 2
      //     && track.pointsEdgeLeft[i - 5].y - track.pointsEdgeLeft[i - 10].y
      //     >= 2) left_down = i-10;
      if (i>(track.pointsEdgeLeft.size())) break;
      // std::cout << "i " << i << " size " << track.pointsEdgeLeft.size() << std::endl; 
      if (track.pointsEdgeLeft[i].y - track.pointsEdgeLeft[i - 5].y <= -20 &&
          track.pointsEdgeLeft[i - 5].y >= track.pointsEdgeLeft[i - 9].y) {
        if (left_down == 0)
          left_down = i - 5;
        else if (track.pointsEdgeLeft[i - 5].y >
                     track.pointsEdgeLeft[left_down].y &&
                 abs(track.pointsEdgeLeft[i - 5].x -
                     track.pointsEdgeLeft[left_down].x) < 10)
          left_down = i - 5;
      }
      if (2 * track.pointsEdgeLeft[i - 5].y - track.pointsEdgeLeft[i].y -
                  track.pointsEdgeLeft[i - 9].y >=
              10 &&
          track.pointsEdgeLeft[i - 5].y - track.pointsEdgeLeft[i].y >= 2 &&
          track.pointsEdgeLeft[i - 5].y - track.pointsEdgeLeft[i - 9].y >= 2) {
        if (left_down == 0)
          left_down = i - 5;
        else if (track.pointsEdgeLeft[i - 5].y >
                     track.pointsEdgeLeft[left_down].y &&
                 abs(track.pointsEdgeLeft[i - 5].x -
                     track.pointsEdgeLeft[left_down].x) < 10)
          left_down = i - 5;
      }

      if (track.pointsEdgeLeft[i].y - track.pointsEdgeLeft[i - 5].y >= -5 &&
          track.pointsEdgeLeft[i].y - track.pointsEdgeLeft[i - 5].y <= 4 &&
          track.pointsEdgeLeft[i - 5].y - track.pointsEdgeLeft[i - 9].y >=
              20) {
        if (left_up == 0)
          left_up = i - 5;
        else if (track.pointsEdgeLeft[i].y > track.pointsEdgeLeft[left_up].y)
          left_up = i - 5;
      }
    }

  
    // 当track.pointsEdgeRight.size()小于10时，size - 10会下溢
    if (track.pointsEdgeRight.size() <= 10) {
      return false;
    }
    for (int i = 10; i < track.pointsEdgeRight.size() - 10; i += 2) {
      if (track.pointsEdgeRight[i].y - track.pointsEdgeRight[i - 5].y >= 20 &&
          track.pointsEdgeRight[i - 5].y <= track.pointsEdgeRight[i - 9].y) {
        if (right_down == 0)
          right_down = i - 5;
        else if (track.pointsEdgeRight[i - 5].y <
                     track.pointsEdgeRight[right_down].y &&
                 abs(track.pointsEdgeRight[i - 5].x -
                     track.pointsEdgeRight[right_down].x) < 10)
          right_down = i - 5;
      }
      if (2 * track.pointsEdgeRight[i - 5].y - track.pointsEdgeRight[i].y -
                  track.pointsEdgeRight[i - 9].y <=
              -10 &&
          track.pointsEdgeRight[i - 5].y - track.pointsEdgeRight[i].y <= -2 &&
          track.pointsEdgeRight[i - 5].y - track.pointsEdgeRight[i - 9].y <=
              -2) {
        if (right_down == 0)
          right_down = i - 5;
        else if (track.pointsEdgeRight[i - 5].y <
                     track.pointsEdgeRight[right_down].y &&
                 abs(track.pointsEdgeRight[i - 5].x -
                     track.pointsEdgeRight[right_down].x) < 10)
          right_down = i - 5;
      }

      if (track.pointsEdgeRight[i].y - track.pointsEdgeRight[i - 5].y <= 5 &&
          track.pointsEdgeRight[i].y - track.pointsEdgeRight[i - 5].y >= -4 &&
          track.pointsEdgeRight[i - 5].y - track.pointsEdgeRight[i - 9].y <=
              -20) {
        if (right_up == 0)
          right_up = i - 5;
        else if (track.pointsEdgeRight[i].y < track.pointsEdgeRight[right_up].y)
          right_up = i - 5;
      }
    }
    if (abs(track.pointsEdgeLeft[left_up].x -
            track.pointsEdgeLeft[left_down].x) < 10)
      left_up = 0;
    if (abs(track.pointsEdgeRight[right_up].x -
            track.pointsEdgeRight[right_down].x) < 10)
      right_up = 0;

    if (left_down != 0 && left_up != 0 &&
        track.pointsEdgeLeft[left_up].x < track.pointsEdgeLeft[left_down].x) {
      if (track.pointsEdgeLeft[left_down].x !=
          track.pointsEdgeLeft[left_up].x) {
        int temp_y;
        for (int i = left_down + 1; i < left_up; i++) {
          temp_y =
              track.pointsEdgeLeft[left_down].y +
              (track.pointsEdgeLeft[left_down].x - track.pointsEdgeLeft[i].x) *
                  (track.pointsEdgeLeft[left_up].y -
                   track.pointsEdgeLeft[left_down].y) /
                  (track.pointsEdgeLeft[left_down].x -
                   track.pointsEdgeLeft[left_up].x);
          if (temp_y < 0)
            track.pointsEdgeLeft[i].y = 0;
          else if (temp_y >= COLSIMAGE)
            track.pointsEdgeLeft[i].y = COLSIMAGE - 1;
          else
            track.pointsEdgeLeft[i].y = temp_y;
        }
      }
    } else {
      if (left_down != 0) {
        uint8_t left_begin = left_down > 20 ? left_down - 20 : 0;
        while (abs(track.pointsEdgeLeft[left_begin].y -
                   track.pointsEdgeLeft[left_down].y) > 10 &&
               left_begin < left_down - 10)
          left_begin++;
        if (track.pointsEdgeLeft[left_begin].x !=
            track.pointsEdgeLeft[left_down].x) {
          int temp_y;
          for (int i = left_down + 1; i < track.pointsEdgeLeft.size(); i++) {
            temp_y = track.pointsEdgeLeft[left_down].y +
                     (track.pointsEdgeLeft[left_down].x -
                      track.pointsEdgeLeft[i].x) *
                         (track.pointsEdgeLeft[left_down].y -
                          track.pointsEdgeLeft[left_begin].y) /
                         (track.pointsEdgeLeft[left_begin].x -
                          track.pointsEdgeLeft[left_down].x);
            if (temp_y < 0)
              track.pointsEdgeLeft[i].y = 0;
            else if (temp_y >= COLSIMAGE)
              track.pointsEdgeLeft[i].y = COLSIMAGE - 1;
            else
              track.pointsEdgeLeft[i].y = temp_y;
          }
          // }
        }
      }
      if (left_up != 0) {
        uint8_t left_end = left_up + 20 < track.pointsEdgeLeft.size() - 1
                               ? left_up + 20
                               : track.pointsEdgeLeft.size() - 1;
        while (abs(track.pointsEdgeLeft[left_end].y -
                   track.pointsEdgeLeft[left_up].y) > 10 &&
               left_end > left_up + 10)
          left_end--;
        if (track.pointsEdgeLeft[left_up].x !=
            track.pointsEdgeLeft[left_end].x) {

          int temp_y;
          for (int i = left_up - 1; i >= 0; i--) {
            temp_y =
                track.pointsEdgeLeft[left_up].y -
                (track.pointsEdgeLeft[i].x - track.pointsEdgeLeft[left_up].x) *
                    (track.pointsEdgeLeft[left_end].y -
                     track.pointsEdgeLeft[left_up].y) /
                    (track.pointsEdgeLeft[left_up].x -
                     track.pointsEdgeLeft[left_end].x);
            if (temp_y < 0)
              track.pointsEdgeLeft[i].y = 0;
            else if (temp_y >= COLSIMAGE)
              track.pointsEdgeLeft[i].y = COLSIMAGE - 1;
            else
              track.pointsEdgeLeft[i].y = temp_y;
          }
          // }
        }
      }
    }

    if (right_down != 0 && right_up != 0 &&
        track.pointsEdgeRight[right_up].x <
            track.pointsEdgeRight[right_down].x) {
      if (track.pointsEdgeRight[right_down].x !=
          track.pointsEdgeRight[right_up].x) {
        int temp_y;
        for (int i = right_down + 1; i < right_up; i++) {
          temp_y = track.pointsEdgeRight[right_down].y +
                   (track.pointsEdgeRight[right_down].x -
                    track.pointsEdgeRight[i].x) *
                       (track.pointsEdgeRight[right_up].y -
                        track.pointsEdgeRight[right_down].y) /
                       (track.pointsEdgeRight[right_down].x -
                        track.pointsEdgeRight[right_up].x);
          if (temp_y < 0)
            track.pointsEdgeRight[i].y = 0;
          else if (temp_y >= COLSIMAGE)
            track.pointsEdgeRight[i].y = COLSIMAGE - 1;
          else
            track.pointsEdgeRight[i].y = temp_y;
        }
      }
    } else {
      if (right_down != 0) {
        uint8_t right_begin = right_down > 20 ? right_down - 20 : 0;
        while (abs(track.pointsEdgeRight[right_begin].y -
                   track.pointsEdgeRight[right_down].y) > 10 &&
               right_begin < right_down - 10)
          right_begin++;
        int temp_y;
        for (int i = right_down + 1; i < track.pointsEdgeRight.size(); i++) {
          temp_y = track.pointsEdgeRight[right_down].y +
                   (track.pointsEdgeRight[right_down].x -
                    track.pointsEdgeRight[i].x) *
                       (track.pointsEdgeRight[right_down].y -
                        track.pointsEdgeRight[right_begin].y) /
                       (track.pointsEdgeRight[right_begin].x -
                        track.pointsEdgeRight[right_down].x);
          if (temp_y < 0)
            track.pointsEdgeRight[i].y = 0;
          else if (temp_y >= COLSIMAGE)
            track.pointsEdgeRight[i].y = COLSIMAGE - 1;
          else
            track.pointsEdgeRight[i].y = temp_y;
        }
        // }
      }

      if (right_up != 0) {
        uint8_t right_end = right_up + 20 < track.pointsEdgeRight.size() - 1
                                ? right_up + 20
                                : track.pointsEdgeRight.size() - 1;
        while (abs(track.pointsEdgeRight[right_end].y -
                   track.pointsEdgeRight[right_up].y) > 10 &&
               right_end > right_up + 10)
          right_end--;
        int temp_y;
        for (int i = right_up - 1; i >= 0; i--) {
          temp_y =
              track.pointsEdgeRight[right_up].y -
              (track.pointsEdgeRight[i].x - track.pointsEdgeRight[right_up].x) *
                  (track.pointsEdgeRight[right_end].y -
                   track.pointsEdgeRight[right_up].y) /
                  (track.pointsEdgeRight[right_up].x -
                   track.pointsEdgeRight[right_end].x);
          if (temp_y < 0)
            track.pointsEdgeRight[i].y = 0;
          else if (temp_y >= COLSIMAGE)
            track.pointsEdgeRight[i].y = COLSIMAGE - 1;
          else
            track.pointsEdgeRight[i].y = temp_y;
        }
      }
    }

    if (left_up || left_down || right_up || right_down) {
      if (left_down && track.pointsEdgeLeft[left_down].x > 50 &&
          track.pointsEdgeLeft[left_down].y > 160)
        crossroadType = CrossroadType::CrossroadRight;
      else if (right_down && track.pointsEdgeRight[right_down].x > 50 &&
               track.pointsEdgeRight[right_down].y < 160)
        crossroadType = CrossroadType::CrossroadLeft;
      else
        crossroadType = CrossroadType::CrossroadStraight;
    } else
      crossroadType = CrossroadType::None;

    if (crossroadType == CrossroadType::CrossroadLeft ||
        crossroadType == CrossroadType::CrossroadRight)
      crossstraight = false;
    else
      crossstraight = true;

    return left_up || left_down || right_up || right_down;
  }

private:
  int _index = 0; // 测试

  POINT pointBreakLU;
  POINT pointBreakLD;
  POINT pointBreakRU;
  POINT pointBreakRD;
  uint16_t counterFild = 0;
  /**
   * @brief 十字道路类型
   *
   */
  enum CrossroadType {
    None = 0,
    CrossroadLeft,     // 左斜入十字
    CrossroadRight,    // 右斜入十字
    CrossroadStraight, // 直入十字
  };

  CrossroadType crossroadType = CrossroadType::None; // 十字道路类型

  /**
   * @brief 搜索十字赛道突变行（左上）
   *
   * @param pointsEdgeLeft
   * @return uint16_t
   */
  uint16_t searchBreakLeftUp(std::vector<POINT> pointsEdgeLeft) {
    uint16_t rowBreakLeftUp = pointsEdgeLeft.size() - 5;
    uint16_t counter = 0;
    uint16_t counterFilter = 0;
    for (int i = pointsEdgeLeft.size() - 5; i > 50; i--) {
      if (pointsEdgeLeft[i].y > 2 &
          abs(pointsEdgeLeft[i].y - pointsEdgeLeft[i + 1].y) < 3) {
        rowBreakLeftUp = i;
        counter = 0;
        counterFilter++;
      } else if (pointsEdgeLeft[i].y <= 2 && counterFilter > 10) {
        counter++;
        if (counter > 5)
          return rowBreakLeftUp;
      }
    }

    return rowBreakLeftUp;
  }
  /**
   * @brief 搜索十字赛道突变行（左下）
   *
   * @param pointsEdgeLeft
   * @return uint16_t
   */
  uint16_t searchBreakLeftDown(std::vector<POINT> pointsEdgeLeft) {
    uint16_t rowBreakLeft = 0;
    uint16_t counter = 0;

    for (int i = 0; i < pointsEdgeLeft.size() / 2; i++) // 寻找左边跳变点
    {
      if (pointsEdgeLeft[i].y > pointsEdgeLeft[rowBreakLeft].y) {
        rowBreakLeft = i;
        counter = 0;
      } else if (pointsEdgeLeft[i].y <=
                 pointsEdgeLeft[rowBreakLeft].y) // 突变点计数
      {
        counter++;
        if (counter > 5)
          return rowBreakLeft;
      }
    }

    return rowBreakLeft;
  }
  /**
   * @brief 搜索十字赛道突变行（右上）
   *
   * @param pointsEdgeRight
   * @return uint16_t
   */
  uint16_t searchBreakRightUp(std::vector<POINT> pointsEdgeRight) {
    uint16_t rowBreakRightUp = pointsEdgeRight.size() - 5;
    uint16_t counter = 0;
    uint16_t counterFilter = 0;
    for (int i = pointsEdgeRight.size() - 5; i > 50; i--) {
      if (pointsEdgeRight[i].y < COLSIMAGE - 2 &
          abs(pointsEdgeRight[i].y - pointsEdgeRight[i + 1].y) < 3) {
        rowBreakRightUp = i;
        counter = 0;
        counterFilter++;
      } else if (pointsEdgeRight[i].y >= COLSIMAGE - 2 && counterFilter > 10) {
        counter++;
        if (counter > 5)
          return rowBreakRightUp;
      }
    }

    return rowBreakRightUp;
  }
  /**
   * @brief 搜索十字赛道突变行（右下）
   *
   * @param pointsEdgeRight
   * @return uint16_t
   */
  uint16_t searchBreakRightDown(std::vector<POINT> pointsEdgeRight) {
    uint16_t rowBreakRightDown = 0;
    uint16_t counter = 0;
    bool start = false;

    for (int i = 0; i < pointsEdgeRight.size() - 10; i++) // 寻找左边跳变点
    {
      if (pointsEdgeRight[i].y < COLSIMAGE - 1)
        counter++;
      else
        counter = 0;

      if (counter > 2) {
        start = true;
        counter = 0;
      }

      if (start) // 屏蔽初始行
      {
        if (pointsEdgeRight[i].y > pointsEdgeRight[i - 2].y)
          counter++;
        else
          counter = 0;

        if (counter > 2)
          return i - 3;
      }
    }

    return rowBreakRightDown;
  }

  /**
   * @brief 直入十字搜索
   *
   * @param pointsEdgeLeft 赛道左边缘点集
   * @param pointsEdgeRight 赛道右边缘点集
   * @return true
   * @return false
   */
  bool searchStraightCrossroad(std::vector<POINT> pointsEdgeLeft,
                               std::vector<POINT> pointsEdgeRight) {
    if (pointsEdgeLeft.size() < ROWSIMAGE * 0.8 ||
        pointsEdgeRight.size() < ROWSIMAGE * 0.8) {
      return false;
    }

    uint16_t counterLeft = 0;
    uint16_t counterRight = 0;
    for (int i = pointsEdgeLeft.size() - 10; i > 1; i--) // 搜索上半部分边缘点
    {
      if (pointsEdgeLeft[i].x > ROWSIMAGE / 2)
        break;
      else if (pointsEdgeLeft[i].y < 2)
        counterLeft++;
    }
    for (int i = pointsEdgeRight.size() - 10; i > 1; i--) // 搜索上半部分边缘点
    {
      if (pointsEdgeRight[i].x > ROWSIMAGE / 2)
        break;
      else if (pointsEdgeRight[i].y > COLSIMAGE - 2)
        counterRight++;
    }
    if (counterLeft > 30 && counterRight > 30)
      return true;
    else
      return false;
  }
};