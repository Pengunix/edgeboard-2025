// crossroad.hpp line 28
    // if (track.pointsEdgeRight.size() < ROWSIMAGE / 2 ||
    // track.pointsEdgeLeft.size() < ROWSIMAGE / 2) // 十字有效行限制
    //     return false;

    // vector<Point2f> pointsToTransform_left =
    // convertPointsToCvPoints(track.pointsEdgeLeft),
    //         pointsToTransform_right =
    //         convertPointsToCvPoints(track.pointsEdgeRight),
    //         transformedPoints;
    // vector<POINT> transformedPoints_Left, transformedPoints_Right;

    // //左边线逆透视
    // perspectiveTransform(pointsToTransform_left, transformedPoints, inv_Mat);
    // transformedPoints_Left = convertCvPointsToPoints(transformedPoints);
    // //右边线逆透视
    // perspectiveTransform(pointsToTransform_right, transformedPoints,
    // inv_Mat); transformedPoints_Right =
    // convertCvPointsToPoints(transformedPoints);

    // transformedPoints_Left = blur_points(transformedPoints_Left,
    // using_kernel_num);   //三角滤波 transformedPoints_Right =
    // blur_points(transformedPoints_Right, using_kernel_num);
    // transformedPoints_Left = resample_points(transformedPoints_Left,
    // using_resample_dist*pixel_per_meter);  //下采样 transformedPoints_Right =
    // resample_points(transformedPoints_Right,
    // using_resample_dist*pixel_per_meter);

    // for(int i=0;i<transformedPoints_Left.size()-3;i += 1)
    // {

    //     int n1_x = transformedPoints_Left[i].x -
    //     transformedPoints_Left[i+1].x; int n1_y = transformedPoints_Left[i].y
    //     - transformedPoints_Left[i+1].y; int n2_x =
    //     transformedPoints_Left[i+2].x - transformedPoints_Left[i+3].x; int
    //     n2_y = transformedPoints_Left[i+2].y - transformedPoints_Left[i+3].y;

    //     double n1_len = sqrt(n1_x*n1_x + n1_x*n1_y);
    //     float n2_len = sqrt(n2_x*n2_x + n2_y*n2_y);
    //     float ang_cos = float(n1_x*n2_x + n1_y*n2_y) / (n1_len * n2_len);
    //     ang_cos = ang_cos>1 ? 1 : ang_cos;
    //     float ang = acos(ang_cos)*180/3.14159;
    //     if(ang>80 && n2_y>0) {left_down_1 = i;break;}
    // }


// crossroad.hpp line 75

    // for(int i=2;i<transformedPoints_Right.size()-3;i += 1)
    // {
    //     int n1_x = transformedPoints_Right[i].x -
    //     transformedPoints_Right[i+1].x; int n1_y =
    //     transformedPoints_Right[i].y - transformedPoints_Right[i+1].y; int
    //     n2_x = transformedPoints_Right[i+2].x -
    //     transformedPoints_Right[i+3].x; int n2_y =
    //     transformedPoints_Right[i+2].y - transformedPoints_Right[i+3].y;
    //     float n1_len = sqrt(n1_x*n1_x + n1_y*n1_y);
    //     float n2_len = sqrt(n2_x*n2_x + n2_y*n2_y);
    //     float ang_cos = float(n1_x*n2_x + n1_y*n2_y) / (n1_len * n2_len);
    //     ang_cos = ang_cos>1 ? 1 : ang_cos;
    //     float ang = acos(ang_cos)*180/3.14159;
    //     ang = abs(ang)>90 ? (180-abs(ang)) : abs(ang);
    //     if(ang>80 && n2_y<0) {right_down_1 = i;break;}
    // }
// crossroad.hpp line 81

      // if(!right_down && right_down_1!=0 &&track.pointsEdgeRight[i - 5].y -
      // track.pointsEdgeRight[i].y <= -2 &&
      //     track.pointsEdgeRight[i - 5].y - track.pointsEdgeRight[i - 10].y <=
      //     -2) right_down = i-10;

// line 117
    // if (track.pointsEdgeLeft[left_up].x >= track.pointsEdgeLeft[left_down].x)
    // left_down = 0; if (track.pointsEdgeRight[right_up].x >=
    // track.pointsEdgeRight[right_down].x) right_down = 0;
    // std::cout << left_up << std::endl;

// line 152
          // if(track.pointsEdgeLeft[left_down].y +
          //         (track.pointsEdgeLeft[left_down].x -
          //         track.pointsEdgeLeft[track.pointsEdgeLeft.size()-1].x) *
          //         (track.pointsEdgeLeft[left_down].y -
          //         track.pointsEdgeLeft[left_begin].y) /
          //         (track.pointsEdgeLeft[left_begin].x -
          //         track.pointsEdgeLeft[left_down].x) >=
          //         track.pointsEdgeRight[track.pointsEdgeRight.size()-1].y)
          //     {left_down=0;}
          // else
          // {
// line 182

          // if(track.pointsEdgeLeft[left_up].y -
          //         (track.pointsEdgeLeft[0].x -
          //         track.pointsEdgeLeft[left_up].x) *
          //         (track.pointsEdgeLeft[left_end].y -
          //         track.pointsEdgeLeft[left_up].y) /
          //         (track.pointsEdgeLeft[left_up].x -
          //         track.pointsEdgeLeft[left_end].x)>=track.pointsEdgeRight[0].y)
          // {
          //     left_up=0;
          // }
          // else
          // {