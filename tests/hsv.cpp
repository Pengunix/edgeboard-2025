#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// 全局变量存储HSV阈值
int H_MIN = 0;
int H_MAX = 180;
int S_MIN = 0;
int S_MAX = 255;
int V_MIN = 0;
int V_MAX = 255;

// 窗口名称
const string windowName = "HSV Masking Tool";
const string controlsWindow = "HSV Controls";

// 滑动条回调函数（空函数，仅用于更新值）
void onTrackbar(int, void *) {}

int main() {
  // 创建控制窗口
  namedWindow(controlsWindow, WINDOW_AUTOSIZE);

  // 创建HSV阈值滑动条
  createTrackbar("H_MIN", controlsWindow, &H_MIN, 180, onTrackbar);
  createTrackbar("H_MAX", controlsWindow, &H_MAX, 180, onTrackbar);
  createTrackbar("S_MIN", controlsWindow, &S_MIN, 255, onTrackbar);
  createTrackbar("S_MAX", controlsWindow, &S_MAX, 255, onTrackbar);
  createTrackbar("V_MIN", controlsWindow, &V_MIN, 255, onTrackbar);
  createTrackbar("V_MAX", controlsWindow, &V_MAX, 255, onTrackbar);

  // 打开摄像头
  VideoCapture cap(0);
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
  if (!cap.isOpened()) {
    cerr << "无法打开摄像头" << endl;
    return -1;
  }

  Mat frame, hsv, mask, result;

  while (true) {
    cap >> frame;
    if (frame.empty())
      break;

    // 转换为HSV颜色空间
    cvtColor(frame, hsv, COLOR_BGR2HSV);

    // 根据滑动条值创建HSV范围掩码
    inRange(hsv, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX),
            mask);

    // 应用掩码到原始图像
    frame.copyTo(result, mask);

    // 显示结果
    imshow("Original", frame);
    imshow("HSV Mask", mask);
    imshow("Result", result);

    // 按ESC退出
    if (waitKey(100) == 27)
      break;
  }

  cap.release();
  destroyAllWindows();
  return 0;
}