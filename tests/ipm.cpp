#include "common.hpp"
#include "capture.hpp"

int main() {
  auto cap = std::make_unique<Capture>(0);
  cap->open();

  for(;;) {
    cv::Mat img = cap->read();
    cv::Mat img_ipm;
    // cv::perspectiveTransform(img, img_ipm, init_ipm_mat());
    cv::warpPerspective(img, img_ipm, auto_init_ipm_mat(), cv::Size(320, 240));
    cv::imshow("orin", img);
    cv::imshow("ipm_test", img_ipm);
    cv::waitKey(10);
  }
}