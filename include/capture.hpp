#pragma once
#include "common.hpp"

class Capture {
private:
  std::shared_ptr<cv::VideoCapture> _capture;
  std::string file_path;
  int dev_index;
  bool _isOpend = false;
  bool _isFile = false;

public:
  Capture(int index) : dev_index(index) { _isFile = false; };
  Capture(std::string path) : file_path(path) { _isFile = true; };
  int open() {
    if (_isFile) {
      _capture = std::make_shared<cv::VideoCapture>(file_path);
    } else {
      _capture = std::make_shared<cv::VideoCapture>(dev_index, cv::CAP_V4L2);
    }
    return _open();
  }
  int open(int dev_index) {
    _capture = std::make_shared<cv::VideoCapture>(dev_index, cv::CAP_V4L2);
    return _open();
  }
  int open(std::string path) {
    _capture = std::make_shared<cv::VideoCapture>(path);
    return _open();
  };

  bool is_open() { return _isOpend; }
  cv::Mat read() {
    cv::Mat frame;
    if (is_open()) {
      _capture->read(frame);
    }
    return frame;
  };
  void close() {
    _isOpend = false;
    _capture->release();
  }

  Capture() = default;
  ~Capture() = default;

private:
  int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  int _open() {
    if ((_capture == nullptr) || (!_capture->isOpened())) {
      spdlog::critical("Video Capture create failed. ");
      return -1;
    }
    //修改摄像头数据源分辨率
    _capture->set(cv::CAP_PROP_FRAME_WIDTH, COLSIMAGE);
    _capture->set(cv::CAP_PROP_FRAME_HEIGHT, ROWSIMAGE);
    _capture->set(cv::CAP_PROP_FPS, 120);
    _capture->set(cv::CAP_PROP_FOURCC, codec);
    _isOpend = true;
#if 0
    double rate = _capture->get(cv::CAP_PROP_FPS);
    double width = _capture->get(cv::CAP_PROP_FRAME_WIDTH);
    double height = _capture->get(cv::CAP_PROP_FRAME_HEIGHT);
    spdlog::info("Camera Param: Frame rate =", rate, " width = ", width, " height = " , height);
#endif
    return 0;
  }
};
