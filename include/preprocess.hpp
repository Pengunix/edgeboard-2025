#pragma once
#include "common.hpp"
/**
**[1] 读取视频
**[2] 图像二值化
*/
class Preprocess {
public:
  /**
   * @brief 图像矫正参数初始化
   *
   */
  Preprocess(const std::string &path) {
    // 读取xml中的相机标定参数
    cameraMatrix =
        cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); // 摄像机内参矩阵
    distCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0)); // 相机的畸变矩阵
    cv::FileStorage file;
    if (file.open(path, cv::FileStorage::READ)) {
      file["camera-atrix"] >> cameraMatrix;
      file["distortion"] >> distCoeffs;
      spdlog::info("[preprocess] 相机参数加载成功");
      enable = true;
    } else {
      spdlog::warn("[preprocess] 相机参数加载失败");
      enable = false;
    }
  };

  /**
   * @brief 图像二值化
   *
   * @param frame	输入原始帧
   * @return cv::Mat	二值化图像
   */
  cv::Mat binaryzation(cv::Mat &frame) {
    cv::Mat imageGray, imageBinary;

    cv::cvtColor(frame, imageGray, cv::COLOR_BGR2GRAY); // RGB转灰度图

    cv::threshold(imageGray, imageBinary, 0, 255,
                  cv::THRESH_OTSU); // OTSU二值化方法

    return imageBinary;
  }

  /**
   * @brief 矫正图像
   *
   * @param imagesPath 图像路径
   */
  cv::Mat correction(cv::Mat &image) {
    if (enable) {
      cv::Size sizeImage; // 图像的尺寸
      sizeImage.width = image.cols;
      sizeImage.height = image.rows;

      cv::Mat mapx =
          cv::Mat(sizeImage, CV_32FC1); // 经过矫正后的X坐标重映射参数
      cv::Mat mapy =
          cv::Mat(sizeImage, CV_32FC1); // 经过矫正后的Y坐标重映射参数
      cv::Mat rotMatrix =
          cv::Mat::eye(3, 3, CV_32F); // 内参矩阵与畸变矩阵之间的旋转矩阵

      // 采用initUndistortRectifyMap+remap进行图像矫正
      initUndistortRectifyMap(cameraMatrix, distCoeffs, rotMatrix, cameraMatrix,
                              sizeImage, CV_32FC1, mapx, mapy);
      cv::Mat imageCorrect = image.clone();
      remap(image, imageCorrect, mapx, mapy, cv::INTER_LINEAR);

      // 采用undistort进行图像矫正
      //  undistort(image, imageCorrect, cameracv::Matrix, distCoeffs);

      return imageCorrect;
    } else {
      return image;
    }
  }
  cv::Mat getCameraMatrix() {
    return cameraMatrix;
  }

private:
  bool enable = false;  // 图像矫正使能：初始化完成
  cv::Mat cameraMatrix; // 摄像机内参矩阵
  cv::Mat distCoeffs;   // 相机的畸变矩阵
};
