#include "capture.hpp"
#include "common.hpp"
#include "controlcenter.hpp"
#include "detection/bridge.hpp"
#include "motion.hpp"
#include "preprocess.hpp"
#include "recognition/crossroad.hpp"
#include "recognition/ring.hpp"
#include "recognition/tracking.hpp"
#include "uart.hpp"

using namespace std::literals;

int main() {
  // 赛道元素
  Motion motion("./config/config.json");
  Tracking track;
  Preprocess preprocess("./config/calibration.xml");
  ControlCenter ctrlCent;
  Ring ring;
  Bridge bridge;
  Crossroad crossroad;

  // 场景变量
  Scene scene = Scene::NormalScene;
  Scene sceneLast = Scene::NormalScene;

  uint64_t frameTime = 0;

  // 摄像头和串口
  auto capture = std::make_shared<Capture>(0);
  capture->open();
  auto uart = std::make_shared<Uart>(
      "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0");
  if (uart->open() == -1) {
    return -1;
  }
  uart->startReceive();
  // uart->buzzer = BUZZER_START;
  // uart->carControl(0, PWMSERVOMID);

  while (true) {
    auto FrameStartTime =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    cv::Mat imageBGR = capture->read();
    cv::Mat imageBin = preprocess.binaryzation(imageBGR);

    track.rowCutUp = motion.params.rowCutUp;
    track.rowCutBottom = motion.params.rowCutBottom;
    track.trackRecognition(imageBin);

    if ((scene == Scene::NormalScene || scene == Scene::CrossScene) &&
        motion.params.cross) {
      if (crossroad.crossRecognition(track)) {
        scene = Scene::CrossScene;
      } else {
        scene = Scene::NormalScene;
      }
    }

    //[12] 环岛识别与路径规划
    if ((scene == Scene::NormalScene || scene == Scene::RingScene) &&
        motion.params.ring) {
      if (ring.process(track, imageBin)) {
        scene = Scene::RingScene;
      } else {
        scene = Scene::NormalScene;
      }
    }
    ctrlCent.fitting(track, scene);

    if (ctrlCent.derailmentCheck(track)) {
      uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
      spdlog::critical("Derail!!");
      return -1;
    }

    if (scene == Scene::RingScene) {
      motion.speed = motion.params.speedRing;
    } else {
      motion.speedCtrl(true, false, ctrlCent);
    }

    ctrlCent.drawImage(track, imageBGR);
    motion.poseCtrl(ctrlCent.controlCenter);

    if (motion.servoPwm > PWMSERVOMAX) {
      motion.servoPwm = PWMSERVOMAX;
    }
    if (motion.servoPwm < PWMSERVOMIN) {
      motion.servoPwm = PWMSERVOMIN;
    }
    // spdlog::info("Servo {}", motion.servoPwm);
    uart->carControl(motion.speed, motion.servoPwm);
    // spdlog::info(motion.servoPwm);
    auto FrameEndTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
    frameTime = FrameEndTime - FrameStartTime;

    sceneLast = scene;
    if (scene == Scene::CrossScene) {
      scene = Scene::NormalScene;
    }

    if (uart->keypress) {
      uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
      std::this_thread::sleep_for(1s);
      spdlog::warn("Uart Exit key Pressed, Exit!");
      return -1;
    }

    cv::imshow("Test", imageBGR);
    savePicture(imageBGR);
    cv::waitKey(1);
  }

  return 0;
}