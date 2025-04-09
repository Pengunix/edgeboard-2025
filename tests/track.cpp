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
  std::vector<int> roadwidth{
      250, 249, 248, 247, 246, 246, 244, 244, 242, 242, 240, 240, 239, 238, 237,
      236, 235, 234, 233, 232, 231, 230, 229, 229, 227, 227, 225, 225, 223, 223,
      221, 221, 219, 219, 218, 217, 216, 215, 214, 213, 212, 211, 210, 210, 208,
      208, 206, 206, 204, 204, 202, 202, 200, 200, 199, 198, 197, 196, 195, 193,
      193, 191, 191, 190, 189, 188, 187, 186, 186, 184, 184, 182, 182, 180, 180,
      178, 178, 177, 176, 175, 174, 173, 172, 172, 170, 170, 169, 168, 167, 166,
      165, 164, 163, 161, 161, 160, 159, 158, 157, 156, 155, 155, 153, 153, 151,
      151, 149, 149, 147, 147, 146, 145, 144, 143, 142, 142, 140, 140, 138, 138,
      137, 136, 135, 134, 133, 132, 131, 130, 130, 129, 128, 127, 126, 125, 125,
      123, 123, 121, 121, 119, 119, 118, 117, 116, 115, 114, 113, 113, 111, 111,
      109, 109, 109, 107, 107, 106, 105, 104, 103, 102, 102, 100, 100, 98,  98,
      97,  96,  95,  94,  93,  92,  91,  90,  90,  89,  88,  87,  86,  86,  84};

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

    // 十字识别
    if ((scene == Scene::NormalScene || scene == Scene::CrossScene) &&
        motion.params.cross) {
      if (crossroad.crossRecognition(track)) {
        scene = Scene::CrossScene;
      } else {
        scene = Scene::NormalScene;
      }
    }

    // 环岛
    if ((scene == Scene::NormalScene || scene == Scene::RingScene) &&
        motion.params.ring) {
      uint8_t ringEnter[10] = {20, 20, 32, 0};
      if (ring.ringRecognition(track, 2, ringEnter, roadwidth)) {
        scene = Scene::RingScene;
      } else {
        scene = Scene::NormalScene;
      }
    }
    ctrlCent.fitting(track, scene, motion.params.aim_distance,
                     motion.params.track_startline, ring.ringStep, ring.R100,
                     ring.ringType);

    if (ctrlCent.derailmentCheck(track)) {
      uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
      spdlog::critical("Derail!!");
      return -1;
    }

    if (scene == Scene::RingScene) {
      motion.speed = motion.params.speedRing;
    } else {
      // TODO(me): 速度控制参数修改
      motion.speedCtrl(true, false, ctrlCent);
    }

    ctrlCent.drawImage(track, imageBGR);
    float P, D;
    if (scene == Scene::NormalScene || scene == Scene::CrossScene ||
        scene == Scene::ParkingScene || scene == Scene::BridgeScene) {

      P = motion.params.pl * motion.pure_angle * motion.pure_angle / 50 +
          motion.params.ph;
      // if(P>motion.params.pl*45*45/50 + motion.params.ph)
      // P=motion.params.pl*45*45/50 + motion.params.ph;
      if (P > 8.3)
        P = 8.3;
      D = motion.params.NormD;
    } else if (scene == Scene::RingScene) {
      if (ring.ringStep == RingStep::Waiting ||
          ring.ringStep == RingStep::Verifing) {
        P = motion.params.pl * motion.pure_angle * motion.pure_angle / 50 +
            motion.params.ph;
        if (P > 9)
          P = 9;
        D = motion.params.NormD;
      } else {
        if (ring.ringNum == 0) {
          P = motion.params.ringP0;
          D = motion.params.ringD0;
        } // R60
        else if (ring.ringNum == 1) {
          P = motion.params.ringP1;
          D = motion.params.ringD1;
        } // R50
      }
    }
    spdlog::info("P {}", P);
    motion.poseCtrl(ctrlCent, scene, P, D, motion.params.pd_P,
                    motion.params.pd_D);

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
    // spdlog::info("Frame time {} ", frameTime);
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

    // :imshow("Test", imageBGR);
    // savePicture(imageBGR);
    // cv::waitKey(1);
  }

  return 0;
}