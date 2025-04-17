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
  std::vector<int> roadwidth;
  roadwidth.reserve(ROWSIMAGE);
  std::ifstream inFile("./config/roadwidth.txt");
  if (!inFile.is_open()) {
    spdlog::critical("Failed to Read roadwidth file");
    return -1;
  }
  int value;
  while (inFile >> value) {
    roadwidth.emplace_back(value);
  }
  inFile.close();

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

#if 0
    std::ofstream outFile("./config/roadwidth.txt", std::ios::app);
    if (!outFile.is_open()) {
      spdlog::critical("RoadWidth File Write Failed!");
      return -1;
    }
    for (int row = 0; row < track.pointsEdgeLeft.size(); ++row) {
      int x = track.pointsEdgeRight[row].y - track.pointsEdgeLeft[row].y;
      outFile << x << "\n";
    }
    outFile.close();
#endif

    // 环岛
    if ((scene == Scene::NormalScene || scene == Scene::RingScene) &&
        motion.params.ring) {
      uint8_t ringEnter[10] = {
          motion.params.ringEnter0, motion.params.ringEnter1,
          motion.params.ringEnter2, motion.params.ringEnter3};
      if (ring.ringRecognition(track, motion.params.ringSum, ringEnter,
                               roadwidth)) {
        scene = Scene::RingScene;
      } else {
        scene = Scene::NormalScene;
      }
    }
    // 十字识别
    if ((scene == Scene::NormalScene || scene == Scene::CrossScene) &&
        motion.params.cross) {
      if (crossroad.crossRecognition(track)) {
        scene = Scene::CrossScene;
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

    ctrlCent.drawImage(track, scene, imageBGR);
    float P, D;
    if (scene == Scene::NormalScene || scene == Scene::CrossScene ||
        scene == Scene::ParkingScene || scene == Scene::BridgeScene) {

      P = motion.params.pl * motion.pure_angle * motion.pure_angle / 50 +
          motion.params.ph;
      // if (P > motion.params.pl * 45 * 45 / 50 + motion.params.ph) {
      //   P = motion.params.pl * 45 * 45 / 50 + motion.params.ph;
      // }
      if (P > 8.1) {
        P = 8.1;
      }
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
    // spdlog::info("scene {} | P {}", getScene(scene), P);
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

    cv::imshow("Test", imageBGR);
    savePicture(imageBGR);
    cv::waitKey(1);
  }

  return 0;
}