#include "capture.hpp"
#include "common.hpp"
#include "controlcenter.hpp"
#include "detection.hpp"
#include "detection/bridge.hpp"
#include "detection/catering.hpp"
#include "detection/crosswalk.hpp"
#include "detection/layby.hpp"
#include "detection/obstacle.hpp"
#include "detection/parking.hpp"
#include "motion.hpp"
#include "preprocess.hpp"
#include "recognition/crossroad.hpp"
#include "recognition/ring.hpp"
#include "recognition/tracking.hpp"
#include "uart.hpp"
using namespace std::literals;
/*
  1. 逆透视矩阵重测
  2. 路宽数组重测
*/
long preTime, frameTime = 0;
long long global_counter = 0;

int main(int argc, char const *argv[]) {
  Preprocess preprocess;                 // 图像预处理类
  Motion motion("./config/config.json"); // 运动控制类
  Tracking tracking;
  Crossroad crossroad;
  Ring ring;
  Bridge bridge;
  Catering catering(motion.params.cateringTravelTime,
                    motion.params.cateringTurningTime,
                    motion.params.cateringStopTime);
  Obstacle obstacle;
  Layby layby(motion.params.laybyMoment, motion.params.laybyStopTime);
  Parking parking(motion.params.parkingTurningTime,
                  motion.params.parkingStopTime);
  StopArea stopArea;
  ControlCenter ctrlCenter;

  // 路宽读取
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

  int countInit = 0;
  // AI异步推理
  std::shared_ptr<Detection> detection =
      std::make_shared<Detection>("./model/yolov3_mobilenet_v1");
  detection->score = motion.params.score;
  std::future<void> future;
  std::vector<PredictResult> AIresults;
  bool AIFlag = false;

  // 摄像头和串口
  std::shared_ptr<Capture> cap = std::make_shared<Capture>(0);
  cap->open();
  std::shared_ptr<Uart> uart = std::make_shared<Uart>("/dev/ttyUSB0");
  uart->open();

  if (!cap->is_open() || !uart->isOpen) {
    spdlog::critical("Capture or Uart Open FAILED!");
    return -1;
  }
  // 串口接收
  uart->buzzer = BUZZER_START;
  uart->carControl(0, PWMSERVOMID);
  uart->startReceive();

  // TODO(me) Debug 模式重写
  if (motion.params.debug) {
  }

  // 等待按键发车
  // if (!motion.params.debug) {
  //   spdlog::info("初始化完毕，等待按键发车");
  //   uart->buzzer = BUZZER_OK;
  //   // TODO(me) 选一下判断，判断特定按键
  //   while (!uart->keypress) {
  //     cv::waitKey(300);
  //   }
  //   // 延时3s发车
  //   for (int i = 0; i < 10; ++i) {
  //     uart->carControl(0, PWMSERVOMID);
  //     cv::waitKey(300);
  //   }
  //   uart->buzzer = BUZZER_START;
  //   uart->carControl(0, PWMSERVOMID);
  // }

  // 初始化参数
  Scene scene = Scene::NormalScene;
  Scene sceneLast = Scene::NormalScene;

  cv::Mat img;

  std::this_thread::sleep_for(1s); // 等待摄像头稳定
  while (1) {
    // TODO(me) 重写Debug
    global_counter++;
    // if (global_counter > 0 && global_counter < 0+30) {
    //   scene = Scene::NormalScene;
    // }
    if (motion.params.debug) {
      preTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count();
    }

    // [01] 获取图像
    img = cap->read();
    // [02] 图像预处理
    cv::Mat imgBinary = preprocess.binaryzation(img);

    // [03] 启动AI推理
    cv::Mat AIimg = img.clone();
    if (!future.valid() &&
        (scene == Scene::NormalScene || scene == Scene::CrossScene ||
         (scene == Scene::RingScene && (ring.ringStep == RingStep::Verifing ||
                                        ring.ringStep == RingStep::Waiting))))
      future = std::async(
          std::launch::async,
          [&detection](const cv::Mat image) { detection->inference(image); },
          AIimg);
    if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready &&
        (scene == Scene::NormalScene || scene == Scene::CrossScene ||
         (scene == Scene::RingScene && (ring.ringStep == RingStep::Verifing ||
                                        ring.ringStep == RingStep::Waiting)))) {
      // 如果异步任务已经完成
      // 获取结果并处理
      AIresults = detection->results;
      AIFlag = true;
      future = std::async(
          std::launch::async,
          [&detection](const cv::Mat image) { detection->inference(image); },
          AIimg);
    } else {
      AIresults.clear();
      AIFlag = false;
    }

    // [04] 赛道识别
    // 图像顶部切行（前瞻距离）
    tracking.rowCutUp = motion.params.rowCutUp;
    tracking.rowCutBottom = motion.params.rowCutBottom;
    // 图像底部切行（盲区距离）
    tracking.trackRecognition(imgBinary);

    // std::ofstream outFile("./config/roadwidth.txt");
    // if (!outFile.is_open()) {
    //   spdlog::critical("Failed to Open roadwidth file");
    //   return -1;
    // }
    // for (int row = 0; row < tracking.pointsEdgeLeft.size(); ++row) {
    //   int x = tracking.pointsEdgeRight[row].y -
    //   tracking.pointsEdgeLeft[row].y; outFile << x << std::endl;
    // }
    // outFile.close();

    if (motion.params.debug) {
      // cv::Mat imgTrack = img.clone();
      // 图像绘制赛道识别结果
      // tracking.drawImage(imgTrack);
      // display.setNewWindow(2, "Track", imgTrack);
      // cv::imshow("Track", imgTrack);
    }

    // [05] 停车区检测
    if (motion.params.stop) {
      if (stopArea.process(detection->results)) {
        scene = Scene::StopScene;
        if (stopArea.countExit > 20) {
          // 停车
          uart->buzzer = BUZZER_FINISH;
          uart->carControl(0, PWMSERVOMID);
          std::this_thread::sleep_for(1s);
          spdlog::critical("-----> System Exit!!! <-----");
          exit(0); // 程序退出
        }
      }
    }

    // [06] 快餐店检测
    if ((scene == Scene::NormalScene || scene == Scene::CateringScene) &&
        motion.params.catering) {
      if (catering.process(tracking, imgBinary, detection->results)) {
        scene = Scene::CateringScene;
      } else
        scene = Scene::NormalScene;
    }

    // [07] 临时停车区检测
    if ((scene == Scene::NormalScene || scene == Scene::LaybyScene) &&
        motion.params.layby) {
      if (layby.process(tracking, imgBinary, detection->results, roadwidth))
        scene = Scene::LaybyScene;
      else
        scene = Scene::NormalScene;
    }

    // [08] 充电停车场检测
    if ((scene == Scene::NormalScene || scene == Scene::ParkingScene) &&
        motion.params.parking) {
      if (parking.process(tracking, imgBinary, detection->results)) {
        scene = Scene::ParkingScene;
        // parking.drawImage(tracking, img);
      } else {
        scene = Scene::NormalScene;
      }
    }

    // [09] 坡道区检测
    if ((scene == Scene::NormalScene || scene == Scene::BridgeScene) &&
        motion.params.bridge) {
      if (bridge.process(tracking, detection->results)) {
        // if (bridge.process(tracking, frameTime, motion.speed, roadwidth,
        //  ctrlCenter.left_num, ctrlCenter.right_num)) {
        scene = Scene::BridgeScene;
      } else {
        scene = Scene::NormalScene;
      }
    }

    // [10] 障碍区检测
    if ((scene == Scene::NormalScene || scene == Scene::ObstacleScene) &&
        motion.params.obstacle) {
      if (obstacle.process(img, tracking, detection, frameTime, motion.speed, 20,
                           roadwidth, motion.params.Obstacle_upscale,
                           motion.params.Obstacle_block_scale,
                           motion.params.Obstacle_distance_block, 0)) {
        obstacle.drawImage(img);
        scene = Scene::ObstacleScene;
      } else
        scene = Scene::NormalScene;
    }

    // [11] 环岛识别与路径规划
    if ((scene == Scene::NormalScene || scene == Scene::RingScene) &&
        motion.params.ring) {
      uint8_t ringEnter[10] = {
          motion.params.ringEnter0, motion.params.ringEnter1,
          motion.params.ringEnter2, motion.params.ringEnter3};
      if (ring.ringRecognition(tracking, motion.params.ringSum, ringEnter,
                               roadwidth)) {
        ring.drawimg(img);
        scene = Scene::RingScene;
      } else {
        scene = Scene::NormalScene;
      }
    }
    // [12] 十字道路识别与路径规划
    if ((scene == Scene::NormalScene || scene == Scene::CrossScene) &&
        motion.params.cross) {
      if (crossroad.crossRecognition(tracking))
        scene = Scene::CrossScene;
      else
        scene = Scene::NormalScene;
    }
    // [13] 车辆控制中心拟合
    ctrlCenter.fitting(tracking, scene, motion.params.aim_distance,
                       motion.params.track_startline, ring.ringStep, ring.R100,
                       ring.ringType, catering.burgerLeft);
    // [14] 出赛道检测
    if (scene != Scene::ParkingScene) {
      if (ctrlCenter.derailmentCheck(tracking)) {
        uart->carControl(0, PWMSERVOMID);
        std::this_thread::sleep_for(1s);
        spdlog::critical("-----> Derail !!! <-----");
        exit(0);
      }
    }

    // [15] 运动控制(速度+方向)
    // 去除调试模式判断
    if (countInit > 30) {
      // 触发停车
      if ((catering.stopEnable && scene == Scene::CateringScene) ||
          (layby.stopEnable && scene == Scene::LaybyScene) ||
          (parking.step == parking.ParkStep::stop)) {
        motion.speed = 0;
        spdlog::info("停车");
      } else if (scene == Scene::CateringScene)
        motion.speed = motion.params.speedCatering;
      else if (scene == Scene::LaybyScene)
        motion.speed = motion.params.speedLayby;
      else if (scene == Scene::ParkingScene &&
               parking.step == parking.ParkStep::trackout) {
        motion.speed = -motion.params.speedParking;
        spdlog::info("倒车");
      } else if (scene == Scene::ParkingScene) // 减速
        motion.speed = motion.params.speedParking;
      else if (scene == Scene::BridgeScene) // 坡道速度
        motion.speed = motion.params.speedBridge;
      else if (scene == Scene::ObstacleScene) // 危险区速度
        motion.speed = motion.params.speedObstacle;
      else if (scene == Scene::RingScene) // 环岛速度
        motion.speed = motion.params.speedRing;
      else if (scene == Scene::StopScene)
        motion.speed = motion.params.speedDown;
      else
        motion.speedCtrl(true, false, ctrlCenter); // 车速控制

    } else {
      countInit++;
    }

    // 舵机动态PD控制
    float P = 0, D = 0;
    if (scene == Scene::NormalScene || scene == Scene::CrossScene ||
        scene == Scene::ParkingScene || scene == Scene::BridgeScene ) {

      P = motion.params.pl * motion.pure_angle * motion.pure_angle / 50 +
          motion.params.ph;
      // if (P > motion.params.pl * 45 * 45 / 50 + motion.params.ph) {
      //   P = motion.params.pl * 45 * 45 / 50 + motion.params.ph;
      // }
      // ! 这里有个限幅，调参时注意
      if (P > 8) {
        P = 8;
      }
      D = motion.params.NormD;
    } else if (scene == Scene::RingScene) {
      if (ring.ringStep == RingStep::Waiting ||
          ring.ringStep == RingStep::Verifing) {
        P = motion.params.pl * motion.pure_angle * motion.pure_angle / 50 +
            motion.params.ph;
        if (P > 11)
          P = 11;
        D = motion.params.NormD;
      } else {
        if (ring.ringNum == 0) {
          P = motion.params.ringP0;
          D = motion.params.ringD0;
        } else if (ring.ringNum == 1) {
          P = motion.params.ringP1;
          D = motion.params.ringD1;
        }
      }
    } else if (scene == Scene::ObstacleScene) {
      if (obstacle.step == Obstacle::Step::Start && obstacle.first == 0) {
        P = motion.params.Obstacle_Start_P;
        D = motion.params.Obstacle_Start_D;
      } else if (obstacle.step == Obstacle::Step::Start ||
                 obstacle.step == Obstacle::Step::Inside) {
        P = motion.params.Obstacle_P1;
        D = motion.params.Obstacle_D1;
      } else {
        P = motion.params.Obstacle_P2;
        D = motion.params.Obstacle_D2;
      }
    } else if (scene == Scene::LaybyScene) {
      P = 1;
      D = 10;
    } else if (scene == Scene::CateringScene) {
      P = 4.5;
      D = 10;
    }
    motion.poseCtrl(ctrlCenter, scene, P, D, motion.params.pd_P,
                    motion.params.pd_D);
    uart->carControl(motion.speed, motion.servoPwm);

    //[16] 综合显示调试UI窗口
    if (motion.params.debug) {
      // 帧率计算
      auto startTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::system_clock::now().time_since_epoch())
                           .count();
      frameTime = startTime - preTime;
      if (motion.params.debug) {
        detection->drawBox(img);
        ctrlCenter.drawImage(tracking, scene, img);
        putText(img, formatDouble2String(1000.0 / frameTime, 1) + "FPS",
                cv::Point(COLSIMAGE - 75, 80), cv::FONT_HERSHEY_PLAIN, 1,
                cv::Scalar(0, 0, 255), 1);

        putText(img,
                "P:" + formatDouble2String(P, 1) +
                    " D:" + formatDouble2String(D, 1),
                cv::Point(COLSIMAGE - 120, 95), cv::FONT_HERSHEY_PLAIN, 1,
                cv::Scalar(0, 0, 255), 1);
        putText(img, "Servo:" + std::to_string(motion.servoPwm),
                cv::Point(COLSIMAGE - 120, 110), cv::FONT_HERSHEY_PLAIN, 1,
                cv::Scalar(0, 0, 255), 1);
        cv::imshow("aa", img);
        cv::waitKey(1);
      }
    }
    if (motion.params.saveImg) {
      savePicture(img);
    }

    //[17] 状态复位
    if (sceneLast != scene) {
      spdlog::info("Scene Change: {} -> {}, global counter {}", getSceneName(sceneLast), getSceneName(scene), global_counter);
      if (scene == Scene::NormalScene)
        uart->buzzer = BUZZER_SLIENT;
      else
        uart->buzzer = BUZZER_SLIENT;
    }
    sceneLast = scene; // 记录当前状态
    if (scene == Scene::ObstacleScene)
      scene = Scene::NormalScene;
    else if (scene == Scene::CrossScene)
      scene = Scene::NormalScene;
    else if (scene == Scene::RingScene)
      scene = Scene::NormalScene;
    else if (scene == Scene::CateringScene)
      scene = Scene::NormalScene;
    else if (scene == Scene::LaybyScene)
      scene = Scene::NormalScene;
    else if (scene == Scene::ParkingScene)
      scene = Scene::NormalScene;
    else if (scene == Scene::StopScene)
      scene = Scene::NormalScene;

    // [17] 按键退出程序
    if (uart->keypress) {
      uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
      std::this_thread::sleep_for(1s);
      spdlog::critical("-----> System Exit!!! <-----");
      exit(0);
    }
  }

  uart->close(); // 串口通信关闭
  cap->close();
  return 0;
}
