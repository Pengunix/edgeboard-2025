#pragma once
#include "common.hpp"
#include "controlcenter.hpp"

/**
 * @brief 运动控制器
 *
 */
class Motion {
private:
  int countShift = 0; // 变速计数器

public:
  /**
   * @brief 初始化：加载配置文件
   *
   */
  Motion(const std::string &path) {
    std::ifstream config_is(path);
    if (!config_is.good()) {
      spdlog::critical("[motion] Errro Load Params file.");
      exit(-1);
    }

    nlohmann::json js_value;
    config_is >> js_value;

    try {
      params = js_value.get<Params>();
    } catch (const nlohmann::detail::exception &e) {
      spdlog::critical("[motion] json parse error! {}", e.what());
      exit(-1);
    }

    speed = params.speedLow;
    spdlog::info("[motion] NormP: {} | NormD: {}", params.NormP, params.NormD);
    spdlog::info("[motion] pd_P: {} | pd_D: {}", params.pd_P, params.pd_D);
    spdlog::info("[motion] speedLow: {}  | speedHigh: {}", params.speedLow,
                 params.speedHigh);
  };

  /**
   * @brief 控制器核心参数
   *
   */
  struct Params {
    float speedLow = 0.0;      // 智能车最低速
    float speedHigh = 0.0;     // 智能车最高速
    float speedBridge = 0.0;   // 坡道速度
    float speedCatering = 0.0; // 快餐店速度
    float speedLayby = 0.0;    // 临时停车区速度
    float speedObstacle = 0.0; // 障碍区速度
    float speedParking = 0.0;  // 停车场速度
    float speedRing = 0.0;     // 环岛速度
    float speedDown = 0.0;     // 特殊区域降速速度

    float pd_P = 0.0;
    float pd_D = 0.0;

    float NormP = 0.0;
    float NormD = 0.0;

    float alpha = 0.0;

    uint8_t ringSum = 0;
    uint8_t ringEnter0 = 0;
    float ringP0 = 0.0;
    float ringD0 = 0.0;
    uint8_t ringEnter1 = 0;
    float ringP1 = 0.0;
    float ringD1 = 0.0;
    uint8_t ringEnter2 = 0;
    float ringP2 = 0.0;
    float ringD2 = 0.0;
    uint8_t ringEnter3 = 0;
    float ringP3 = 0.0;
    float ringD3 = 0.0;

    float aim_distance;
    uint16_t track_startline;

    float pl = 0.0;
    float ph = 0.0;

    bool debug = false;        // 调试模式使能
    bool saveImg = false;      // 存图使能
    uint16_t rowCutUp = 0;     // 图像顶部切行
    uint16_t rowCutBottom = 0; // 图像顶部切行
    int parkingStopTime = 100;
    int parkingTurningTime = 21;
    int laybyMoment = 110;
    int laybyStopTime = 200;

    int cateringTurningTime = 50; // 转弯时间 25帧
    int cateringTravelTime = 10; // 行驶时间 10帧 在斜线路段的行驶时间
    int cateringStopTime = 100; // 停车时间

    int Obstacle_disleaving = 0;
    float Obstacle_upscale = 0;
    float Obstacle_block_scale = 0;
    float Obstacle_distance_block = 0;

    int Obstacle_Start_P = 0;
    int Obstacle_Start_D = 0;
    int Obstacle_P1 = 0;
    int Obstacle_D1 = 0;
    int Obstacle_P2 = 0;
    int Obstacle_D2 = 0;

    bool bridge = true;   // 坡道区使能
    bool catering = true; // 快餐店使能
    bool layby = true;    // 临时停车区使能
    bool obstacle = true; // 障碍区使能
    bool parking = true;  // 停车场使能
    bool ring = true;     // 环岛使能
    bool cross = true;    // 十字道路使能
    bool stop = true;     // 停车区使能

    float score = 0.0;                                      // AI检测置信度
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(
        Params, speedLow, speedHigh, speedBridge, speedCatering, speedLayby,
        speedObstacle, speedParking, speedRing, speedDown, pd_P, pd_D, NormP,
        NormD, alpha, ringSum, ringEnter0, ringP0, ringD0, ringEnter1, ringP1,
        ringD1, ringEnter2, ringP2, ringD2, ringEnter3, ringP3, ringD3,
        aim_distance, track_startline, pl, ph, debug, saveImg, rowCutUp,
        rowCutBottom, parkingStopTime, parkingTurningTime, laybyMoment,
        laybyStopTime, cateringTurningTime, cateringTravelTime,
        cateringStopTime, Obstacle_disleaving, Obstacle_upscale,
        Obstacle_block_scale, Obstacle_distance_block,
        Obstacle_Start_P, Obstacle_Start_D, Obstacle_P1, Obstacle_D1,
        Obstacle_P2, Obstacle_D2, bridge, catering, layby, obstacle, parking,
        ring, cross, stop, score 
        ); // 添加构造函数
  };

  Params params;                   // 读取控制参数
  uint16_t servoPwm = PWMSERVOMID; // 发送给舵机的PWM
  float speed = 0.0;               // 发送给电机的速度

  float pure_angle = 0, pure_angleLast = 0;
  /**
   * @brief 姿态PD控制器
   *
   * @param controlCenter 智能车控制中心
   */
  void poseCtrl(const ControlCenter &controlCenter, Scene scene, float P,
                float D, float pd_P, float pd_D) {
    if (scene == Scene::NormalScene || scene == Scene::CrossScene ||
        scene == Scene::ParkingScene || scene == Scene::RingScene) {
      // 纯跟随
      POINT car(ROWSIMAGE - 1, COLSIMAGE / 2 - 5);
      // 计算远锚点偏差值 delta
      float dy = controlCenter.centerEdge[controlCenter.aim_idx].x - car.x;
      float dx = car.y - controlCenter.centerEdge[controlCenter.aim_idx].y;
      float dn = sqrt(dx * dx + dy * dy);
      pure_angle =
          atanf(pixel_per_meter * 2 * params.aim_distance * dx / dn / dn) / PI *
          180;
      // pure_angle -= 1;
      // if(pure_angle>0) pure_angle += 2;//pure_angle *= left_scale;
      // else pure_angle -= 1;//pure_angle *= right_scale;
      // cout << pure_angle << endl;
      // 偏角度闭环
      int pwmDiff = 0;
      pwmDiff = (pure_angle * P) + (pure_angle - pure_angleLast) * D;
      pure_angleLast = pure_angle;

      // 二阶误差闭环
      float pd_error = COLSIMAGE / 2.0 - controlCenter.controlCenter;
      static int pd_errorLast = 0;
      int pd_pwmDiff = 0;
      pd_pwmDiff = (pd_error * pd_P) + (pd_error - pd_errorLast) * pd_D;
      pd_errorLast = pd_error;

      servoPwm = (uint16_t)(PWMSERVOMID + params.alpha * pwmDiff +
                            (1 - params.alpha) * pd_pwmDiff); // PWM转换
      // 舵机限幅
      if (servoPwm > PWMSERVOMAX) {
        servoPwm = PWMSERVOMAX;
      }
      if (servoPwm < PWMSERVOMIN) {
        servoPwm = PWMSERVOMIN;
      }
    } else {
      float error =
          COLSIMAGE / 2 - controlCenter.controlCenter; // 图像控制中心转换偏差
      static int errorLast = 0; // 记录前一次的偏差
      int pwmDiff = 0;
      pwmDiff = (error * P) + (error - errorLast) * D; // 舵机PWM偏移量
      errorLast = error;
      servoPwm = (uint16_t)(PWMSERVOMID + pwmDiff); // PWM转换
                                                    // 舵机限幅
      if (servoPwm > PWMSERVOMAX) {
        servoPwm = PWMSERVOMAX;
      }
      if (servoPwm < PWMSERVOMIN) {
        servoPwm = PWMSERVOMIN;
      }
    }
  }

  /**
   * @brief 变加速控制
   *
   * @param enable 加速使能
   * @param control
   */
  void speedCtrl(bool enable, bool slowDown, ControlCenter control) {
    uint8_t controlLow = 0;   // 速度控制下限
    uint8_t controlMid = 5;   // 控制率
    uint8_t controlHigh = 10; // 速度控制上限

    if (slowDown) {
      countShift = controlLow;
      speed = params.speedDown;
    } else if (enable) // 加速使能
    {
      if (control.centerEdge.size() < 10) {
        speed = params.speedLow;
        countShift = controlLow;
        return;
      }
      if (control.centerEdge[control.centerEdge.size() - 1].x > ROWSIMAGE / 2) {
        speed = params.speedLow;
        countShift = controlLow;
        return;
      }
      if (abs(control.sigmaCenter) < 100.0) {
        countShift++;
        if (countShift > controlHigh)
          countShift = controlHigh;
      } else {
        countShift--;
        if (countShift < controlLow)
          countShift = controlLow;
      }

      if (countShift > controlMid)
        speed = params.speedHigh;
      else
        speed = params.speedLow;
    } else {
      countShift = controlLow;
      speed = params.speedLow;
    }
  }
};
