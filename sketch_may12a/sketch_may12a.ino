#include <RPLidarC1.h>
#include <ESP32Servo.h>

// ========== 硬件引脚定义（保持不变） ==========
const int SERVO_PIN = 10; // 舵机引脚
const int DIR_PIN = 46;   // 电机方向引脚
const int MOTOR_PIN = 21; // 电机速度引脚

RPLidar lidar; // 创建激光雷达对象
Servo servo;   // 创建舵机对象

// ========== 算法参数（与Python仿真一致，单位统一为米） ==========
#define MAX_RANGE 2.0         // 最大有效距离 (m) (100cm)
#define MAX_SPEED 40          // 最大PWM值 (0-255) 比赛限速
#define FRONT_THRESHOLD 0.8   // 前方避障阈值 (m) (60cm)
#define SIDE_THRESHOLD 0.4    // 侧边平衡阈值 (m) (30cm)
#define SIDE_FORCE_WEIGHT 0.5 // 侧边力权重
#define TURN_GAIN 1.0         // 转向增益（将目标角度差映射到舵机角度范围）
#define MAX_STOP_DIST 0.03

// 舵机角度映射参数
#define STEERING_CENTER 90
#define STEERING_MAX_LEFT 60
#define STEERING_MAX_RIGHT 120
#define LIDAR_OFFSET 0 // 如果雷达0度不朝前，修改此偏移量（度）
#define MIN_SPEED 15   // 最小启动PWM，防止电机卡死

int servo_offset = 3; // 用于调节舵机安装误差
int speed = 0;        // 当前电机PWM值
int servo_angle = 90; // 目标舵机角度

float distances[360] = {0}; // 存储每个角度的最近距离（单位：米）


void computeControl(float* distances, int &out_speed, int &out_servo_angle) {
  float repel_x = 0, repel_y = 0;
  int count = 0;

  // 1. 遍历雷达数据，计算“平均排斥力”
  for (int angle = 0; angle < 360; angle++) {
    float dist = distances[angle];
    if (dist <= 0.05f || dist > MAX_RANGE) continue;

    float rel_angle = (angle > 180) ? (angle - 360) : angle;
    float rad = radians(rel_angle);

    // 只处理前方 180 度范围内的障碍物
    if (abs(rel_angle) <= 90) {
      float threshold = (abs(rel_angle) <= 45) ? FRONT_THRESHOLD : SIDE_THRESHOLD;
      if (dist < threshold) {
        float weight = (threshold - dist) / threshold;
        // 累加排斥力向量 (方向与障碍物相反)
        repel_x += weight * cos(rad + M_PI);
        repel_y += weight * sin(rad + M_PI);
        count++;
      }
    }
  }

  // 2. 归一化：避免障碍物点数越多力越大的问题
  if (count > 0) {
    repel_x /= count;
    repel_y /= count;
  }

  // 3. 转向逻辑：将排斥力叠加到“前进趋势”上
  // 增大驱动力的权重(1.5)，确保 x 轴方向永远向上，防止原地打转或倒车
  float drive_bias = 1.5f; 
  float target_x = drive_bias + repel_x; 
  float target_y = repel_y * 2.5f; // 放大 y 轴感度，让转向更灵敏

  float target_rad = atan2(target_y, target_x);
  float target_deg = degrees(target_rad); // 得到 -180 到 180 的度数

  // 映射转向：假设 target_deg 平滑地在 -45 到 45 之间
  // 修正后的映射不再是只有极值，而是线性的
  int final_servo = STEERING_CENTER + (int)constrain(target_deg, -30, 30);

  // 4. 速度逻辑：不再受侧边力干扰，只看正前方的阻碍
  // 检查正前方 30 度内是否有极近障碍物
  float front_dist = 2.0f;
  for(int a = -15; a <= 15; a++) {
    int idx = (a + 360) % 360;
    if (distances[idx] > 0.05f && distances[idx] < front_dist) front_dist = distances[idx];
  }

  int final_speed = MAX_SPEED;
  if (front_dist < FRONT_THRESHOLD) {
    // 速度随距离线性下降，最低保留 30% 的速度用于蠕动，除非距离小于 15cm
    float slow_factor = (front_dist - 0.15f) / (FRONT_THRESHOLD - 0.15f);
    final_speed = MAX_SPEED * constrain(slow_factor, 0.3f, 1.0f);
    if (front_dist < 0.15f) final_speed = 0; // 太近了，停下
  }

  out_speed = final_speed;
  out_servo_angle = final_servo;
}
// 



// void computeControl(float* distances, int &out_speed, int &out_servo_angle) {
//   float f_front_x = 0, f_front_y = 0;
//   float f_side_x   = 0, f_side_y   = 0;

//   // 1. 基础驱动力：给一个恒定的向前推力
//   float drive_force_x = 0.2;
//   float drive_force_y = 0.0;

//   for (int i = 0; i < 360; i++) {
//     // 处理雷达安装偏移，计算逻辑角度
//     int angle = (i + LIDAR_OFFSET + 360) % 360;
//     float dist = distances[i];

//     if (dist <= 0.05f || dist > MAX_RANGE) continue;

//     // 将 0-359 映射到 -180 到 179，方便判断左右
//     // 映射后：0为正前，-90为右侧，90为左侧（假设逆时针为正）
//     float rel_angle = (angle > 180) ? (angle - 360) : angle;
//     float rel_angle_rad = radians(rel_angle);

//     // ===== 1. 前方扇区 (正前左右各45度，共90度) =====
//     if (abs(rel_angle) <= 45) {
//       if (dist < FRONT_THRESHOLD) {
//         float mag = (FRONT_THRESHOLD - dist) / FRONT_THRESHOLD;
//         // 排斥力：背离障碍物方向
//         f_front_x += mag * cos(rel_angle_rad + M_PI);
//         f_front_y += mag * sin(rel_angle_rad + M_PI);
//       }
//     }
//     // ===== 2. 侧方扇区 (左右45到100度) =====
//     else if (abs(rel_angle) > 45 && abs(rel_angle) <= 100) {
//       if (dist < SIDE_THRESHOLD) {
//         float mag = ((SIDE_THRESHOLD - dist) / SIDE_THRESHOLD) * SIDE_FORCE_WEIGHT;
//         f_side_x += mag * cos(rel_angle_rad + M_PI);
//         f_side_y += mag * sin(rel_angle_rad + M_PI);
//       }
//     }
//   }

//   // 最终合力 = 驱动力 + 前方排斥 + 侧方排斥
//   float total_x = drive_force_x + f_front_x + f_side_x;
//   float total_y = drive_force_y + f_front_y + f_side_y;

//   // 计算目标转向角
//   float target_angle = atan2(total_y, total_x);

//   // 映射转向：注意 angle_diff 的限幅
//   float angle_diff = constrain(target_angle, -radians(30), radians(30));
//   float target_turn_norm = angle_diff / radians(30);

//   // 映射到舵机角度
//   int target_servo_angle = STEERING_CENTER + (int)(target_turn_norm * 30); // 30为摆幅

//   // ===== 速度逻辑优化 =====
//   // 只有真正前方的力 (f_front) 才显著降低速度，避免侧边墙壁导致停车
//   float front_obstruction = sqrt(f_front_x * f_front_x + f_front_y * f_front_y);
//   int target_speed = MAX_SPEED * (1.0f - front_obstruction * 0.8f);

//   // 确保最慢也会爬行，除非障碍物极近
//   if (target_speed < MIN_SPEED) target_speed = MIN_SPEED;

//   out_speed = target_speed;
//   out_servo_angle = target_servo_angle;
// }

// ========== 核心避障算法（车体相对坐标系） ==========
// void computeControl(float* distances, int &out_speed, int &out_servo_angle) {
//   float f_front_x = 0, f_front_y = 0;   // 前方避障合力
//   float f_side_x   = 0, f_side_y   = 0; // 侧方平衡合力

//   // 前进的基础驱动力（沿车头正前方，即角度0°）
//   f_side_x += 0.1;   // 力的大小0.1，方向沿x轴正方向
//   f_side_y += 0.0;

//   // 遍历360个角度，计算每个障碍物贡献的力
//   for (int angle = 0; angle < 360; angle++) {
//     float dist = distances[angle];
//     // 过滤无效数据（距离小于0.05m视为死区，大于MAX_RANGE忽略）
//     if (dist <= 0.05f || dist > MAX_RANGE) continue;

//     // 将角度转换为弧度，并定义0°为正前方，逆时针为正
//     float rel_angle_rad = radians(angle);

//     // ===== 1. 前方45°范围紧急避障 =====
//     if (abs(rel_angle_rad) <= radians(45)) {
//       if (dist < FRONT_THRESHOLD) {
//         float mag = (FRONT_THRESHOLD - dist) / FRONT_THRESHOLD;
//         // 排斥力方向：从障碍物指向机器人（即障碍物方向的反向）
//         f_front_x += mag * cos(rel_angle_rad + M_PI);
//         f_front_y += mag * sin(rel_angle_rad + M_PI);
//       }
//     }
//     // ===== 2. 侧方45°~90°范围边缘平衡 =====
//     else if (abs(rel_angle_rad) > radians(45) && abs(rel_angle_rad) <= radians(90)) {
//       if (dist < SIDE_THRESHOLD) {
//         float mag = ((SIDE_THRESHOLD - dist) / SIDE_THRESHOLD) * SIDE_FORCE_WEIGHT;
//         f_side_x += mag * cos(rel_angle_rad + M_PI);
//         f_side_y += mag * sin(rel_angle_rad + M_PI);
//       }
//     }
//   }

// 合力

// float combined_mag = sqrt(total_x * total_x + total_y * total_y);

//   // 默认速度和转向
//   int target_speed = MAX_SPEED;
//   float target_turn_norm = 0;   // 归一化转向值 [-1,1]

//   if (combined_mag > 0.01f) {
//     // 目标方向（车体坐标系下）
//     float target_angle = atan2(total_y, total_x);
//     float angle_diff = target_angle;   // 因为当前前进方向为0°

//     // 死区处理
//     if (fabs(angle_diff) < radians(5)) angle_diff = 0;
//     // 限幅最大转向角度（±20°）
//     angle_diff = constrain(angle_diff, -radians(20), radians(20));

//     // 归一化到 [-1, 1]
//     target_turn_norm = angle_diff / radians(20);

//     // 动态速度：前方障碍越近，速度越慢
//     float front_mag = sqrt(f_front_x * f_front_x + f_front_y * f_front_y);
//     target_speed = MAX_SPEED * (1.0f - front_mag * 0.2f);
//     target_speed = constrain(target_speed, 0, MAX_SPEED);
//   }

//   // 将归一化转向值 (-1..1) 映射到舵机角度 (左转60 ~ 直行90 ~ 右转120)
//   int target_servo_angle;
//   if (target_turn_norm < 0) {
//     // 左转
//     target_servo_angle = STEERING_CENTER + target_turn_norm * (STEERING_CENTER - STEERING_MAX_LEFT);
//   } else {
//     // 右转
//     target_servo_angle = STEERING_CENTER + target_turn_norm * (STEERING_MAX_RIGHT - STEERING_CENTER);
//   }
//   target_servo_angle = constrain(target_servo_angle, STEERING_MAX_LEFT, STEERING_MAX_RIGHT);

//   // 输出
//   out_speed = target_speed;
//   out_servo_angle = target_servo_angle;
// }

// ========== 初始化 ==========
void setup()
{
  // 可选：打开串口调试（如需调试可取消注释）
  Serial.begin(115200);

  // 启动激光雷达
  lidar.begin(Serial2);
  lidar.startScan();

  // 初始化舵机
  servo.attach(SERVO_PIN);
  servo.write(STEERING_CENTER + servo_offset);

  // 初始化电机方向引脚（前进方向）
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH); // 设置电机为正转
}

// ========== 主循环 ==========
unsigned long lastControlTime = 0;
const int CONTROL_INTERVAL = 50; // 每50ms强制计算一次，不论有没有startBit

void loop()
{
  if (lidar.waitPoint())
  {
    float distance = lidar.getCurrentPoint().distance / 1000.0f;
    int angle = (int)lidar.getCurrentPoint().angle % 360;

    // 只有在距离有效时才更新数组
    if (distance > 0.05f)
    {
      distances[angle] = distance;
    }
    else
    {
      // 如果收到0，说明没看到东西，设为一个很大的值表示安全
      distances[angle] = 5.0f;
    }

    // 改进：使用时间驱动 + startBit 双保险
    if (lidar.getCurrentPoint().startBit || (millis() - lastControlTime > CONTROL_INTERVAL))
    {
      lastControlTime = millis();

      int new_speed, new_servo_angle;
      computeControl(distances, new_speed, new_servo_angle);

      // 最后的防线：在这里强制检查 speed
      // 如果算法逻辑想停，但我们想让它蠕动
      if (new_speed < MIN_SPEED && new_speed > 0)
      {
        new_speed = MIN_SPEED;
      }

      servo.write(new_servo_angle + servo_offset);
      analogWrite(MOTOR_PIN, new_speed);
    }
  }
  else
  {
    // 移除这里的 delay(1000)，它会让车瞬间失去控制
    // 只做简单的尝试重启
    lidar.startScan();
  }
}
// void loop() {
//   if (IS_OK(lidar.waitPoint())) {   // 等待一个新扫描点
//     float distance = lidar.getCurrentPoint().distance / 1000.0f;  // 转换为米
//     int angle = lidar.getCurrentPoint().angle;                    // 角度 0~359
//     bool startBit = lidar.getCurrentPoint().startBit;

//     if (angle >= 0 && angle <= 359) {
//       distances[angle] = distance;   // 更新该角度下的最近距离
//     }

//     // 每当新一圈开始时（startBit为true），根据所有角度数据计算控制指令
//     if (startBit) {
//       int new_speed, new_servo_angle;
//       computeControl(distances, new_speed, new_servo_angle);

//       // 应用控制
//       speed = new_speed;
//       servo_angle = new_servo_angle;

//       servo.write(servo_angle + servo_offset);
//       analogWrite(MOTOR_PIN, speed);

//       // 可选：调试输出（注意会影响性能，平时注释）
//        Serial.printf("Speed=%d, Servo=%d\n", speed, servo_angle);
//     }
//   } else {
//     // 雷达通信错误，重启扫描
//     lidar.startScan();
//     delay(1000);
//   }
// }