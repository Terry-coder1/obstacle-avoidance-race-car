#include <RPLidarC1.h>
#include <ESP32Servo.h>


// ========== 硬件引脚定义 ==========
const int SERVO_PIN = 10; 
const int DIR_PIN = 46;   
const int MOTOR_PIN = 21; 

// ========== 核心算法参数 (针对 50cm 轨道微调) ==========
#define MAX_SPEED 32          // 巡航 PWM 值
#define MIN_SPEED 18          // 保证电机能转动的最小 PWM (防止嗡嗡响却不走)
#define FRONT_THRESHOLD 0.60  // 前方探测距离 (m)，50cm 开始减速比较稳
#define SIDE_THRESHOLD 0.24   // 侧边阈值 (m)，必须小于 0.25m，否则走在路中间会感到“压抑”
#define STOP_DISTANCE 0.05    // 过近距离 (m)

// 动力分配权重
#define DRIVE_BIAS 0.6f       // 强力前进权重，防止被两侧斥力抵消导致停车
#define TURN_SENSITIVITY 2.8f // 转向灵敏度，数值越大窄道纠偏越快
#define STEERING_CENTER 90
#define STEERING_LIMIT 25     // 转向角限制在中心点 +/- 25度

// ========== 全局对象与变量 ==========
RPLidar lidar;
Servo servo;

int servo_offset = 6;         // 舵机安装物理偏差调节
float distances[360] = {0};   // 雷达数据缓存
unsigned long lastControlTime = 0;
unsigned long lastDataTime = 0;
const int CONTROL_INTERVAL = 50; // 20Hz 控制频率

// ========== 核心控制算法 ==========
void computeControl(float* dists, int &out_speed, int &out_servo_angle) {
    float repel_x = 0, repel_y = 0;
    int count = 0;

    // 1. 计算斥力向量 (仅处理前方 180 度)
    for (int a = 0; a < 360; a++) {
        float d = dists[a];
        if (d <= 0.05f || d > 1.5f) continue; // 忽略无效和过远数据

        int rel_angle = (a > 180) ? (a - 360) : a;
        if (abs(rel_angle) <= 90) {
            float threshold = (abs(rel_angle) <= 30) ? FRONT_THRESHOLD : SIDE_THRESHOLD;
            
            if (d < threshold) {
                float weight = (threshold - d) / threshold;
                float rad = rel_angle * PI / 180.0f;
                // 斥力方向：物体方向 + 180度
                repel_x += weight * cos(rad + PI);
                repel_y += weight * sin(rad + PI);
                count++;
            }
        }
    }

    if (count > 0) {
        repel_x /= count;
        repel_y /= count;
    }

    // 2. 合成目标向量 (关键：确保前进趋势始终占优)
    // DRIVE_BIAS 设为 0.6，远大于平均斥力，确保 target_x 不会变为负数
    float target_x = DRIVE_BIAS + repel_x; 
    float target_y = repel_y * TURN_SENSITIVITY;

    float target_deg = 0.8*atan2(target_y, target_x) * 180.0f / PI+0.2*target_deg;
    // ========== 新增：打印目标角度和舵机输出 ==========
Serial.printf("【控制输出】target_deg: %.2f° | out_servo: %d\n",
              target_deg, out_servo_angle);
// ==================================================


    // 3. 舵机逻辑：限制转向幅度，防止窄道内“甩尾”撞墙
    int steer_diff = -(int)constrain(target_deg, -STEERING_LIMIT, STEERING_LIMIT);
    out_servo_angle = STEERING_CENTER + steer_diff;

    // 4. 速度逻辑：正前方 40 度范围内最小距离决定速度
    // float min_front_d = 2.0f;
    // for (int a = -10; a <= 10; a++) {
    //     int idx = (a + 360) % 360;
    //     if (dists[idx] > 0.05f && dists[idx] < min_front_d) {
    //         min_front_d = dists[idx];
    //     }
    // }

    // if (min_front_d < STOP_DISTANCE) {
    //     out_speed = MIN_SPEED; // 距离太近，紧急避险
    // } else if (min_front_d < FRONT_THRESHOLD) {
    //     // 线性减速，但保留最低蠕动速度
    //     float factor = (MAX_SPEED - MIN_SPEED) / (FRONT_THRESHOLD - STOP_DISTANCE)*(min_front_d-STOP_DISTANCE)/MAX_SPEED+MIN_SPEED/MAX_SPEED;
    //     out_speed = (int)(MAX_SPEED * factor);
    //     if (out_speed < MIN_SPEED) out_speed = MIN_SPEED;
    // } else {
    //     out_speed = MAX_SPEED;
    // }
    // 4. 速度逻辑：结合 避障减速 + 转弯自动限速
float min_front_d = 2.0f;
for (int a = -10; a <= 10; a++) {
    int idx = (a + 360) % 360;
    if (dists[idx] > 0.05f && dists[idx] < min_front_d) {
        min_front_d = dists[idx];
    }
}

// 第一步：先计算避障需要的速度（保留你原来的避障逻辑）
float obstacle_speed;
if (min_front_d < STOP_DISTANCE) {
    obstacle_speed = MIN_SPEED; // 距离太近，最低蠕动速度
} else if (min_front_d < FRONT_THRESHOLD) {
    // 用map简化线性减速，比手动公式更直观
    obstacle_speed = map(min_front_d, STOP_DISTANCE, FRONT_THRESHOLD, MIN_SPEED, MAX_SPEED);
} else {
    obstacle_speed = MAX_SPEED;
}

// ==============================================
// 新增：转弯自动限速逻辑（解决你当前的问题）
// ==============================================
// steer_diff 就是你之前的转向偏移量（-30 ~ 30）
float turn_angle_abs = abs(steer_diff);
// 转向角度越大，限速越低：
// - 直行(0°)：限速MAX_SPEED
// - 最大转弯(30°)：限速MIN_SPEED
float turn_angle_norm = turn_angle_abs / STEERING_LIMIT; // 归一化到0~1
// 平方曲线：小角度减速少，大角度减速多
float turn_factor = 1.0f - 0.6f * turn_angle_norm * turn_angle_norm;
float turn_limit_speed = MAX_SPEED * turn_factor;



// ==============================================
// 最终速度：取两个速度的最小值，哪个更严格用哪个
// 1. 避障需要减速时，用避障的速度
// 2. 转弯需要减速时，用转弯的限速
// ==============================================
out_speed = min(obstacle_speed, turn_limit_speed);

// 全局最低速度保护，永远不低于MIN_SPEED
out_speed = max(out_speed, MIN_SPEED);

}

void setup() {
    Serial.begin(115200);
    
    // 初始化雷达 (Serial2)
    lidar.begin(Serial2);
    lidar.startScan();

    // 初始化舵机
    servo.attach(SERVO_PIN);
    servo.write(STEERING_CENTER + servo_offset);

    // 初始化电机
    pinMode(DIR_PIN, OUTPUT);
    pinMode(MOTOR_PIN, OUTPUT);
    digitalWrite(DIR_PIN, HIGH); 
    analogWrite(MOTOR_PIN, 0);

    // 预填充数据，防止初始判断出错
    for(int i=0; i<360; i++) distances[i] = 2.0f;
}

void loop() {
    if (IS_OK(lidar.waitPoint())) {
        lastDataTime = millis();
        float d = lidar.getCurrentPoint().distance / 1000.0f;
        int a = lidar.getCurrentPoint().angle ;
 
        if (d > 0.05f) {
            distances[a] = d;
        } else if (d == 0) {
            distances[a] = 3.0f; // 0通常代表无穷远或吸收
        }

        // 达到控制周期或雷达转完一圈
        if (lidar.getCurrentPoint().startBit || (millis() - lastControlTime >= CONTROL_INTERVAL)) {
            lastControlTime = millis();
 // ========== 新增：打印四个关键方向的距离 ==========
    Serial.printf("【雷达数据】0°: %.2fm | 90°: %.2fm | 180°: %.2fm | 270°: %.2fm\n",
                  distances[1], distances[91], distances[181], distances[271]);
    // ==================================================

            int final_speed, final_servo;
            computeControl(distances, final_speed, final_servo);

            // 执行控制
            servo.write(final_servo + servo_offset);
            analogWrite(MOTOR_PIN, final_speed);

            // 调试信息（建议测试稳了后注释掉，减少开销）
            // Serial.printf("Speed: %d | Angle: %d | F_Dist: %.2f\n", final_speed, final_servo, distances[0]);
        }
    } else {
    // --- 修复后的容错处理：雷达掉线保护 ---
    static unsigned long lastRestartTime = 0;
    // 增加重启间隔限制，防止频繁重启导致系统卡死
    if (millis() - lastDataTime > 500 && millis() - lastRestartTime > 2000) {
        lastRestartTime = millis();
        analogWrite(MOTOR_PIN, MIN_SPEED); // 立即停车，保证安全
        Serial.println(" 雷达通信超时，正在重启扫描...");
        lidar.startScan(); // 直接重启扫描，无需先停止
        lastDataTime = millis();
    }
}
        
}