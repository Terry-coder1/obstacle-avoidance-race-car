#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include <RPLidar.h>

// ==============================================
// 硬件引脚定义 - 请根据你的实际接线修改这里
// ==============================================
// 后轮电机驱动 (L298N)
#define MOTOR_IN1 14 // 左后轮方向1
#define MOTOR_IN2 27 // 左后轮方向2
#define MOTOR_IN3 26 // 右后轮方向1
#define MOTOR_IN4 25 // 右后轮方向2
#define MOTOR_PWM 12 // 后轮PWM调速 (两个后轮共用一个PWM)

// 舵机转向
#define STEERING_SERVO_PIN 13  // 舵机信号线
#define STEERING_CENTER 90     // 舵机中值 (直行角度)
#define STEERING_MAX_LEFT 60   // 最大左转角度
#define STEERING_MAX_RIGHT 120 // 最大右转角度

// RPLIDAR C1 激光雷达 (UART2)
#define RPLIDAR_TX 16   // 雷达TX接ESP32 RX2
#define RPLIDAR_RX 17   // 雷达RX接ESP32 TX2
#define RPLIDAR_MOTOR 4 // 雷达电机控制引脚

// ==============================================
// 算法参数 - 与你的Python仿真完全对应
// ==============================================
#define ROBOT_RADIUS 15       // 小车半径 (cm)
#define MAX_RANGE 200         // 激光雷达最大探测距离 (cm)
#define MAX_SPEED 40          // 比赛限制最大速度 (PWM 0-255)
#define TURN_SPEED 0.2        // 转向速度 (弧度/帧)
#define FRONT_THRESHOLD 70    // 前方避障阈值 (cm)
#define SIDE_THRESHOLD 40     // 侧边平衡阈值 (cm)
#define SIDE_FORCE_WEIGHT 0.5 // 侧边力权重

// 传感器数据结构体
struct SensorData
{
    float angle;    // 绝对角度 (弧度)
    float distance; // 距离 (cm)
};

// ==============================================
// 全局对象和变量
// ==============================================
RPLidar lidar;
Servo steeringServo;
float robot_angle = 0; // 机器人当前朝向 (弧度)

// ==============================================
// 硬件控制函数
// ==============================================

// 设置后轮速度 (-MAX_SPEED ~ MAX_SPEED，正为前进，负为后退)
void setMotorSpeed(int speed)
{
    // 限制速度不超过比赛规定的最大值
    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

    // 左右电机都反转 (根据你的要求预设)
    if (speed > 0)
    {
        // 前进
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, HIGH);
        digitalWrite(MOTOR_IN3, LOW);
        digitalWrite(MOTOR_IN4, HIGH);
        analogWrite(MOTOR_PWM, speed);
    }
    else if (speed < 0)
    {
        // 后退
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
        digitalWrite(MOTOR_IN3, HIGH);
        digitalWrite(MOTOR_IN4, LOW);
        analogWrite(MOTOR_PWM, -speed);
    }
    else
    {
        // 停止
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, LOW);
        digitalWrite(MOTOR_IN3, LOW);
        digitalWrite(MOTOR_IN4, LOW);
        analogWrite(MOTOR_PWM, 0);
    }
}

// 设置舵机转向角度 (-1.0 ~ 1.0，-1为最左，1为最右)
void setSteering(float turn)
{
    turn = constrain(turn, -1.0, 1.0);

    int servoAngle;
    if (turn < 0)
    {
        // 左转
        servoAngle = map(turn * 100, -100, 0, STEERING_MAX_LEFT, STEERING_CENTER);
    }
    else
    {
        // 右转
        servoAngle = map(turn * 100, 0, 100, STEERING_CENTER, STEERING_MAX_RIGHT);
    }

    steeringServo.write(servoAngle);
}

// ==============================================
// 核心避障算法 - 与你的Python代码完全一致
// ==============================================
void simple_avoid_algorithm(SensorData *sensor_data, int sensor_count, float &out_speed, float &out_turn)
{
    float f_front_x = 0, f_front_y = 0;
    float f_side_x = 0, f_side_y = 0;

    // 前进的基础力
    f_side_x += 0.1 * cos(robot_angle);
    f_side_y += 0.1 * sin(robot_angle);

    for (int i = 0; i < sensor_count; i++)
    {
        float abs_angle = sensor_data[i].angle;
        float dist = sensor_data[i].distance;

        // 过滤无效数据
        if (dist <= 0 || dist > MAX_RANGE)
            continue;

        // 计算相对角度
        float rel_angle = fmod(abs_angle - robot_angle + M_PI, 2 * M_PI) - M_PI;

        // 1. 处理前方45°范围的紧急避障
        if (fabs(rel_angle) <= radians(45))
        {
            if (dist < FRONT_THRESHOLD)
            {
                float mag = (FRONT_THRESHOLD - dist) / FRONT_THRESHOLD;
                f_front_x += mag * cos(abs_angle + M_PI);
                f_front_y += mag * sin(abs_angle + M_PI);
            }
        }
        // 2. 处理侧边45°~90°的边缘平衡
        else if (fabs(rel_angle) > radians(45) && fabs(rel_angle) <= radians(90))
        {
            if (dist < SIDE_THRESHOLD)
            {
                float mag = ((SIDE_THRESHOLD - dist) / SIDE_THRESHOLD) * SIDE_FORCE_WEIGHT;
                f_side_x += mag * cos(abs_angle + M_PI);
                f_side_y += mag * sin(abs_angle + M_PI);
            }
        }
    }

    // 合并所有的力
    float total_x = f_front_x + f_side_x;
    float total_y = f_front_y + f_side_y;
    float combined_mag = sqrt(total_x * total_x + total_y * total_y);

    float speed = MAX_SPEED;
    float turn = 0;

    if (combined_mag > 0.01)
    {
        // 计算最终目标角度
        float target_angle = atan2(total_y, total_x);
        float angle_diff = fmod(target_angle - robot_angle + M_PI, 2 * M_PI) - M_PI;

        // 角度死区和限制
        if (angle_diff < radians(5) && angle_diff > -radians(5))
        {
            angle_diff = 0;
        }
        if (angle_diff > radians(20))
            angle_diff = radians(20);
        if (angle_diff < -radians(20))
            angle_diff = -radians(20);

        // 转向限制
        turn = max(min(angle_diff / radians(20), 1.0), -1.0);

        // 动态速度：前方有障碍时减速
        float front_mag = sqrt(f_front_x * f_front_x + f_front_y * f_front_y);
        speed = MAX_SPEED * (1.0 - front_mag * 0.7);
    }

    out_speed = speed;
    out_turn = turn;
}

// ==============================================
// 初始化
// ==============================================
void setup()
{
    Serial.begin(115200);

    // 初始化电机引脚
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_IN3, OUTPUT);
    pinMode(MOTOR_IN4, OUTPUT);
    pinMode(MOTOR_PWM, OUTPUT);

    // 初始化舵机
    steeringServo.attach(STEERING_SERVO_PIN);
    steeringServo.write(STEERING_CENTER);

    // 初始化激光雷达
    lidar.begin(Serial2);
    pinMode(RPLIDAR_MOTOR, OUTPUT);
    digitalWrite(RPLIDAR_MOTOR, HIGH); // 启动雷达电机

    // 等待雷达启动
    delay(1000);
    Serial.println("RPLIDAR C1 启动成功！");

    // 初始状态
    robot_angle = 0;
    setMotorSpeed(0);
}

// ==============================================
// 主循环
// ==============================================
void loop()
{
    // 1. 读取激光雷达数据
    SensorData sensor_data[360]; // 最多存储360个点
    int sensor_count = 0;

    rplidar_response_measurement_node_t nodes[72];
    size_t count = _countof(nodes);

    // 扫描一次雷达数据
    if (lidar.grabScanData(nodes, count, 1000))
    {
        lidar.ascendScanData(nodes, count);

        for (int i = 0; i < count && sensor_count < 360; i++)
        {
            float angle = (nodes[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
            float distance = nodes[i].distance_q2 / 4.0f;

            // 转换为弧度，并调整坐标系 (0°为正前方，逆时针增加)
            angle = radians(360 - angle);

            sensor_data[sensor_count].angle = angle;
            sensor_data[sensor_count].distance = distance;
            sensor_count++;
        }
    }

    // 2. 运行避障算法
    float speed, turn;
    simple_avoid_algorithm(sensor_data, sensor_count, speed, turn);

    // 3. 更新机器人朝向
    robot_angle += turn * TURN_SPEED;
    robot_angle = fmod(robot_angle + M_PI, 2 * M_PI) - M_PI;

    // 4. 控制硬件
    setMotorSpeed((int)speed);
    setSteering(turn);

    // 5. 调试输出
    Serial.printf("速度: %.1f, 转向: %.2f, 雷达点数: %d\n", speed, turn, sensor_count);

    // 与仿真保持相同的50Hz刷新率
    delay(20);
}