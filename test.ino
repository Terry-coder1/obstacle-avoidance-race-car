#include <Arduino.h>
#include <math.h>

// -------------------------- 硬件引脚定义 (可根据你的接线修改) --------------------------
// 电机驱动引脚 (L298N)
#define LEFT_MOTOR_IN1 14
#define LEFT_MOTOR_IN2 27
#define LEFT_MOTOR_PWM 12
#define RIGHT_MOTOR_IN3 26
#define RIGHT_MOTOR_IN4 25
#define RIGHT_MOTOR_PWM 13

// 超声波传感器引脚 (这里以1个超声波为例，如果你有多个，可扩展此部分)
#define ULTRASONIC_TRIG 5
#define ULTRASONIC_ECHO 18

// -------------------------- 算法参数 (和你的Python代码完全对应) --------------------------
#define ROBOT_RADIUS 15
#define MAX_RANGE 200
#define ROBOT_SPEED 6
#define TURN_SPEED 0.2
#define FRONT_THRESHOLD 70    // 前方避障阈值
#define SIDE_THRESHOLD 40     // 侧边平衡阈值
#define SIDE_FORCE_WEIGHT 0.5 // 侧边力权重

// 传感器数据结构体
struct SensorData
{
    float angle;    // 传感器的绝对角度 (弧度)
    float distance; // 传感器探测到的距离 (cm)
};

// 机器人状态
float robot_angle = 0; // 机器人当前朝向

// -------------------------- 工具函数 --------------------------
// 读取HC-SR04超声波距离
long readUltrasonic(int trigPin, int echoPin)
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    // 距离转换：声速343m/s，来回所以除以2，转成cm
    return duration * 0.034 / 2;
}

// 设置左右电机的速度 (-255 ~ 255，正为前进，负为后退)
void setMotorSpeed(float left_speed, float right_speed)
{
    // 左电机
    if (left_speed > 0)
    {
        digitalWrite(LEFT_MOTOR_IN1, HIGH);
        digitalWrite(LEFT_MOTOR_IN2, LOW);
        analogWrite(LEFT_MOTOR_PWM, min((int)left_speed, 255));
    }
    else
    {
        digitalWrite(LEFT_MOTOR_IN1, LOW);
        digitalWrite(LEFT_MOTOR_IN2, HIGH);
        analogWrite(LEFT_MOTOR_PWM, min((int)-left_speed, 255));
    }

    // 右电机
    if (right_speed > 0)
    {
        digitalWrite(RIGHT_MOTOR_IN3, HIGH);
        digitalWrite(RIGHT_MOTOR_IN4, LOW);
        analogWrite(RIGHT_MOTOR_PWM, min((int)right_speed, 255));
    }
    else
    {
        digitalWrite(RIGHT_MOTOR_IN3, LOW);
        digitalWrite(RIGHT_MOTOR_IN4, HIGH);
        analogWrite(RIGHT_MOTOR_PWM, min((int)-right_speed, 255));
    }
}

// -------------------------- 核心避障算法 (和你的Python代码完全一致) --------------------------
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

        // 计算相对角度
        float rel_angle = (abs_angle - robot_angle + M_PI) fmod(2 * M_PI) - M_PI;

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

    // 合并力
    float total_x = f_front_x + f_side_x;
    float total_y = f_front_y + f_side_y;
    float combined_mag = sqrt(total_x * total_x + total_y * total_y);

    float speed = ROBOT_SPEED;
    float turn = 0;

    if (combined_mag > 0.01)
    {
        // 计算目标角度
        float target_angle = atan2(total_y, total_x);
        float angle_diff = (target_angle - robot_angle + M_PI) fmod(2 * M_PI) - M_PI;

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
        turn = max(min(angle_diff, TURN_SPEED), -TURN_SPEED);
    }

    out_speed = speed;
    out_turn = turn;
}

// -------------------------- 初始化 --------------------------
void setup()
{
    Serial.begin(115200);

    // 初始化电机引脚
    pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);
    pinMode(LEFT_MOTOR_PWM, OUTPUT);
    pinMode(RIGHT_MOTOR_IN3, OUTPUT);
    pinMode(RIGHT_MOTOR_IN4, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM, OUTPUT);

    // 初始化超声波引脚
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);

    // 初始状态
    robot_angle = 0;
}

// -------------------------- 主循环 --------------------------
void loop()
{
    // 1. 读取传感器数据，填充sensor_data
    // 注意：这里是单超声波的示例，如果你有多个超声波/激光雷达，修改这部分即可
    SensorData sensor_data[4]; // 如果你有4个传感器，就改成4个
    int sensor_count = 0;

    // 示例：读取前方超声波，角度为当前机器人朝向
    long front_dist = readUltrasonic(ULTRASONIC_TRIG, ULTRASONIC_ECHO);
    if (front_dist < MAX_RANGE)
    {
        sensor_data[sensor_count].angle = robot_angle;
        sensor_data[sensor_count].distance = front_dist;
        sensor_count++;
    }
    // 如果你有左方超声波，添加这部分：
    // long left_dist = readUltrasonic(LEFT_TRIG, LEFT_ECHO);
    // if(left_dist < MAX_RANGE) {
    //   sensor_data[sensor_count].angle = robot_angle + radians(90);
    //   sensor_data[sensor_count].distance = left_dist;
    //   sensor_count++;
    // }
    // 右方、后方的传感器同理添加即可，和你的仿真逻辑完全兼容

    // 2. 运行避障算法，得到速度和转向
    float speed, turn;
    simple_avoid_algorithm(sensor_data, sensor_count, speed, turn);

    // 3. 更新机器人朝向
    robot_angle += turn;
    // 角度归一化
    robot_angle = (robot_angle + M_PI) fmod(2 * M_PI) - M_PI;

    // 4. 差分驱动：将整体速度和转向，转换为左右轮的速度
    // 这个系数20可以根据你的小车调整，用来匹配速度和转向的灵敏度
    float left_wheel = speed * 20 - turn * 100;
    float right_wheel = speed * 20 + turn * 100;

    // 5. 控制电机
    setMotorSpeed(left_wheel, right_wheel);

    // 6. 调试输出
    Serial.printf("Speed: %.2f, Turn: %.2f, Left: %.0f, Right: %.0f\n", speed, turn, left_wheel, right_wheel);

    delay(20); // 和你的仿真一样，20ms刷新一次，约50Hz
}