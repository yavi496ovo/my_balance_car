#include "pid_app.h"

#include "motor.h"

/* ===================== PID 调参区 ===================== */
float Med_Angle = -1.62f;          /* 机械中值：车静止直立时的角度 */
float Vertical_Kp = 45.0f;         /* 直立环 P：越大扶正越有力，过大容易抖 */
float Vertical_Kd = 1.0f;          /* 直立环 D：陀螺仪阻尼，越大越抑制前后摆 */

float Target_Speed = 0.0f;        /* 目标速度：直接参与 Velocity()，0 表示原地平衡 */
float Velocity_Kp = 0.00f;         /* 速度环 P：越大速度修正越强；为 0 时目标速度不会产生作用 */
float Velocity_Ki;                 /* 速度环 I：在 Velocity() 中自动等于 Kp/250 */
float Velocity_Filter = 0.5f;      /* 速度误差滤波系数 a：越大越平滑，响应越慢 */
float Velocity_Integral_Limit = 20000.0f; /* 速度环积分限幅，参考工程为 +/-20000 */

float Target_Turn = 0.0f;          /* 目标转向：0 表示不转向 */
float Turn_Kp = 0.0f;              /* 转向环 P */
float Turn_Kd = 0.0f;              /* 转向环 D */
float Turn_Limit = 500.0f;         /* 转向输出限幅，参考工程为 500 */

float Output_Limit = 900.0f;       /* 电机 PWM 输出限幅 */
float Output_Direction = 1.0f;     /* 整体输出方向：若直立环整体反了，改成 -1 */
float Left_Direction = 1.0f;       /* 左电机方向修正：左轮反了就改成 -1 */
float Right_Direction = 1.0f;      /* 右电机方向修正：右轮反了就改成 -1 */

uint8_t Speed_Polarity_Test = 0U;  /* 速度极性测试：1 开启；正常平衡必须为 0 */
float Speed_Polarity_Kp = 10.0f;   /* 极性测试比例：PWM 太小看不出就加大 */

typedef struct {
    float angle_deg;
    float gyro_dps;
    float left_speed;
    float right_speed;

    float speed_feedback;
    float speed_error_lpf;
    float speed_integral;

    float velocity_out;
    float target_angle_deg;
    float vertical_out;
    float turn_out;
    float left_pwm;
    float right_pwm;

    uint8_t sensor_ready;
    uint8_t running;
    uint8_t fault;
} pid_app_t;

static pid_app_t pid_app;

static void pid_app_reset(void);
static float Limit_Value(float value, float min, float max);
static void Limit_Motor(float *moto1, float *moto2);
static int16_t Float_To_PWM(float value);
static float Vertical(float Med, float Angle, float gyro_Y);
static float Velocity(float Target, float encoder_L, float encoder_R);
static float Turn(float gyro_Z, float Target_turn);
static void Speed_Polarity_Test_Task(void);

static float Limit_Value(float value, float min, float max)
{
    if (value < min) {
        return min;
    }

    if (value > max) {
        return max;
    }

    return value;
}

static void Limit_Motor(float *moto1, float *moto2)
{
    *moto1 = Limit_Value(*moto1, -Output_Limit, Output_Limit);
    *moto2 = Limit_Value(*moto2, -Output_Limit, Output_Limit);
}

static int16_t Float_To_PWM(float value)
{
    value = Limit_Value(value, -(float)MOTOR_PWM_MAX, (float)MOTOR_PWM_MAX);

    if (value >= 0.0f) {
        return (int16_t)(value + 0.5f);
    }

    return (int16_t)(value - 0.5f);
}

static float Vertical(float Med, float Angle, float gyro_Y)
{
    return Vertical_Kp * (Angle - Med) + Vertical_Kd * gyro_Y;
}

static float Velocity(float Target, float encoder_L, float encoder_R)
{
    float Err;
    float Err_LowOut;
    float temp;

    Velocity_Ki = Velocity_Kp / 200.0f;

    Err = (encoder_L + encoder_R) - Target;
    Err_LowOut =
        (1.0f - Velocity_Filter) * Err +
        Velocity_Filter * pid_app.speed_error_lpf;
    pid_app.speed_error_lpf = Err_LowOut;

    pid_app.speed_integral += Err_LowOut;
    pid_app.speed_integral =
        Limit_Value(pid_app.speed_integral,
                    -Velocity_Integral_Limit,
                    Velocity_Integral_Limit);

    pid_app.speed_feedback = encoder_L + encoder_R;

    temp = Velocity_Kp * Err_LowOut +
           Velocity_Ki * pid_app.speed_integral;
    return temp;
}

static float Turn(float gyro_Z, float Target_turn)
{
    float temp;

    temp = Turn_Kp * Target_turn + Turn_Kd * gyro_Z;
    return Limit_Value(temp, -Turn_Limit, Turn_Limit);
}

static void Speed_Polarity_Test_Task(void)
{
    float speed;
    float pwm;

    /*
     * 速度极性测试旁路：
     * 1. 车轮必须架空，不能上地跑。
     * 2. 此模式不使用 Target_Speed，也不经过直立环，只测试“编码器速度 -> PWM”方向。
     * 3. 手动正向转轮时，如果 speed_feedback 为正，left_pwm/right_pwm 也应为正。
     * 4. 测完必须把 Speed_Polarity_Test 改回 0，恢复正常 PID 控制链。
     */
    speed = pid_app.left_speed + pid_app.right_speed;
    pwm = Speed_Polarity_Kp * speed;

    pid_app.speed_feedback = speed;
    pid_app.speed_error_lpf = speed;
    pid_app.speed_integral = 0.0f;
    pid_app.velocity_out = 0.0f;
    pid_app.target_angle_deg = Med_Angle;
    pid_app.vertical_out = pwm;
    pid_app.turn_out = 0.0f;
    pid_app.left_pwm = Limit_Value(pwm, -Output_Limit, Output_Limit);
    pid_app.right_pwm = Limit_Value(pwm, -Output_Limit, Output_Limit);

    motor_set_output(Float_To_PWM(pid_app.left_pwm),
                     Float_To_PWM(pid_app.right_pwm));
}

void pid_app_init(void)
{
    pid_app_reset();
    pid_app.running = 1U;
    motor_enable(1U);
}

static void pid_app_reset(void)
{
    pid_app.angle_deg = Med_Angle;
    pid_app.gyro_dps = 0.0f;
    pid_app.left_speed = 0.0f;
    pid_app.right_speed = 0.0f;
    pid_app.speed_feedback = 0.0f;
    pid_app.speed_error_lpf = 0.0f;
    pid_app.speed_integral = 0.0f;
    pid_app.velocity_out = 0.0f;
    pid_app.target_angle_deg = Med_Angle;
    pid_app.vertical_out = 0.0f;
    pid_app.turn_out = 0.0f;
    pid_app.left_pwm = 0.0f;
    pid_app.right_pwm = 0.0f;
    pid_app.sensor_ready = 0U;
    pid_app.fault = 0U;
}

void pid_app_task(void)
{
    float angle;
    float gyro;
    float PWM_out;
    float MOTO1;
    float MOTO2;

    if (pid_app.running == 0U) {
        motor_set_output(0, 0);
        return;
    }

    if (Speed_Polarity_Test != 0U) {
        Speed_Polarity_Test_Task();
        return;
    }

    if (pid_app.sensor_ready != 0U) {
        angle = pid_app.angle_deg;
        gyro = pid_app.gyro_dps;
    } else {
        /* 传感器未就绪时用机械中值，避免上电乱动。 */
        angle = Med_Angle;
        gyro = 0.0f;
    }

    /*
     * 对应参考工程 Control()：
     * Velocity_out = Velocity(Target_Speed, Encoder_Left, Encoder_Right);
     * Vertical_out = Vertical(Velocity_out + Med_Angle, Angle, Gyro);
     * MOTO1 = Vertical_out - Turn_out;
     * MOTO2 = Vertical_out + Turn_out;
     */
    pid_app.velocity_out =
        Velocity(Target_Speed, pid_app.left_speed, pid_app.right_speed);
    pid_app.target_angle_deg = pid_app.velocity_out + Med_Angle;
    pid_app.vertical_out =
        Vertical(pid_app.target_angle_deg, angle, gyro);
    pid_app.turn_out = Turn(0.0f, Target_Turn);

    PWM_out = Output_Direction * pid_app.vertical_out;
    MOTO1 = PWM_out - pid_app.turn_out;
    MOTO2 = PWM_out + pid_app.turn_out;
    Limit_Motor(&MOTO1, &MOTO2);

    pid_app.left_pwm = MOTO1 * Left_Direction;
    pid_app.right_pwm = MOTO2 * Right_Direction;

    motor_set_output(Float_To_PWM(pid_app.left_pwm),
                     Float_To_PWM(pid_app.right_pwm));
}

void pid_app_update_sensor(float angle_deg, float gyro_dps, uint8_t ready)
{
    pid_app.angle_deg = angle_deg;
    pid_app.gyro_dps = gyro_dps;
    pid_app.sensor_ready = ready;
}

void pid_app_update_encoder(float left_speed, float right_speed)
{
    pid_app.left_speed = left_speed;
    pid_app.right_speed = right_speed;
}

void pid_app_get_debug(pid_app_debug_t *debug)
{
    if (debug == 0) {
        return;
    }

    debug->angle_deg = pid_app.angle_deg;
    debug->target_angle_deg = pid_app.target_angle_deg;
    debug->speed_angle_deg = pid_app.velocity_out;
    debug->speed_target = Target_Speed;
    debug->speed_feedback = pid_app.speed_feedback;
    debug->gyro_dps = pid_app.gyro_dps;
    debug->balance_pwm = pid_app.vertical_out;
    debug->turn_pwm = pid_app.turn_out;
    debug->left_pwm = pid_app.left_pwm;
    debug->right_pwm = pid_app.right_pwm;
    debug->sensor_ready = pid_app.sensor_ready;
    debug->running = pid_app.running;
    debug->fault = pid_app.fault;
}
