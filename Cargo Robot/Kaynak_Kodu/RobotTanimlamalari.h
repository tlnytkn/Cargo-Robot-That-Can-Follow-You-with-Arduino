char auth[] = "S2crXISLMlMw0wVCqXkzByyU4iQP3l5R";

#define SERVO_PIN 3
#define GPS_TX_PIN 6

#define BLUETOOTH_TX_PIN 10
#define BLUETOOTH_RX_PIN 11

#define MOTOR_A_EN_PIN 5
#define MOTOR_B_EN_PIN 9
#define MOTOR_A_IN_1_PIN 7
#define MOTOR_A_IN_2_PIN 8
#define MOTOR_B_IN_1_PIN 12
#define MOTOR_B_IN_2_PIN 4
#define MOTOR_A_OFFSET 20
#define MOTOR_B_OFFSET 0

#define DECLINATION_ANGLE 0.23f
#define COMPASS_OFFSET 0.0f
#define GPS_UPDATE_INTERVAL 1000
#define GPS_STREAM_TIMEOUT 18
#define GPS_WAYPOINT_TIMEOUT 45

struct Cografi_Lokasyon {
  float lat;
  float lon;
};
