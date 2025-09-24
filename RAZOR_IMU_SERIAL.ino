#include <ArduinoJson.h>
#include <SparkFunMPU9250-DMP.h>

#define SerialPort SerialUSB //This Use Default Serial
// #define SerialPort Serial1 //This use for RX TX pin

#define TODEG 180.0 / M_PI
#define TORAD M_PI / 180.0

MPU9250_DMP imu;
DynamicJsonDocument doc(1024);

struct Quaternion {
  double w, x, y, z;
};

struct EulerAngles {
  double roll, pitch, yaw;
};

// this implementation assumes normalized quaternion
// converts to Euler angles in 3-2-1 sequence
EulerAngles ToEulerAngles(Quaternion q) {
  EulerAngles angles;

  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  angles.roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
  double cosp = sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
  angles.pitch = 2 * atan2(sinp, cosp) - M_PI / 2;

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  angles.yaw = atan2(siny_cosp, cosy_cosp);

  return angles;
}

void setup() {
  SerialPort.begin(115200);
  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS) {
    while (1) {
      // SerialPort.println("Unable to communicate with MPU-9250");
      // SerialPort.println("Check connections, and try again.");
      // SerialPort.println();
      delay(5000);
    }
  }

  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_CAL_GYRO |  // Enable 6-axis quat
                 DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_RAW_ACCEL,  // Use gyro calibration
               60);// Set DMP FIFO rate to 10 Hz

  // imu.set
  imu.resetFifo();  
  delay(1);
}

void loop() {
  // Check for new data in the FIFO
  if (imu.fifoAvailable()) {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if (imu.dmpUpdateFifo() == INV_SUCCESS) {
      imu.computeEulerAngles();
      printIMUData();
    }
  }
}

double x = 0;
void printIMUData(void) {
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  // Quaternion values are, by default, stored in Q30 long
  // format. calcQuat turns them into a float between -1 and 1
  float q0 = imu.calcQuat(imu.qw);
  float q1 = imu.calcQuat(imu.qx);
  float q2 = imu.calcQuat(imu.qy);
  float q3 = imu.calcQuat(imu.qz);

  Quaternion quat;
  quat.w = q0;
  quat.x = q1;
  quat.y = q2;
  quat.z = q3;

  float AccX = imu.calcAccel(imu.ax);
  float AccY = imu.calcAccel(imu.ay);
  float AccZ = imu.calcAccel(imu.az);


  float gyX = imu.calcGyro(imu.gx);
  float gyY = imu.calcGyro(imu.gy);
  float gyZ = imu.calcGyro(imu.gz);

  EulerAngles euler = ToEulerAngles(quat);


  // doc["roll"].set(imu.roll);
  // doc["pitch"].set(imu.pitch);
  doc["yaw"].set(imu.yaw);



  const double accel_x = imu.calcAccel(imu.ax) * 9.80f;
  const double accel_y = imu.calcAccel(imu.ay) * 9.80f;
  const double accel_z = imu.calcAccel(imu.az) * 9.80f;

  const double alpha = euler.yaw;
  const double beta = euler.pitch;
  const double gamma = euler.roll;

  double x_ = (cos(alpha) * cos(beta)) * accel_x + (cos(alpha) * sin(beta) * sin(gamma) - sin(alpha) * cos(gamma)) * accel_y + (cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma)) * accel_z;

  double y_ = (sin(alpha) * cos(beta)) * accel_x + (sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma)) * accel_y + (sin(alpha) * sin(beta) * cos(gamma) - cos(alpha) * sin(gamma)) * accel_z;

  double z_ = (-sin(beta) * accel_x) + (cos(beta) * sin(gamma)) * accel_y + (cos(beta) * cos(gamma)) * accel_z;

  // x +=x_;
  doc["accX"].set(AccX);
  doc["accY"].set(AccY);
  // doc["accZ"].set(AccZ);

  String jsonString;
  serializeJson(doc, jsonString);
  SerialPort.println(jsonString);
}