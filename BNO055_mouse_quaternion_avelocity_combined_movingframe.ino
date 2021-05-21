#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <BleMouse.h>

// mouse object
BleMouse ble("test", "me");

// vector transformation structs
typedef struct sQ
{
  float x;
  float y;
  float z;
  float w;
} sQ;

typedef struct sV
{
  float x;
  float y;
  float z;
} sV;

/** I2C pins **/
#define I2C_SDA 33
#define I2C_SCL 32

// I2C object
TwoWire wire_custom = TwoWire(0);

// Sensor object
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &wire_custom);

/**************************************************************************/
void sensor_calibration(void)
{
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
  Serial.print("\t\t");
}

/**************************************************************************/
double radius_to_degrees = 57.295779513;
signed char x;
signed char y;
float a;
float x_placeholder;
float y_placeholder;

/**************************************************************************/
struct sQ q_getconj(sQ* q)
{
  sQ qc;
  qc.x = -q->x;
  qc.y = -q->y;
  qc.z = -q->z;
  qc.w = q->w;
  return qc;
}

struct sQ q_mult(sQ* q1, sQ* q2)
{
  sQ q;
  q.w = q1->w * q2->w - (q1->x * q2->x + q1->y * q2->y + q1->z * q2->z);
  q.x = q1->w * q2->x + q2->w * q1->x + q1->y * q2->z - q1->z * q2->y;
  q.y = q1->w * q2->y + q2->w * q1->y + q1->z * q2->x - q1->x * q2->z;
  q.z = q1->w * q2->z + q2->w * q1->z + q1->x * q2->y - q1->y * q2->x;
  return q;
}

void rotate_v(sQ* q, sV* v)
{
  sQ r;
  r.w = 0;
  r.x = v->x;
  r.y = v->y;
  r.z = v->z;
  sQ qc = q_getconj(q);
  sQ qq = q_mult(&r, &qc);
  sQ rq = q_mult(q, &qq);
  v->x = rq.x;
  v->y = rq.y;
  v->z = rq.z;
}

/**************************************************************************/
void sensor_data (float& x_value, float& y_value) {

  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  // imu::Quaternion quat_conjugate = bno.getQuat().conjugate();

  // angular velocity vector
  imu::Vector<3> velocity = bno.getVector (Adafruit_BNO055::VECTOR_GYROSCOPE);

  // assigning values
  sQ q;
  q.w = quat.w();
  q.x = quat.x();
  q.y = quat.y();
  q.z = quat.z();

  sV vel;
  vel.x = velocity.x();
  vel.y = velocity.y();
  vel.z = velocity.z();

  // 1. create "hand" vector
  sV hv;
  hv.x = 1;
  hv.y = 0;
  hv.z = 0;

  // 2. rotate "hand" vector
  rotate_v(&q, &hv);

  // 3. calculate yaw angle
  a = atan2(hv.y, hv.x);

  // 4. construct new quaternion
  sQ qA;
  qA.w = cos(a / 2);
  qA.x = 0;
  qA.y = 0;
  qA.z = -sin(a / 2);

  // 5. rotate velocity vector in Earth reference frame
  rotate_v(&q, &vel);

  // 5b. then rotate in new reference frame
  rotate_v(&qA, &vel);

  //Serial print
  Serial.print(" x: ");
  Serial.print(velocity.x(), 2);
  Serial.print(" y: ");
  Serial.print(velocity.y(), 2);
  Serial.print(" z: ");
  Serial.print(velocity.z(), 2);
  Serial.print("\t");

  Serial.print(" xT: ");
  Serial.print(vel.x, 2);
  Serial.print(" yT: ");
  Serial.print(vel.y, 2);
  Serial.print(" zT: ");
  Serial.print(vel.z, 2);
  Serial.print("\n");

  // Updating variables
  x_value = vel.z;
  y_value = vel.y;

  //delay between samples
  delay(10);
}

/**************************************************************************/
void transform_sensor_data  (float x_initial, float y_initial, signed char& x_final, signed char& y_final) {

  const float sensitivity = 0.5;

  x_final = round(constrain(-x_initial * sensitivity, -128, +127));
  y_final = round(constrain(y_initial * sensitivity, -128, +127));

  //Serial.print(x_final);
  //Serial.print("\t");
  //Serial.print(y_final);
  //Serial.print("\n");
}

/**************************************************************************/
void setup(void)
{
  wire_custom.begin(I2C_SDA, I2C_SCL, 400000);

  Serial.begin(115200);

  ble.begin();

  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  // bno.setExtCrystalUse(true);
}

/**************************************************************************/
void loop(void)
{
  sensor_calibration ();

  sensor_data (x_placeholder, y_placeholder);

  transform_sensor_data (x_placeholder, y_placeholder, x, y);

  if (ble.isConnected()) {
    ble.move(x, y);
  }
}
