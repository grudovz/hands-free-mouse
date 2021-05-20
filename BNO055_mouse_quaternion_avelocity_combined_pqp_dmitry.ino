#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <BleMouse.h>

// mouse object
BleMouse ble("test", "me");

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
float x_placeholder;
float y_placeholder;

/**************************************************************************/
void sensor_data (float& x_value, float& y_value) {
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  imu::Quaternion quat_conjugate = bno.getQuat().conjugate();

  // angular velocity vector
  imu::Vector<3> velocity = bno.getVector (Adafruit_BNO055::VECTOR_ACCELEROMETER);

  typedef struct sQ
  {
    double x = quat.x;
    double y = quat.y;
    double z = quat.z;
    double w = quat.w;
  } sQ;

  typedef struct sV
  {
    double x = velocity.x;
    double y = velocity.y;
    double z = velocity.z;
  } sV;

  sQ q_getconj(sQ* q)
  {
    sQ qc;
    qc.x = -q->x;
    qc.y = -q->y;
    qc.z = -q->z;
    qc.w = q->w;
    return qc;
  }

  sQ q_mult(sQ* q1, sQ* q2)
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


  //Serial print
  Serial.print(" x: ");
  Serial.print(velocity.z(), 2);
  Serial.print(" y: ");
  Serial.print(velocity.y(), 2);
  Serial.print(" z: ");
  Serial.print(velocity.x(), 2);
  Serial.print("\t");

  // transformed angular velocity vector
  // quat * velocity * quat_conjugate;
  imu::Vector<3> velocity_transformed = bno.getQuat().rotateVector(velocity);

  //Serial print
  Serial.print(" transformed:  ");
  Serial.print(" x: ");
  Serial.print(velocity_transformed.z(), 2);
  Serial.print(" y: ");
  Serial.print(velocity_transformed.y(), 2);
  Serial.print(" z: ");
  Serial.print(velocity_transformed.x(), 2);
  Serial.print("\t");

  Serial.print("qW: ");
  Serial.print(quat.w(), 2);
  Serial.print(" qX: ");
  Serial.print(quat.x(), 2);
  Serial.print(" qY: ");
  Serial.print(quat.y(), 2);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 2);
  Serial.print("\n");

  // Updating variables
  // x_value = euler.x() * radius_to_degrees;
  // y_value = euler.y() * radius_to_degrees;

  //delay between samples
  delay(100);
}


/**************************************************************************/
void transform_sensor_data  (float x_initial, float y_initial, signed char& x_final, signed char& y_final) {

  const float sensitivity = 1;

  x_final = round(constrain(-x_initial * sensitivity, -128, +127));
  y_final = round(constrain(y_initial * sensitivity, -128, +127));

  Serial.print(x_final);
  Serial.print("\t");
  Serial.print(y_final);
  Serial.print("\n");
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
  /*
    transform_sensor_data (x_placeholder, y_placeholder, x, y);

    if (ble.isConnected()) {
      ble.move(x, y);
    }
  */
}
