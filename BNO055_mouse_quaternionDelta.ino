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
  //delay between samples
  delay(10);
 // 2nd Quaternion data
  imu::Quaternion quat_two = bno.getQuat();
  // quaternion comparison
  imu::Quaternion quatDelta = quat_two * quat.conjugate ();
  imu::Vector<3> euler = quatDelta.toEuler();
  Serial.print(" X: ");
  Serial.print(euler.x()*radius_to_degrees, 2);
  Serial.print(" Y: ");
  Serial.print(euler.y()*radius_to_degrees, 2);
  Serial.print(" Z: ");
  Serial.print(euler.z()*radius_to_degrees, 2);
  Serial.print("\t\t");
  
  // Updating variables
  x_value = euler.x() * radius_to_degrees;
  y_value = euler.y() * radius_to_degrees;
}

/**************************************************************************/
void transform_sensor_data  (float x_initial, float y_initial, signed char& x_final, signed char& y_final) {
  
  const float sensitivity = 2;
  
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
  bno.setExtCrystalUse(true);
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
