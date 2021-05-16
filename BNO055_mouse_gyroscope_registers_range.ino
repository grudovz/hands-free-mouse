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

// other variables
uint8_t register_1;

/**************************************************************************/
void write_register(uint8_t sensor_address, uint8_t register_address, uint8_t register_value) {
  wire_custom.beginTransmission (sensor_address);
  wire_custom.write(register_address);
  wire_custom.write(register_value);
  wire_custom.endTransmission ();
  delay(100);
}

/**************************************************************************/
byte read_register(uint8_t sensor_address, uint8_t register_address, uint8_t bytes) {
  uint8_t result;
  wire_custom.beginTransmission (sensor_address);
  wire_custom.write(register_address);
  wire_custom.endTransmission ();
  wire_custom.requestFrom(sensor_address, bytes);
  result = wire_custom.read();
  return result;
}

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
  //delay between samples
  delay(1);

  // angular velocity vector
  imu::Vector<3> velocity = bno.getVector (Adafruit_BNO055::VECTOR_GYROSCOPE);
  Serial.print(" x: ");
  Serial.print(velocity.z(), 2);
  Serial.print(" y: ");
  Serial.print(velocity.y(), 2);
  Serial.print(" z: ");
  Serial.print(velocity.x(), 2);
  Serial.print("\t");

  // Updating variables
  x_value = velocity.z();
  y_value = velocity.y();
}

/**************************************************************************/
void transform_sensor_data  (float x_initial, float y_initial, signed char& x_final, signed char& y_final) {
  const float sensitivity = 0.1;

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
  delay(5000);
  // bno.setExtCrystalUse(true);

  // updating range
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_CONFIG);
  write_register (0x28, 0x07, 0x01);
  write_register (0x28, 0x0A, 0b00110010);
  // printing register 1
  register_1 = read_register (0x28, 0x0A, 1);
  Serial.print(register_1);
  write_register (0x28, 0x07, 0x00);
  // bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_ACCGYRO);

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
