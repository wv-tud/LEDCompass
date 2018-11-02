#ifndef ARDUINO
#define ARDUINO 101
#endif

#define MPU6050_DMP_FIFO_RATE_DIVISOR 0x27 //  200 Hz / (1 + value)

#include "src/Adafruit_NeoPixel/Adafruit_NeoPixel.h"
#include "src/AltSoftSerial/AltSoftSerial.h"
#include "src/NeoGPS/src/NMEAGPS.h"
#include "src/NeoGPS/src/Streamers.h"
#include <Wire.h>
#include "src/Adafruit_Sensor/Adafruit_Sensor.h"
#include "src/Adafruit_HMC5883_Unified/Adafruit_HMC5883_U.h"
#include "src/i2cdevlib/Arduino/I2Cdev/I2Cdev.h"
#include "src/i2cdevlib/Arduino/MPU6050/MPU6050_6Axis_MotionApps20.h"

// LED ring
#define NEOPIXEL_PIN 5
#define NEOPIXEL_SIZE 32
Adafruit_NeoPixel NeoPixel = Adafruit_NeoPixel(NEOPIXEL_SIZE, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// GPS
#define GPS_PORT_NAME "AltSoftSerial"
#define UBX_M8N_BAUD 9600
static gps_fix GPS_measurement;
NMEAGPS sen_UBX_M8N;
AltSoftSerial UBX_M8N_port; // 8 & 9 for an UNO

// Mag
#define HMC5883_SENSOR_ID 12345
Adafruit_HMC5883_Unified sen_HMC5883 = Adafruit_HMC5883_Unified(HMC5883_SENSOR_ID);
VectorFloat MAG_measurement(0.0, 0.0, 0.0);
float headingDegrees;

#define MAG_CALIB_UKF_NORM 49.1904
const float H[3] = {18.9300,	0.4289,	45.4000 };
const float HMC5883_bias[3] = {9.185, -9.82, 16.53};

/* MPU
 * AFS_SEL | Full Scale Range | LSB Sensitivity (setFullScaleAccelRange)
 * --------+------------------+----------------
 * 0       | +/- 2g           | 8192 LSB/mg
 * 1       | +/- 4g           | 4096 LSB/mg
 * 2       | +/- 8g           | 2048 LSB/mg
 * 3       | +/- 16g          | 1024 LSB/mg
 * 
 * FS_SEL | Full Scale Range   | LSB Sensitivity (setFullScaleGyroRange)
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 * 
 *          |   ACCELEROMETER    |           GYROSCOPE (setDLPFMode, setRate)
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
 */
#define MPU6050_ACC_RANGE MPU6050_ACCEL_FS_2
#define MPU6050_GYRO_RANGE MPU6050_GYRO_FS_2000
#define MPU6050_DLPF MPU6050_DLPF_BW_188
#define MPU6050_GYRO_SMPLRT 0 // Gyroscope Output Rate / (1 + SMPLRT_DIV)
const float MPU6050_ACC_SENS[4] = {8192, 4069, 2048, 1024};
const float MPU6050_GYRO_SENS[4] = {131, 65.5, 32.8, 16.4};

float AccelSens = MPU6050_ACC_SENS[MPU6050_ACC_RANGE];
float GyroSens = MPU6050_GYRO_SENS[MPU6050_GYRO_RANGE];

MPU6050 sen_MPU6050;
VectorFloat ACC_measurement;
VectorFloat GYRO_measurement;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 ACC_raw;  // [x, y, z]            accel sensor measurements
VectorInt16 GYRO_raw; // [x, y, z]            gravity-free accel sensor measurements
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

// Onboard LED
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// Setup
void setup()
{
  // MPU
  MPU6050_init();
  // GPS + Compass
  UBX_M8N_init();
  HMC5883_init();
  // NeoPixel
  NeoPixel_init();
  // configure Build-in LED for output
  pinMode(LED_PIN, OUTPUT);
}

void MPU6050_init()
{
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(UBX_M8N_BAUD);
  while (!Serial)
    ; // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  sen_MPU6050.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(sen_MPU6050.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = sen_MPU6050.dmpInitialize();

  sen_MPU6050.setIntFIFOBufferOverflowEnabled(true);
  sen_MPU6050.setIntDataReadyEnabled(true);
  sen_MPU6050.setFullScaleAccelRange(MPU6050_ACC_RANGE);
  sen_MPU6050.setFullScaleGyroRange(MPU6050_GYRO_RANGE);
  sen_MPU6050.setDLPFMode(MPU6050_DLPF);

  // supply your own gyro offsets here, scaled for min sensitivity
  /*sen_MPU6050.setXGyroOffset(220);
  sen_MPU6050.setYGyroOffset(76);
  sen_MPU6050.setZGyroOffset(-85);
  sen_MPU6050.setZAccelOffset(1788); // 1688 factory default for my test chip*/

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    sen_MPU6050.setDMPEnabled(true);
    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void UBX_M8N_init()
{
  UBX_M8N_port.begin(UBX_M8N_BAUD);
}

void HMC5883_init()
{
  sensor_t sensor;
  sen_HMC5883.getSensor(&sensor);

  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" uT");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" uT");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
}

void NeoPixel_init()
{
  NeoPixel.begin();
  NeoPixel.setBrightness(30); //adjust brightness here
  NeoPixel.show();            // Initialize all pixels to 'off'
}

int time;
float min = 0.05;
float max = 1.0;

void loop()
{
  // Read sensor data
  UBX_M8N_read();
  HMC5883_read();
  MPU6050_read();

  Serial.print("Heading: ");
  Serial.print(headingDegrees);
  Serial.print(" deg");
  Serial.print("  [");
  Serial.print(MAG_measurement.x);
  Serial.print(" ");
  Serial.print(MAG_measurement.y);
  Serial.print(" ");
  Serial.print(MAG_measurement.z);
  Serial.println("]");

  Serial.print("Accel: ");
  Serial.print(ACC_measurement.x);
  Serial.print(" ");
  Serial.print(ACC_measurement.y);
  Serial.print(" ");
  Serial.print(ACC_measurement.z);
  Serial.println();

  Serial.print("Gyro: ");
  Serial.print(GYRO_measurement.x);
  Serial.print(" ");
  Serial.print(GYRO_measurement.y);
  Serial.print(" ");
  Serial.print(GYRO_measurement.z);
  Serial.println();
  
  // Set pixels
  NeoPixel_set();
  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}

void NeoPixel_set()
{
  time = millis();
  for (int i = 0; i < NeoPixel.numPixels(); i++)
  {
    float ledDeg = (i * 1.0 - 16.0) * 50.0 / 16.0;
    float magDeg = headingDegrees;
    if(magDeg > 180.0) magDeg -= 360.0;
    float brightness = 1.0 - abs((magDeg - ledDeg) / 90.0);
    if (brightness < min)
    {
      brightness = min;
    }
    if (brightness > max)
    {
      brightness = max;
    }
    NeoPixel.setPixelColor(i, NeoPixel.Color(round(255.0 * brightness), round(255.0 * brightness * brightness), round(255.0 * brightness * brightness)));
  }
  NeoPixel.show();
}

void MPU6050_read()
{
  // if programming failed, don't try to do anything
  if (!dmpReady)
  {
    Serial.println(F("MPU6050 DMP error"));
    return;
  }
  // check for overflow (this should never happen unless our code is too inefficient)
  if (sen_MPU6050.getIntFIFOBufferOverflowStatus())
  {
    sen_MPU6050.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }
  else if (sen_MPU6050.getIntDataReadyStatus() && sen_MPU6050.dmpPacketAvailable())
  {
    sen_MPU6050.getFIFOBytes(fifoBuffer, sen_MPU6050.dmpGetFIFOPacketSize());

    sen_MPU6050.dmpGetAccel(&ACC_raw, fifoBuffer);
    sen_MPU6050.dmpGetGyro(&GYRO_raw, fifoBuffer);
    sen_MPU6050.dmpGetQuaternion(&q, fifoBuffer);

    ACC_measurement.x = ((float)ACC_raw.x) / AccelSens;
    ACC_measurement.y = ((float)ACC_raw.y) / AccelSens;
    ACC_measurement.z = ((float)ACC_raw.z) / AccelSens;

    GYRO_measurement.x = ((float)GYRO_raw.x) / GyroSens;
    GYRO_measurement.y = ((float)GYRO_raw.y) / GyroSens;
    GYRO_measurement.z = ((float)GYRO_raw.z) / GyroSens;
  }
}

void UBX_M8N_read()
{
  if (sen_UBX_M8N.available(UBX_M8N_port) > 0)
  {
    GPS_measurement = sen_UBX_M8N.read();
    //trace_all(Serial, sen_UBX_M8N, GPS_measurement);
  }
}

void HMC5883_read()
{
  if (sen_HMC5883.begin())
  {
    /* Get a new sensor event */
    sensors_event_t event;
    sen_HMC5883.getEvent(&event);

    MAG_measurement.x = event.magnetic.x - HMC5883_bias[0];
    MAG_measurement.y = event.magnetic.y - HMC5883_bias[1];
    MAG_measurement.z = event.magnetic.z - HMC5883_bias[2];
    //MAG_measurement.rotate(&q);

    // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    float heading = atan2(MAG_measurement.y - H[1], MAG_measurement.x - H[0]);

    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.

    // Velden: +1° 44'
    float declinationAngle = 0.0;
    heading += declinationAngle;

    // Correct for when signs are reversed.
    if (heading < 0)
      heading += 2 * PI;

    // Check for wrap due to addition of declination.
    if (heading > 2 * PI)
      heading -= 2 * PI;

    // Convert radians to degrees for readability.
    headingDegrees = heading * 180 / M_PI;

    //Serial.print("Heading (degrees): ");
    //Serial.println(headingDegrees);
  }
}
