#undef max
#undef min

#include <ThingerWiFiNINA.h>
#include "wiring_private.h"
#include <TinyGPS.h>
#include <Arduino_LSM6DS3.h>

#define USERNAME "USERNAME"
#define DEVICE_ID "DEVICE_ID"
#define DEVICE_CREDENTIAL "DEVICE_CREDENTIAL"

#define SSID "SSID-NAME"
#define SSID_PASSWORD "SSID-PASSWORD"

ThingerWiFiNINA thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);

TinyGPS gps;
Uart ss (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// Attach the interrupt handler to the SERCOM
void SERCOM0_Handler()
{
  ss.IrqHandler();
}

float xA, yA, zA;
float xG, yG, zG;
float Vone, Vtwo, Vthree, Vtotal;
float windowSize = 3; //Window Size in Seconds
float Min, Max;
int threshold1 = 2.0, threshold2 = 4.54; //Thresholds of accident detection. Before threshold2 value was 7.72
float startTime, endTime;
String cValue, nValue;
int detected = 0;

void setup() {
  // configure wifi network
  thing.add_wifi(SSID, SSID_PASSWORD);

  // Reassign pins 5 and 6 to SERCOM alt
  pinPeripheral(5, PIO_SERCOM_ALT);
  pinPeripheral(6, PIO_SERCOM_ALT);
  Serial.begin(9600);
  ss.begin(9600);

  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }

  //Displaying the data to the platform
  thing["deviceInfo"] >> [](pson & out) {
    out["lat"] = Latitude();
    out["lon"] = Longitude();
    out["date"] = Date();
    out["spd"] = speed();
    out["xA"] = xA;
    out["yA"] = yA;
    out["zA"] = zA;
    out["xG"] = xG;
    out["yG"] = yG;
    out["zG"] = zG;
    out["velocity_3d"] = Vtotal;
    out["detectAccident"] = nValue;
  };
}
void AccandGyro() {
  IMU.readAcceleration(xA, yA, zA);     //Initializing Accelerometer
  IMU.readGyroscope(xG, yG, zG);      //Initializing Gyroscope
}

void loop() {
  //Handling the connection
  thing.handle();

  //Run the loop only when there is valid data
  if (Latitude() != 1000 && Longitude() != 1000) {
    Serial.println("The location");
    Serial.print(Latitude());
    Serial.print(" ,,,,,,, ");
    Serial.println(Longitude());
    Serial.println("The date and time");
    Serial.println(Date());
    Serial.println("The speed");
    Serial.println(speed());
    Serial.println("Acceleration: (xA yA zA) ");
    Acc();
    Serial.print(xA);
    Serial.print('\t');
    Serial.print(yA);
    Serial.print('\t');
    Serial.println(zA);

    Serial.print('\n');

    Serial.println("Gyroscope: (xG yG zG)");
    Gyro();
    Serial.print(xG);
    Serial.print('\t');
    Serial.print(yG);
    Serial.print('\t');
    Serial.println(zG);
    Serial.print('\n');
    Serial.println("Total Velocity: ");
    Serial.println(findV());
    Serial.println(examineForAccident());
    Serial.print("----------------------------");
    Serial.print('\n');
  }
  else
    Serial.println("No Signal");

  smartdelay(2000);
}

//Function for getting the accelerometer values
static float Acc() {
  IMU.readAcceleration(xA, yA, zA);
  return xA, yA, zA;
}

//Function for getting the gyroscope values
static float Gyro() {
  IMU.readGyroscope(xG, yG, zG);
  return xG, yG, zG;
}

//Delay function
static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    //check is signal is available
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

//Optional (extra spaces)
static void spaces(int n) {
  //create spaces
  for (int i = 0; i < 5; i++)
    Serial.print(" ");
}

//Latitude Function
static float Latitude() {
  //print the latitude
  float flat, flon;
  gps.f_get_position(&flat, &flon);
  return flat;
}

//Longitude Function
static float Longitude() {
  //print the Longitude
  float flat, flon;
  gps.f_get_position(&flat, &flon);
  return flon;
}

//Date & Time Function
static String Date() {
  //return string of date and time
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  byte convHour = hour + 4;
  if (convHour >= 24)
    convHour = convHour - 24;
  return "Date: " + String(day) + "/" + String(month) + "/" + String(year) + " Time: " + String(convHour) + ":" + String(minute) + ":" + String(second);
}

//Function for getting the speed
static float speed() {
  float speed = 0;
  speed = gps.f_speed_kmph();
  if (speed <= 0)
    return 0;
  else
    return speed;
}

static float findV() { //Find Vtotal by adding values in 3D
  AccandGyro();
  Vone = xA * sin(zG);
  Vtwo = yA * sin(yG);
  Vthree = zA * cos(yG) * cos(zG);
  Vtotal = Vone + Vtwo + Vthree;
  return Vtotal;
}
static String checkAccident() {  //Checking for accident occurance
  findV();
  Min = 1023; Max = 0;
  if (abs(Vtotal) > threshold1) {
    Max = abs(Vtotal);  //Setting initial value as Max
    windowEnd(windowSize);
    while (millis() < endTime) {
      if (Vtotal > Max) { //Find Max within Window
        Max = Vtotal;
      }
      if (Vtotal < Min) { //Finding Min within Window
        Min = Vtotal;
      }
    }
    if ((Max - Min) >= threshold2 && abs(yA) >= 0.9) { //Checking parameters within Created window
      return "Accident Detected!";
    }
    else {
      return "No Accident";
    }
  }
  else
    return "No Accident";
}
static void windowEnd(float windowSize) {  //Calculation of End if windows
  startTime = millis();
  endTime = windowSize * 1000 + startTime;
}

String examineForAccident() {
  if (detected)
    delay(10000);
  nValue = checkAccident();
  if (nValue.equals("Accident Detected!") && !cValue.equals("Accident Detected!")) {
    return "Accident Detected!";
    detected = 1;
  }
  else if (nValue.equals("No Accident") && !cValue.equals("No Accident")) {
    detected = 0;
    return "No Accident";
  }
  else
  {}
  cValue = nValue;
}
