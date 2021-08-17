#include <SPI.h>
#include <stdio.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#include <Wire.h>
#include <MPU6050_tockn.h>
#include <MPU6050.h>
#include <FahmiKalmanFilter.h> // Source: https://github.com/fahmimsh/Auto-Rice-Clean-dengan-PID-dan-Kalman-Filter/tree/main/library

#define SD_CS 7
#define SCK 13
#define MOSI 11
#define RS 10
#define LCD_CS 8
#define RST 9
#define Bat 2

MPU6050_tockn mpu6050_1(Wire);
MPU6050 mpu6050_2;
unsigned long timer = 0;
float timeStep = 0.01;

float X12 = 0, Y12 = 0, Z12 = 0;
float Kal_imu1_roll = 0, Kal_imu1_pitch = 0, imu1_yaw = 0;
float Kal_imu2_roll = 0, Kal_imu2_pitch = 0, imu2_yaw = 0; 
 /*FahmiKalmanFilter(e_mea, e_est, q);
    e_mea: Measurement Uncertainty 
    e_est: Estimation Uncertainty 
    q: Process Noise */
    
FahmiKalmanFilter Slave1_kalmanX(0.001, 0.003, 0.03);
FahmiKalmanFilter Slave1_kalmanY(0.001, 0.003, 0.03);
FahmiKalmanFilter Slave2_kalmanX(0.001, 0.003, 0.03);
FahmiKalmanFilter Slave2_kalmanY(0.001, 0.003, 0.03);

Adafruit_ST7735 tft = Adafruit_ST7735(LCD_CS, RS, RST);
unsigned long time_prev = 0;

void setup() {  
  Serial.begin(115200);
  pinMode(Bat,INPUT);
  time_prev = millis();
  //lcd  tft
  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST7735_BLACK);
  tft.setTextWrap(false);
  //MULAI
  tft.setRotation(3);
  tft.setCursor(0, 1); //x,y
  tft.setTextColor(ST7735_RED);
  tft.setTextSize(2);
  tft.print("GYRO WPASS");
  tft.drawLine(3, 17, 118, 17, ST7735_CYAN);
  tft.drawRoundRect(137, 1, 22, 10, 5, ST7735_MAGENTA); //kotak
  tft.fillRoundRect(140, 1, 22, 10, 5, ST7735_MAGENTA);
  tft.drawChar(128, 1, 37, ST7735_MAGENTA, ST7735_BLACK, 1); //%

  tft.setTextColor(ST7735_GREEN);
  tft.setCursor(2, 22); //x,y
  tft.println("  CALIBRASI");
  tft.setCursor(2, 44); //x,y
  tft.println("   SENSOR");
  tft.setCursor(2, 66); //x,y
  tft.println(" SEJAJARKAN");
  tft.setCursor(2, 88); //x,y
  tft.println("   SENSOR");
  tft.setCursor(2, 110); //x,y
  tft.println("SLAVE1&SLAVE2");
  
  Wire.begin();
  mpu6050_1.begin();
  mpu6050_1.calcGyroOffsets(true);
  
  while(!mpu6050_2.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu6050_2.calibrateGyro();
  mpu6050_2.setThreshold(3);
  
  mpu6050_2.setAccelOffsetX(-4081);
  mpu6050_2.setAccelOffsetY(191);
  mpu6050_2.setAccelOffsetZ(1381);
  mpu6050_2.setGyroOffsetZ(35);
/*
  Data rata rata 10000 baca sensor setiap mikro secon per frekuensi
....................  XAccel      YAccel        ZAccel      XGyro     YGyro     ZGyro
 [-4081,-4078] --> [-8,21]  [191,192] --> [-8,7]  [1381,1382] --> [16377,16398] [-66,-65] --> [-1,3]  [176,177] --> [0,3] [35,36] --> [-2,2]
.................... [-4081,-4079] --> [-8,6] [191,192] --> [-8,7]  [1381,1382] --> [16379,16398] [-66,-65] --> [-1,3]  [176,177] --> [0,3] [35,36] --> [-1,2]
.................... [-4081,-4080] --> [-8,5] [191,192] --> [-8,7]  [1381,1382] --> [16379,16398] [-66,-65] --> [-1,3]  [176,177] --> [0,3] [35,36] --> [-1,2]
.................... [-4081,-4080] --> [-10,5]  [191,192] --> [-8,7]  [1381,1382] --> [16382,16398] [-66,-65] --> [-1,3]  [176,177] --> [0,3] [35,36] --> [-2,2]
-------------- done --------------
*/

  tft.fillRect(0, 20, 160, 110, ST7735_BLACK);  
  tft.drawLine(80, 19, 80, 128, ST7735_CYAN);
  tft.drawLine(1, 73, 80, 73, ST7735_CYAN); //55

  tft.setCursor(12, 22); //x,y
  tft.setTextColor(ST7735_BLUE);
  tft.setTextSize(1);
  tft.println("SLAVE 1");

  tft.setCursor(2, 34); //x,y
  tft.setTextColor(ST7735_BLUE);
  tft.setTextSize(1);
  tft.println(" X:");
  tft.setCursor(2, 46); //x,y
  tft.println(" Y:");
  tft.setCursor(2, 58); //x,y
  tft.println(" Z:");
  

  tft.setCursor(14, 78); //x,y
  tft.setTextColor(ST7735_BLUE);
  tft.setTextSize(1);
  tft.println("SLAVE 2");

  tft.setCursor(2, 90); //x,y
  tft.setTextColor(ST7735_BLUE);
  tft.setTextSize(1);
  tft.println(" X:");
  tft.setCursor(2, 102); //x,y
  tft.println(" Y:");
  tft.setCursor(2, 114); //x,y
  tft.println(" Z:");

  tft.setCursor(100, 22); //x,y
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(1);
  tft.println("RESULT");

  tft.setCursor(82, 34); //x,y
  tft.setTextColor(ST7735_CYAN);
  tft.setTextSize(1);
  tft.println(" X:");
  tft.setTextColor(ST7735_YELLOW);
  tft.setCursor(82, 67); //x,y
  tft.println(" Y:");
  tft.setTextColor(ST7735_MAGENTA);
  tft.setCursor(82, 100); //x,y
  tft.println(" Z:");
}
void loop() {
  //MPU 1
  mpu6050_1.update();
  //--------------------MPU 2
  Vector normAccel = mpu6050_2.readNormalizeAccel();
  Vector normGyro = mpu6050_2.readNormalizeGyro();

  int pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  int roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
  int yaw = yaw + normGyro.ZAxis * timeStep;

  Kal_imu1_roll = Slave1_kalmanY.updateEstimate(mpu6050_1.getAngleY());
  Kal_imu1_pitch = Slave1_kalmanX.updateEstimate(mpu6050_1.getAngleX());
  imu1_yaw = mpu6050_1.getAngleZ();
  Kal_imu2_roll = Slave2_kalmanY.updateEstimate(pitch);
  Kal_imu2_pitch = Slave2_kalmanX.updateEstimate(roll);
  imu2_yaw = yaw;

  X12 = Kal_imu1_pitch - Kal_imu2_pitch;
  Y12 = Kal_imu1_roll - Kal_imu2_roll;
  Z12 = imu1_yaw - imu2_yaw;
  
  //Serial Plotter
  Serial.print("IMU 1 --> Pitch(X): "); Serial.print(Kal_imu1_pitch); 
  Serial.print("  Rol(Y): "); Serial.print(Kal_imu1_roll);
  Serial.print("  Yaw(Z): "); Serial.print(imu1_yaw);
  Serial.print(" || || IMU 2 --> Pitch(X): "); Serial.print(Kal_imu2_pitch);
  Serial.print("  Rol(Y): "); Serial.print(Kal_imu2_roll);
  Serial.print("  Yaw(Z): "); Serial.println(imu2_yaw);

  //tft.drawChar(90, text_position + 20, 247, text_color, COLOR2, 1); //degree symbol
  
  //Slave1
  tft.fillRect(20, 34, 45, 34, ST7735_BLACK);
  tft.setCursor(20, 34); //x,y
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.println(Kal_imu1_pitch);
  tft.drawChar(60, 34, 247, ST7735_YELLOW, ST7735_BLACK, 1);
  
  tft.setCursor(20, 46); //x,y
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.println(Kal_imu1_roll);
  tft.drawChar(60, 46, 247, ST7735_YELLOW, ST7735_BLACK, 1);

  tft.setCursor(20, 58); //x,y
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.println(imu1_yaw);
  tft.drawChar(60, 58, 247, ST7735_YELLOW, ST7735_BLACK, 1);

  //Slave2
  tft.fillRect(20, 90, 45, 34, ST7735_BLACK);
  tft.setCursor(20, 90); //x,y
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.println(Kal_imu2_pitch);
  tft.drawChar(60, 90, 247, ST7735_YELLOW, ST7735_BLACK, 1);
  
  tft.setCursor(20, 102); //x,y
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.println(Kal_imu2_roll);
  tft.drawChar(60, 102, 247, ST7735_YELLOW, ST7735_BLACK, 1);

  tft.setCursor(20, 114); //x,y
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.println(imu2_yaw);
  tft.drawChar(60, 114, 247, ST7735_YELLOW, ST7735_BLACK, 1);

  //RESULT
  //xxxxxxxxx
  tft.fillRect(100, 34, 45, 10, ST7735_BLACK);
  tft.setCursor(100, 34); //x,y
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.println(X12);
  tft.drawChar(140, 34, 247, ST7735_YELLOW, ST7735_BLACK, 1);

  if (Kal_imu1_pitch <= 3 && Kal_imu1_pitch >= -3 && Kal_imu2_pitch <= 3 && Kal_imu2_pitch >= -3){
    tft.fillRect(82, 44, 60, 20, ST7735_BLACK);
    tft.setCursor(100, 44); //x,y
    tft.setTextColor(ST7735_GREEN);
    tft.setTextSize(1);
    tft.println("SUDAH");
    tft.setCursor(95, 54); //x,y
    tft.println("SEJAJAR");
  } else if(Kal_imu1_pitch <= 10 && Kal_imu1_pitch >= 4 || Kal_imu2_pitch <= -4 && Kal_imu2_pitch >= -10){
      
    tft.fillRect(82, 44, 60, 20, ST7735_BLACK);
    tft.setCursor(100, 44); //x,y
    tft.setTextColor(ST7735_YELLOW);
    tft.setTextSize(1);
    tft.println("HAMPIR");
    tft.setCursor(95, 54); //x,y
    tft.println("SEJAJAR");
  } else if(Kal_imu1_pitch >= 20 || Kal_imu1_pitch <= -20 || Kal_imu2_pitch >= 20 || Kal_imu2_pitch <= -20){
    tft.fillRect(82, 44, 60, 20, ST7735_BLACK);
    tft.setCursor(100, 44); //x,y
    tft.setTextColor(ST7735_RED);
    tft.setTextSize(1);
    tft.println("TIDAK");
    tft.setCursor(95, 54); //x,y
    tft.println("SEJAJAR");
  }
  
  //yyyyyyyyy
  tft.fillRect(100, 67, 45, 10, ST7735_BLACK);
  tft.setCursor(100, 67); //x,y
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.println(Y12);
  tft.drawChar(140, 67, 247, ST7735_YELLOW, ST7735_BLACK, 1);

  if (Kal_imu1_roll <= 3 && Kal_imu1_roll >= -3 && Kal_imu2_roll <= 3 && Kal_imu2_roll >= -3){
    tft.fillRect(82, 77, 60, 20, ST7735_BLACK);
    tft.setCursor(100, 77); //x,y
    tft.setTextColor(ST7735_GREEN);
    tft.setTextSize(1);
    tft.println("SUDAH");
    tft.setCursor(95, 87); //x,y
    tft.println("SEJAJAR");
  } else if(Kal_imu1_roll <= 10 && Kal_imu1_roll >= 4 || Kal_imu2_roll <= -4 && Kal_imu2_roll >= -10){
    tft.fillRect(82, 77, 60, 20, ST7735_BLACK);
    tft.setCursor(100, 77); //x,y
    tft.setTextColor(ST7735_YELLOW);
    tft.setTextSize(1);
    tft.println("HAMPIR");
    tft.setCursor(95, 87); //x,y
    tft.println("SEJAJAR");
  } else if(Kal_imu1_roll >= 20 || Kal_imu1_roll <= -20 || Kal_imu2_roll >= 20 || Kal_imu2_roll <= -20){
    tft.fillRect(82, 77, 60, 20, ST7735_BLACK);
    tft.setCursor(100, 77); //x,y
    tft.setTextColor(ST7735_RED);
    tft.setTextSize(1);
    tft.println("TIDAK");
    tft.setCursor(95, 87); //x,y
    tft.println("SEJAJAR");
  }
  
  //zzzzzzzzz
  tft.fillRect(100, 100, 45, 10, ST7735_BLACK);
  tft.setCursor(100, 100); //x,y
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.println(Z12);
  tft.drawChar(140, 100, 247, ST7735_YELLOW, ST7735_BLACK, 1);

  if (imu1_yaw <= 3 && imu1_yaw >= -3 && imu1_yaw <= 3 && imu1_yaw >= -3){
    tft.fillRect(82, 110, 60, 20, ST7735_BLACK);
    tft.setCursor(100, 110); //x,y
    tft.setTextColor(ST7735_GREEN);
    tft.setTextSize(1);
    tft.println("SUDAH");
    tft.setCursor(95, 120); //x,y
    tft.println("SEJAJAR");
  } else if(imu1_yaw <= 10 && imu1_yaw >= 4 || imu1_yaw <= -4 && imu1_yaw >= -10){
    tft.fillRect(82, 110, 60, 20, ST7735_BLACK);
    tft.setCursor(100, 110); //x,y
    tft.setTextColor(ST7735_YELLOW);
    tft.setTextSize(1);
    tft.println("HAMPIR");
    tft.setCursor(95, 120); //x,y
    tft.println("SEJAJAR");
  } else if(imu1_yaw >= 20 || imu1_yaw <= -20 || imu1_yaw >= 20 || imu1_yaw <= -20){
    tft.fillRect(82, 110, 60, 20, ST7735_BLACK);
    tft.setCursor(100, 110); //x,y
    tft.setTextColor(ST7735_RED);
    tft.setTextSize(1);
    tft.println("TIDAK");
    tft.setCursor(95, 120); //x,y
    tft.println("SEJAJAR");
  }

  //kondisi
  if(digitalRead(Bat)>0){
    tft.setCursor(120, 11); //x,y
    tft.setTextColor(ST7735_YELLOW);
    tft.setTextSize(1);
    tft.println("Charger");
  } else {
    tft.fillRect(120, 11, 40, 9, ST7735_BLACK);
  }
  if (millis() - time_prev >= 3600000){
    tft.fillRect(140, 1, 40, 9, ST7735_BLACK);
    tft.fillRoundRect(145, 1, 22, 9, 5, ST7735_MAGENTA);
  } else if(millis() - time_prev >= 7200000){
    tft.fillRect(140, 1, 40, 9, ST7735_BLACK);
    tft.fillRoundRect(150, 1, 22, 9, 5, ST7735_MAGENTA);
  }else if(millis() - time_prev >= 10800000){
    tft.fillRect(140, 1, 40, 9, ST7735_BLACK);
    tft.fillRoundRect(155, 1, 22, 9, 5, ST7735_MAGENTA);
  }
  delay(5);
}
