/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to 
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and 
 Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.
 
 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 
 Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library. 
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */
 
//#include "ST_F401_84MHZ.h" 
//F401_init84 myinit(0);
#include "mbed.h"
#include "MPU9250.h"
#include "BMP280.h"
#include "N5110.h"
#include "math.h"

// Using NOKIA 5110 monochrome 84 x 48 pixel display
// pin 9 - Serial clock out (SCLK)
// pin 8 - Serial data out (DIN)
// pin 7 - Data/Command select (D/C)
// pin 5 - LCD chip select (CS)
// pin 6 - LCD reset (RST)
//Adafruit_PCD8544 display = Adafruit_PCD8544(9, 8, 7, 5, 6);

  MPU9250 mpu9250;  // Instantiate MPU9250 class
   
   BMP280 bmp280;   // Instantiate BMP280 class
   
   Timer t;


 //  Serial pc(USBTX, USBRX); // tx, rx
  Serial pc(P0_9, P0_11); // tx, rx

   //        VCC,   SCE,  RST,    D/C,  MOSI, SCLK, LED
 //  N5110 lcd(P0_0, P0_1, P0_2, P0_3, P0_4, P0_5, P0_6);
   

float sum = 0;
uint32_t sumCount = 0;
char buffer[14];
uint8_t whoami = 0;
double Temperature, Pressure; // stores BMP280 pressures sensor pressure and temperature
int32_t rawPress, rawTemp;   // pressure and temperature raw count output for BMP280

int32_t readBMP280Temperature()
{
  uint8_t rawData[3];  // 20-bit pressure register data stored here
  bmp280.readBytes(BMP280_ADDRESS, BMP280_TEMP_MSB, 3, &rawData[0]);  
  return (int32_t) (((int32_t) rawData[0] << 16 | (int32_t) rawData[1] << 8 | rawData[2]) >> 4);
}

int32_t readBMP280Pressure()
{
  uint8_t rawData[3];  // 20-bit pressure register data stored here
  bmp280.readBytes(BMP280_ADDRESS, BMP280_PRESS_MSB, 3, &rawData[0]);  
  return (int32_t) (((int32_t) rawData[0] << 16 | (int32_t) rawData[1] << 8 | rawData[2]) >> 4);
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of
// “5123” equals 51.23 DegC.
int32_t bmp280_compensate_T(int32_t adc_T)
{
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8
//fractional bits).
//Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t bmp280_compensate_P(int32_t adc_P)
{
  long long var1, var2, p;
  var1 = ((long long)t_fine) - 128000;
  var2 = var1 * var1 * (long long)dig_P6;
  var2 = var2 + ((var1*(long long)dig_P5)<<17);
  var2 = var2 + (((long long)dig_P4)<<35);
  var1 = ((var1 * var1 * (long long)dig_P3)>>8) + ((var1 * (long long)dig_P2)<<12);
  var1 = (((((long long)1)<<47)+var1))*((long long)dig_P1)>>33;
  if(var1 == 0)
  {
    return 0;
    // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125)/var1;
  var1 = (((long long)dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((long long)dig_P8) * p)>> 19;
  p = ((p + var1 + var2) >> 8) + (((long long)dig_P7)<<4);
  return (uint32_t)p;
}

 

        
int main()
{
  pc.baud(9600);  
  myled = 1;
  
  wait(15);
  
  //Set up I2C
  i2c.frequency(400000);  // use fast (400 kHz) I2C  
  
  pc.printf("CPU SystemCoreClock is %d Hz\r\n", SystemCoreClock);   
  
  t.start();        
  
 // lcd.init();
//  lcd.setBrightness(0.05);
  
    myled = 0;
    
  // Read the WHO_AM_I register, this is a good test of communication
  whoami = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  pc.printf("I AM 0x%x\n\r", whoami); pc.printf("I SHOULD BE 0x71\n\r");
  myled = 1;
  if (whoami == 0x71) // WHO_AM_I should always be 0x71
  {  
    pc.printf("MPU9250 WHO_AM_I is 0x%x\n\r", whoami);
    pc.printf("MPU9250 is online...\n\r");
 //   lcd.clear();
 //   lcd.printString("MPU9250 is", 0, 0);
  //  sprintf(buffer, "0x%x", whoami);
 //   lcd.printString(buffer, 0, 1);
 //   lcd.printString("shoud be 0x71", 0, 2);  
    wait(1);
    
    mpu9250.resetMPU9250(); // Reset registers to default in preparation for device calibration
    mpu9250.MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
    pc.printf("x-axis self test: acceleration trim within : %f % of factory value\n\r", SelfTest[0]);  
    pc.printf("y-axis self test: acceleration trim within : %f % of factory value\n\r", SelfTest[1]);  
    pc.printf("z-axis self test: acceleration trim within : %f % of factory value\n\r", SelfTest[2]);  
    pc.printf("x-axis self test: gyration trim within : %f % of factory value\n\r", SelfTest[3]);  
    pc.printf("y-axis self test: gyration trim within : %f % of factory value\n\r", SelfTest[4]);  
    pc.printf("z-axis self test: gyration trim within : %f % of factory value\n\r", SelfTest[5]);  
    mpu9250.calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
    pc.printf("x gyro bias = %f\n\r", gyroBias[0]);
    pc.printf("y gyro bias = %f\n\r", gyroBias[1]);
    pc.printf("z gyro bias = %f\n\r", gyroBias[2]);
    pc.printf("x accel bias = %f\n\r", accelBias[0]);
    pc.printf("y accel bias = %f\n\r", accelBias[1]);
    pc.printf("z accel bias = %f\n\r", accelBias[2]);
    wait(2);
    mpu9250.initMPU9250(); 
    pc.printf("MPU9250 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    mpu9250.initAK8963(magCalibration);
    pc.printf("AK8963 initialized for active data mode....\n\r"); // Initialize device for active mode read of magnetometer
    pc.printf("Accelerometer full-scale range = %f  g\n\r", 2.0f*(float)(1<<Ascale));
    pc.printf("Gyroscope full-scale range = %f  deg/s\n\r", 250.0f*(float)(1<<Gscale));
    if(Mscale == 0) pc.printf("Magnetometer resolution = 14  bits\n\r");
    if(Mscale == 1) pc.printf("Magnetometer resolution = 16  bits\n\r");
    if(Mmode == 2) pc.printf("Magnetometer ODR = 8 Hz\n\r");
    if(Mmode == 6) pc.printf("Magnetometer ODR = 100 Hz\n\r");
    wait(1);
   }
   else
   {
    pc.printf("Could not connect to MPU9250: \n\r");
    pc.printf("%#x \n",  whoami);
    myled = 0;
 
  //  lcd.clear();
   // lcd.printString("MPU9250", 0, 0);
  //  lcd.printString("no connection", 0, 1);
   // sprintf(buffer, "WHO_AM_I 0x%x", whoami);
  //  lcd.printString(buffer, 0, 2); 
 
    while(1) ; // Loop forever if communication doesn't happen
    }

    mpu9250.getAres(); // Get accelerometer sensitivity
    mpu9250.getGres(); // Get gyro sensitivity
    mpu9250.getMres(); // Get magnetometer sensitivity
    pc.printf("Accelerometer sensitivity is %f LSB/g \n\r", 1.0f/aRes);
    pc.printf("Gyroscope sensitivity is %f LSB/deg/s \n\r", 1.0f/gRes);
    pc.printf("Magnetometer sensitivity is %f LSB/G \n\r", 1.0f/mRes);
    magbias[0] = +0.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    magbias[1] = +0.;  // User environmental x-axis correction in milliGauss
    magbias[2] = +0.;  // User environmental x-axis correction in milliGauss
    
    // Read the WHO_AM_I register of the BMP-280, this is a good test of communication
    uint8_t c = bmp280.readByte(BMP280_ADDRESS, BMP280_ID);   
    if(c == 0x58) {
 
    pc.printf("BMP-280 is 0x%x\n\r", c);
    pc.printf("BMP-280 should be 0x58\n\r");
    pc.printf("BMP-280 online...\n\r");
   
    //bmp280.BMP280Init();
    
  // Set T and P oversampling rates and sensor mode
  bmp280.writeByte(BMP280_ADDRESS, BMP280_CTRL_MEAS, Tosr << 5 | Posr << 2 | Mode);
  // Set standby time interval in normal mode and bandwidth
  bmp280.writeByte(BMP280_ADDRESS, BMP280_CONFIG, SBy << 5 | IIRFilter << 2);
  uint8_t calib[24];
  bmp280.readBytes(BMP280_ADDRESS, BMP280_CALIB00, 24, &calib[0]);
  dig_T1 = (uint16_t)(((uint16_t) calib[1] << 8) | calib[0]);
  dig_T2 = ( int16_t)((( int16_t) calib[3] << 8) | calib[2]);
  dig_T3 = ( int16_t)((( int16_t) calib[5] << 8) | calib[4]);
  dig_P1 = (uint16_t)(((uint16_t) calib[7] << 8) | calib[6]);
  dig_P2 = ( int16_t)((( int16_t) calib[9] << 8) | calib[8]);
  dig_P3 = ( int16_t)((( int16_t) calib[11] << 8) | calib[10]);
  dig_P4 = ( int16_t)((( int16_t) calib[13] << 8) | calib[12]);
  dig_P5 = ( int16_t)((( int16_t) calib[15] << 8) | calib[14]);
  dig_P6 = ( int16_t)((( int16_t) calib[17] << 8) | calib[16]);
  dig_P7 = ( int16_t)((( int16_t) calib[19] << 8) | calib[18]);
  dig_P8 = ( int16_t)((( int16_t) calib[21] << 8) | calib[20]);
  dig_P9 = ( int16_t)((( int16_t) calib[23] << 8) | calib[22]);

  pc.printf("dig_T1 is %d\n\r", dig_T1);
  pc.printf("dig_T2 is %d\n\r", dig_T2);
  pc.printf("dig_T3 is %d\n\r", dig_T3);
  pc.printf("dig_P1 is %d\n\r", dig_P1);
  pc.printf("dig_P2 is %d\n\r", dig_P2);
  pc.printf("dig_P3 is %d\n\r", dig_P3);
  pc.printf("dig_P4 is %d\n\r", dig_P4);
  pc.printf("dig_P5 is %d\n\r", dig_P5);
  pc.printf("dig_P6 is %d\n\r", dig_P6);
  pc.printf("dig_P7 is %d\n\r", dig_P7);
  pc.printf("dig_P8 is %d\n\r", dig_P8);
  pc.printf("dig_P9 is %d\n\r", dig_P9);
  
    pc.printf("BMP-280 calibration complete...\n\r");
   }
   else 
   {
    pc.printf("BMP-280 is 0x%x\n\r", c);
    pc.printf("BMP-280 should be 0x55\n\r");
    while(1); // idle here forever
   }
    
 while(1) {
  
  // If intPin goes high, all data registers have new data
  if(mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt

    mpu9250.readAccelData(accelCount);  // Read the x/y/z adc values   
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes - accelBias[1];   
    az = (float)accelCount[2]*aRes - accelBias[2]; 
    
  //   pc.printf("reached here!...\n\r"); 
    
    mpu9250.readGyroData(gyroCount);  // Read the x/y/z adc values
    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes - gyroBias[1];  
    gz = (float)gyroCount[2]*gRes - gyroBias[2];   
  
    mpu9250.readMagData(magCount);  // Read the x/y/z adc values   
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];  
    mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];   
  }
   
    Now = t.read_us();
    deltat = (float)((Now - lastUpdate)/1000000.0f) ; // set integration time by time elapsed since last filter update
    lastUpdate = Now;
    
    sum += deltat;
    sumCount++;
    
//    if(lastUpdate - firstUpdate > 10000000.0f) {
//     beta = 0.04;  // decrease filter gain after stabilized
//     zeta = 0.015; // increasey bias drift gain after stabilized
 //   }
    
   // Pass gyro rate as rad/s
//  mpu9250.MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);

    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = t.read_ms() - count;
    if (delt_t > 500) { // update LCD once per half-second independent of read rate

    pc.printf("ax = %f", 1000*ax); 
    pc.printf(" ay = %f", 1000*ay); 
    pc.printf(" az = %f  mg\n\r", 1000*az); 

    pc.printf("gx = %f", gx); 
    pc.printf(" gy = %f", gy); 
    pc.printf(" gz = %f  deg/s\n\r", gz); 
    
    pc.printf("gx = %f", mx); 
    pc.printf(" gy = %f", my); 
    pc.printf(" gz = %f  mG\n\r", mz); 
    
    tempCount = mpu9250.readTempData();  // Read the adc values
    temperature = ((float) tempCount) / 333.87f + 21.0f; // Temperature in degrees Centigrade
    pc.printf(" temperature = %f  C\n\r", temperature); 
    
    pc.printf("q0, q1, q2, q3 = %f %f %f %f\n\r",q[0], q[1], q[2], q[3]);
    
    rawPress =  readBMP280Pressure();
    Pressure = (float) bmp280_compensate_P(rawPress)/25600.; // Pressure in mbar
    rawTemp =   readBMP280Temperature();
    Temperature = (float) bmp280_compensate_T(rawTemp)/100.;

  //  float altitude = 145366.45f*(1.0f - pow(Pressure/1013.25f, 0.190284f) );
    pc.printf("Temperature = %f C\n\r", Temperature);
    pc.printf("Pressure = %f Pa\n\r", Pressure); 
    
/*    lcd.clear();
    lcd.printString("MPU9250", 0, 0);
    lcd.printString("x   y   z", 0, 1);
    sprintf(buffer, "%d %d %d mg", (int)(1000.0f*ax), (int)(1000.0f*ay), (int)(1000.0f*az));
    lcd.printString(buffer, 0, 2);
    sprintf(buffer, "%d %d %d deg/s", (int)gx, (int)gy, (int)gz);
    lcd.printString(buffer, 0, 3);
    sprintf(buffer, "%d %d %d mG", (int)mx, (int)my, (int)mz);
    lcd.printString(buffer, 0, 4); 
 */  
  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    roll  *= 180.0f / PI;

    pc.printf("Yaw, Pitch, Roll: %f %f %f\n\r", yaw, pitch, roll);
    pc.printf("average rate = %f\n\r", (float) sumCount/sum);
//    sprintf(buffer, "YPR: %f %f %f", yaw, pitch, roll);
//    lcd.printString(buffer, 0, 4);
//    sprintf(buffer, "rate = %f", (float) sumCount/sum);
//    lcd.printString(buffer, 0, 5);
    
    myled= !myled;
    count = t.read_ms(); 

    if(count > 1<<21) {
        t.start(); // start the timer over again if ~30 minutes has passed
        count = 0;
        deltat= 0;
        lastUpdate = t.read_us();
    }
    sum = 0;
    sumCount = 0; 
}

}
 
 }
