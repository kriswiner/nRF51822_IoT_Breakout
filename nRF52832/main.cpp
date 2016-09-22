* MPU9250 Basic Example Code
 by: Kris Winer
 date: September 20, 2016
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to 
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and 
 Mahony filter algorithms. 
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 */
 
#include "mbed.h"
#include "MPU9250.h"
#include "BMP280.h"
#include "math.h"

   MPU9250 mpu9250;  // Instantiate MPU9250 class
   
   BMP280 bmp280;   // Instantiate BMP280 class
   
   Timer t;
   
   InterruptIn myInterrupt(P0_8); // One nRF52 Dev Board variant uses pin 8, one uses pin 10

/* Serial pc(USBTX, USBRX); // tx, rx*/
  Serial pc(P0_12, P0_14); // tx, rx  

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

void myinthandler() // interrupt handler
{
  newData = true;
}
        
        
int main()
{
  pc.baud(9600);  
  myled = 0; // turn off led
  
  wait(5);
  
  //Set up I2C
  i2c.frequency(400000);  // use fast (400 kHz) I2C  
   
  t.start(); // enable system timer
  
  myled = 1; // turn on led
    
  myInterrupt.rise(&myinthandler);  // define interrupt for INT pin output of MPU9250
  
  // Read the WHO_AM_I register, this is a good test of communication
  whoami = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  pc.printf("I AM 0x%x\n\r", whoami); pc.printf("I SHOULD BE 0x71\n\r");
  myled = 1;
  
  if (whoami == 0x71) // WHO_AM_I should always be 0x71
  {  
    pc.printf("MPU9250 WHO_AM_I is 0x%x\n\r", whoami);
    pc.printf("MPU9250 is online...\n\r");
    wait(1);
    
    mpu9250.resetMPU9250(); // Reset registers to default in preparation for device calibration

    mpu9250.MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
    pc.printf("x-axis self test: acceleration trim within: %f pct of factory value\n\r", SelfTest[0]);  
    pc.printf("y-axis self test: acceleration trim within: %f pct of factory value\n\r", SelfTest[1]);  
    pc.printf("z-axis self test: acceleration trim within: %f pct of factory value\n\r", SelfTest[2]);  
    pc.printf("x-axis self test: gyration trim within: %f pct of factory value\n\r", SelfTest[3]);  
    pc.printf("y-axis self test: gyration trim within: %f pct of factory value\n\r", SelfTest[4]);  
    pc.printf("z-axis self test: gyration trim within: %f pct of factory value\n\r", SelfTest[5]);  
 
    mpu9250.getAres(); // Get accelerometer sensitivity
    mpu9250.getGres(); // Get gyro sensitivity
    mpu9250.getMres(); // Get magnetometer sensitivity
    pc.printf("Accelerometer sensitivity is %f LSB/g \n\r", 1.0f/aRes);
    pc.printf("Gyroscope sensitivity is %f LSB/deg/s \n\r", 1.0f/gRes);
    pc.printf("Magnetometer sensitivity is %f LSB/G \n\r", 1.0f/mRes);

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
    wait(1);

    mpu9250.initAK8963(magCalibration);
    pc.printf("AK8963 initialized for active data mode....\n\r"); // Initialize device for active mode read of magnetometer
    pc.printf("Accelerometer full-scale range = %f  g\n\r", 2.0f*(float)(1<<Ascale));
    pc.printf("Gyroscope full-scale range = %f  deg/s\n\r", 250.0f*(float)(1<<Gscale));
    if(Mscale == 0) pc.printf("Magnetometer resolution = 14  bits\n\r");
    if(Mscale == 1) pc.printf("Magnetometer resolution = 16  bits\n\r");
    if(Mmode == 2) pc.printf("Magnetometer ODR = 8 Hz\n\r");
    if(Mmode == 6) pc.printf("Magnetometer ODR = 100 Hz\n\r");

    pc.printf("Mag Calibration: Wave device in a figure eight until done!");
    wait(4);
    mpu9250.magcalMPU9250(magBias, magScale);
    pc.printf("Mag Calibration done!\n\r");
    pc.printf("x mag bias = %f\n\r", magBias[0]);
    pc.printf("y mag bias = %f\n\r", magBias[1]);
    pc.printf("z mag bias = %f\n\r", magBias[2]);
    wait(2);
    }
    
   else
   
   {
    pc.printf("Could not connect to MPU9250: \n\r");
    pc.printf("%#x \n",  whoami);
    myled = 0;
 
    while(1) ; // Loop forever if communication doesn't happen
    }
    
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
   
 /* Main Loop*/
 while(1) {
  
  // If intPin goes high, all data registers have new data
  // if(mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // OUse polling to check for data ready  
   if(newData){   // wait for interrupt for data ready
    newData = false;  // reset newData flag
    
    mpu9250.readMPU9250Data(MPU9250Data); // INT cleared on any read

//    mpu9250.readAccelData(accelCount);  // Read the x/y/z adc values   
    // Now we'll calculate the accleration value into actual g's
    ax = (float)MPU9250Data[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)MPU9250Data[1]*aRes - accelBias[1];   
    az = (float)MPU9250Data[2]*aRes - accelBias[2];  
        
 //   mpu9250.readGyroData(gyroCount);  // Read the x/y/z adc values
    // Calculate the gyro value into actual degrees per second
    gx = (float)MPU9250Data[4]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
    gy = (float)MPU9250Data[5]*gRes - gyroBias[1]; 
    gz = (float)MPU9250Data[6]*gRes - gyroBias[2];   
  
   }
 
    if(mpu9250.readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) {

   mpu9250.readMagData(magCount);  // Read the x/y/z adc values   
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];  
    mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];  
    mx *= magScale[0]; // poor man's soft iron calibration
    my *= magScale[1];
    mz *= magScale[2];  
   }
   
    Now = t.read_us();
    deltat = (float)((Now - lastUpdate)/1000000.0f) ; // set integration time by time elapsed since last filter update
    lastUpdate = Now;
    
    sum += deltat;
    sumCount++;
    
   // Pass gyro rate as rad/s
  mpu9250.MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
//  mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);

    // Serial print and/or display at 1 s rate independent of data rates
    delt_t = t.read_ms() - count;
    if (delt_t > 1000) { // update LCD once per second independent of read rate

    pc.printf("ax = %f", 1000*ax); 
    pc.printf(" ay = %f", 1000*ay); 
    pc.printf(" az = %f  mg\n\r", 1000*az); 

    pc.printf("gx = %f", gx); 
    pc.printf(" gy = %f", gy); 
    pc.printf(" gz = %f  deg/s\n\r", gz); 
    
    pc.printf("mx = %f", mx); 
    pc.printf(" my = %f", my); 
    pc.printf(" mz = %f  mG\n\r", mz); 
    
 //   tempCount = mpu9250.readTempData();  // Read the adc values
    temperature = ((float) MPU9250Data[3]) / 333.87f + 21.0f; // Temperature in degrees Centigrade
    pc.printf("gyro temperature = %f  C\n\r", temperature); 
    
    pc.printf("q0, q1, q2, q3 = %f %f %f %f\n\r",q[0], q[1], q[2], q[3]);
    
    rawPress =  readBMP280Pressure();
    Pressure = (float) bmp280_compensate_P(rawPress)/25600.0f; // Pressure in mbar
    rawTemp =   readBMP280Temperature();
    Temperature = (float) bmp280_compensate_T(rawTemp)/100.0f;

    float altitude = 145366.45f*(1.0f - powf(Pressure/1013.25f, 0.190284f) );
    pc.printf("Temperature = %f C\n\r", Temperature);
    pc.printf("Pressure = %f Pa\n\r", Pressure); 
    pc.printf("Altitude = %f feet\n\r", altitude); 

  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    yaw   = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asinf(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    roll  *= 180.0f / PI;

    pc.printf("Yaw, Pitch, Roll: %f %f %f\n\r", yaw, pitch, roll);
    pc.printf("average rate = %f Hz \n\r", (float) sumCount/sum);
    
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
