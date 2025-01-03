// code for the MS5540C depth/temperature sensor
//  It uses SPI communication, require initial read to get calibration
//  information.

     // MS554x Depth Sensor (and thermometer)
 // SPI Pins for Uno      Mega    Cable
 //   VCC         3v+     3v+     Red
 //   GND         GND     GND     Green
 //   DIN (MOSI)  11       51     Lt Blue  
 //  DOUT (MISO)  12       50     Brown
 //  SCLK  (SCK)  13       52     Blue 
 //  MCLK  (CS?)   9       11     Tan

  #include <ros.h>
  #include "Wire.h" // This library allows you to communicate with I2C devices.
  #include <SPI.h>
  #include <std_msgs/Float32MultiArray.h>
  #include <std_msgs/Int16MultiArray.h>

  //imu variables
  const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
  float acc_x, acc_y, acc_z; // variables for accelerometer raw data
  float gyro_x, gyro_y, gyro_z; // variables for gyro raw data
  int temperature; // variables for temperature data


   class Float32MultiArray{
    int data_length;
    int16_t * data;
   };

//depth sensor variables
 int clock = 9; //9;
 unsigned int word1 = 0;
 unsigned int word2 = 0;
 unsigned int word3 = 0;
 unsigned int word4 = 0;
 long c1 = 0;
 long c2 = 0;
 long c3 = 0;
 long c4 = 0;
 long c5 = 0;
 long c6 = 0;



ros::NodeHandle  nh;
std_msgs::Float32MultiArray read_msg;
ros::Publisher chatter("read_sensors", &read_msg);


 void resetsensor() //this function called frequently
{
 SPI.setDataMode(SPI_MODE0);
 SPI.transfer(0x15);
 SPI.transfer(0x55);
 SPI.transfer(0x40);
}

void setup() {

// Serial.begin(9600);

 //initialize I2C
   Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

//initialize SPI
 SPI.begin(); //see SPI library details on arduino.cc for details
 SPI.setBitOrder(MSBFIRST);
 SPI.setClockDivider(SPI_CLOCK_DIV32); //divide 16 MHz to communicate on 500 kHz
 pinMode(clock, OUTPUT);

//initialize node
 nh.initNode();
 nh.advertise(chatter);
 
 delay(100); //delay
 
}
void loop(){

  
  //read data from IMU sensor
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  acc_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  acc_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  acc_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  
  float temp_imu=int(temperature/340.00+36.53);
 
 //read data from depth sensor
 TCCR1B = (TCCR1B & 0xF8) | 1 ; //generates the MCKL signal
 analogWrite (clock, 128) ;
 resetsensor();//resets the sensor - caution: afterwards mode = SPI_MODE0!
 
 //Calibration word 1
 unsigned int word11 = 0;
 SPI.transfer(0x1D); //send first byte of command to get calibration word 1
 SPI.transfer(0x50); //send second byte of command to get calibration word 1
 SPI.setDataMode(SPI_MODE1); //change mode in order to listen
 word1 = SPI.transfer(0x00); //send dummy byte to read first byte of word
 word1 = word1 << 8; //shift returned byte
 word11 = SPI.transfer(0x00); //send dummy byte to read second byte of word
 word1 = word1 | word11; //combine first and second byte of word
 resetsensor();//resets the sensor
 
 //Calibration word 2; see comments on calibration word 1
 byte word22 = 0;
 SPI.transfer(0x1D);
 SPI.transfer(0x60);
 SPI.setDataMode(SPI_MODE1);
 word2 = SPI.transfer(0x00);
 word2 = word2 <<8;
 word22 = SPI.transfer(0x00);
 word2 = word2 | word22;
 resetsensor();//resets the sensor
 
 //Calibration word 3; see comments on calibration word 1
 byte word33 = 0;
 SPI.transfer(0x1D);
 SPI.transfer(0x90);
 SPI.setDataMode(SPI_MODE1);
 word3 = SPI.transfer(0x00);
 word3 = word3 <<8;
 word33 = SPI.transfer(0x00);
 word3 = word3 | word33;
 resetsensor();//resets the sensor
 
 //Calibration word 4; see comments on calibration word 1
 byte word44 = 0;
 SPI.transfer(0x1D);
 SPI.transfer(0xA0);
 SPI.setDataMode(SPI_MODE1);
 word4 = SPI.transfer(0x00);
 word4 = word4 <<8;
 word44 = SPI.transfer(0x00);
 word4 = word4 | word44;

 c1 = word1 >> 1;
 c2 = ((word3 & 0x3F) << 6) | ((word4 & 0x3F));
 c3 = (word4 >> 6) ;
 c4 = (word3 >> 6);
 c5 = (word2 >> 6) | ((word1 & 0x1) >> 10);
 c6 = word2 & 0x3F;
 resetsensor();//resets sensor


 TCCR1B = (TCCR1B & 0xF8) | 1 ; //generates the MCKL signal
 analogWrite (clock, 128) ;
 resetsensor();//resets the sensor 
 
 //caution: afterwards mode = SPI_MODE0!

 //Temperature:
 unsigned int tempMSB = 0; //first byte of value
 unsigned int tempLSB = 0; //last byte of value
 unsigned int D2 = 0;
 SPI.transfer(0x0F); //send first byte of command to get temperature value
 SPI.transfer(0x20); //send second byte of command to get temperature value
 delay(35); //wait for conversion end
 SPI.setDataMode(SPI_MODE1); //change mode in order to listen
 tempMSB = SPI.transfer(0x00); //send dummy byte to read first byte of value
 tempMSB = tempMSB << 8; //shift first byte
 tempLSB = SPI.transfer(0x00); //send dummy byte to read second byte of value
 D2 = tempMSB | tempLSB; //combine first and second byte of value
 resetsensor();//resets the sensor
 
 //Pressure:
 unsigned int presMSB = 0; //first byte of value
 unsigned int presLSB =0; //last byte of value
 unsigned int D1 = 0;
 SPI.transfer(0x0F); //send first byte of command to get pressure value
 SPI.transfer(0x40); //send second byte of command to get pressure value
 delay(35); //wait for conversion end
 SPI.setDataMode(SPI_MODE1); //change mode in order to listen
 presMSB = SPI.transfer(0x00); //send dummy byte to read first byte of value
 presMSB = presMSB << 8; //shift first byte
 presLSB = SPI.transfer(0x00); //send dummy byte to read second byte of value
 D1 = presMSB | presLSB;

 const long UT1 = (c5 * 8) + 20224;
 const long dT =(D2 - UT1);
 const long TEMP = 200 + ((dT * (c6 + 50))/1024);
 const long OFF = (c2*4) + (((c4 - 512) * dT)/4096);
 const long SENS = c1 + ((c3 * dT)/1024) + 24576;

 float pressure = ((((SENS * (D1 - 7168))/16384)- OFF)/32)+250;
 float TEMPREAL = TEMP/10;
// Serial.print("pressure = ");
// Serial.print(pressure);
// Serial.println(" mbar");
 const long dT2 = dT - ((dT >> 7 * dT >> 7) >> 3);
 float waterTemp = (200 + (dT2*(c6+100) >>11))/10;
// Serial.print("temperature = ");
// Serial.print(waterTemp);
// Serial.println(" °C");
// Serial.println("************************************");

//Depth
 float depth=(((pressure-988)*100)/(9.81*1000));

 //data collection
 float sensors_values[10]={ pressure,waterTemp,depth,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,temp_imu};
 read_msg.data = sensors_values;
 read_msg.data_length= 10;
 
 //send data
 chatter.publish( &read_msg );
 nh.spinOnce();
 delay(500);
}
 
