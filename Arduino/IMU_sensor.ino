  #include <ros.h>
  #include <std_msgs/Int16.h>
  #include <std_msgs/String.h>
  #include <std_msgs/Float32.h>
  #include <std_msgs/Float32MultiArray.h>
  #include <Wire.h>

const int MPU_addr=0x68;  // I2C address of the MPU-6050
float AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

//Set up the ros node and publisher
ros::NodeHandle nh;
std_msgs::Float32MultiArray imu_msg;
ros::Publisher imu("imu", &imu_msg);

   class Float32MultiArray{
    int data_length;
    int16_t * data;
};

                           
void setup()
{
  nh.initNode();
  nh.advertise(imu);
  
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
//  Serial.begin(9600);
}
long publisher_timer;
void loop()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers  String AX = String(mpu6050.getAccX());
  
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  String AX = String(AcX);
  String AY = String(AcY);
  String AZ = String(AcZ);
  String GX = String(GyX);
  String GY = String(GyY);
  String GZ = String(GyZ);
  String tmp = String(Tmp);
  String data = "A" + AX + "B"+ AY + "C" + AZ + "D" + GX + "E" + GY + "F" + GZ + "G" ;
//  Serial.println(data);
//  int length = data.indexOf("G") +2;
//  char data_final[length+1];
//  data.toCharArray(data_final, length+1);
//  
//  if (millis() > publisher_timer) {
//    // step 1: request reading from sensor
//    imu_msg.data = data_final;
//    imu.publish(&imu_msg);
//    publisher_timer = millis() + 100; //publish ten times a second
//    nh.spinOnce();
//  }
    float imu_data[7]={AcX,AcY,AcZ,GyX,GyY,GyZ,Tmp};
    imu_msg.data = imu_data;
    imu_msg.data_length=7;
    imu.publish(&imu_msg);
    nh.spinOnce();
    delay(500);
}
