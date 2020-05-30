//設定関連の関数群

//基本的な初期設定
void Init(){
  Madgwick MadgwickFilter;
  #define MPU6050_PWR_MGMT_1   0x6B
  #define MPU_ADDRESS  0x68
  // UARTの設定。シリアルポートを開き，データレートを115200bpsにセットする
  Serial.begin(115200);     
  // PWMの設定。

  // I2Cの設定。
  Wire.begin();
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_PWR_MGMT_1);  //MPU6050_PWR_MGMT_1レジスタの設定
  Wire.write(0x00);
  Wire.endTransmission();

  MadgwickFilter.begin(100); //100Hz
}

//モータ、もといESCのセットアップ。
//駆動電源をいれて、ESCから音がなったらコントローラのボタンを押して実行する。
Servo motor1,motor2,motor3,motor4;  // create servo object to control a servo
void initMotor(){
  
  #define VAL_MIN 1000
  #define VAL_MAX 2000
  
 // int val;    // variable to read the value from the analog pin 
  
 // Serial.begin(115200);//9600でやってた。これで動くか不明
 // while (!Serial.available());
  motor1.attach(3);  // attaches the servo on pin 9 to the servo object 
  motor2.attach(5);
  motor3.attach(6);
  motor4.attach(9); 
 // val = 1000;
  
  // Wait for input

 // Serial.read();
 // Serial.println("Writing maximum output...");
 // Serial.println("Turn on power source, then wait 2 seconds and press any key.");
  
  motor1.writeMicroseconds(VAL_MAX);
  motor2.writeMicroseconds(VAL_MAX);
  motor3.writeMicroseconds(VAL_MAX);
  motor4.writeMicroseconds(VAL_MAX);

  delay(2000);
  
  // Send min output
  //Serial.println("Sending minimum output");
  motor1.writeMicroseconds(VAL_MIN);
  motor2.writeMicroseconds(VAL_MIN);
  motor3.writeMicroseconds(VAL_MIN);
  motor4.writeMicroseconds(VAL_MIN);
  
  delay(5000);
  
}
