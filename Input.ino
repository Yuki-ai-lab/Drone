//入力処理の関数群

Madgwick MadgwickFilter;  
double avedrix = 0,avedriy = 0,avedriz = 0,aveyaw = 0,averoll = 0,avepitch = 0,avedrix_acc = 0,avedriy_acc = 0,avedriz_acc = 0;
double avenum = 300.0,pullval = 0;
int counter_init=0,counter_init1=0;
bool flag_init = true,flag_init1 = true;
  
//I2Cを用いてセンサの値を受け取り、補正、格納する関数。
void getSensor(){
    //加速度x,y,z ロール角、ピッチ角、ヨー角
    

    //　↓取得のプログラムを書く↓
  //digitalWrite(3,HIGH);
  
   Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  //digitalWrite(12,HIGH);
  //while (Wire.available() < 14);
  //digitalWrite(12,LOW);
  int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw, Temperature;

  axRaw = Wire.read() << 8 | Wire.read();
  ayRaw = Wire.read() << 8 | Wire.read();
  azRaw = Wire.read() << 8 | Wire.read();
  Temperature = Wire.read() << 8 | Wire.read();
  gxRaw = Wire.read() << 8 | Wire.read();
  gyRaw = Wire.read() << 8 | Wire.read();
  gzRaw = Wire.read() << 8 | Wire.read();
//digitalWrite(3,LOW);
  ///////////////////////////////////////////////////////////////////////////この上にバグ有
  

  // 加速度値を分解能で割って加速度(G)に変換する
  double acc_x = axRaw / 16384.0;  //FS_SEL_0 16,384 LSB / g
  double acc_y = ayRaw / 16384.0;
  double acc_z = azRaw / 16384.0;

  // 角速度値を分解能で割って角速度(degrees per sec)に変換する
  double gyro_x = gxRaw / 131.0;  // (度/s)
  double gyro_y = gyRaw / 131.0;
  double gyro_z = gzRaw / 131.0;
  
  if(flag_init){
    avedrix += gyro_x/avenum;
    avedriy += gyro_y/avenum;
    avedriz += gyro_z/avenum;
    avedrix_acc += acc_x/avenum;
    avedriy_acc += acc_y/avenum;
    avedriz_acc += acc_z/avenum;
    
    counter_init += 1;
    if(counter_init >= avenum){
      flag_init = false;
    }
  }
  
  //c.f. Madgwickフィルターを使わずに、PRY（pitch, roll, yaw）を計算
  double pleroll  = atan2(acc_y, acc_z) * RAD_TO_DEG;
  double plepitch = atan(-acc_x / sqrt(acc_y * acc_y + acc_z * acc_z)) * RAD_TO_DEG;
  
  
  //Madgwickフィルターを用いて、PRY（pitch, roll, yaw）を計算
  MadgwickFilter.updateIMU(gyro_x-avedrix, gyro_y-avedriy, gyro_z-avedriz, acc_x, acc_y, acc_z);

  //PRYの計算結果を取得する
  //double roll  = MadgwickFilter.getRoll();
  //double pitch = MadgwickFilter.getPitch();
  double pleyaw   = MadgwickFilter.getYaw();
  
  if(flag_init1){
    aveyaw += pleyaw/avenum;
    averoll += pleroll/avenum;
    avepitch += plepitch/avenum;
    counter_init1 += 1;
    if(counter_init1 >= avenum){
      flag_init1 = false;
    }
  }
 /* if(flag_init1){
      pullval = pleyaw;
      flag_init1 = false;
    }*/
  
 // double yaw = pleyaw-pullval;
  double yaw = pleyaw-aveyaw;
  double roll = pleroll-averoll;
  double pitch = plepitch-avepitch;
  
  //データの格納
  sensor[0] = (acc_x-avedrix_acc)*9.8;
  sensor[1] = (acc_y-avedriy_acc)*9.8;
  sensor[2] = (acc_z-avedriz_acc+1.0)*9.8;
  sensor[3] = roll;
  sensor[4] = pitch*-1;
  sensor[5] = yaw*-4.4;

  //Serial.print(sensor[0]);  Serial.print(",");
  //Serial.print(sensor[1]);  Serial.print(",");
  //Serial.print(sensor[2]);  Serial.println("");
  //Serial.println(sensor[5]); 
  
}

//UARTを用いてコントローラの値を raspberry pi から受け取り、変換、格納する関数。
//返り値としてこの関数が実行されたかどうかを返す。
bool getController(){
    int receiveData[2]={0},xboxType,xboxNum,xboxValue,Negative;
    bool receiveCompleate=0;

    // 受信したデータがバッファにあるなら
    if (Serial.available() > 0) {
        // 受信したデータの1バイトを読み取る
        receiveData[0] = Serial.read();
        // 受信したデータが1番目のデータなら
        if(receiveData[0] & 0x80) {
            //2番目のデータを待って受信
            while(!Serial.available());
            receiveData[1] = Serial.read();
            receiveCompleate=1;
        }
    }

    //　データを読み取れたら
    if(receiveCompleate){
      
        //データの復元
        xboxType = ((receiveData[0] & 0x20) >> 5) + 1;
        xboxNum = (receiveData[0] & 0x1E) >> 1;
        xboxValue = (receiveData[0] & 0x01) << 7 | receiveData[1] & 0x7F;
        Negative = (receiveData[0] & 0x40) >> 6;
        if(Negative) xboxValue *= -1; 

        //データの格納
        controller[0] = (int)xboxType;
        controller[1] = (int)xboxNum;
        controller[2] = (int)xboxValue;
                
        receiveCompleate=0;
        
        //受信完了の合図
        return true;
    }
    // データを読み取れなかったら
    else{
        //未受信の合図
        return false;
    }
}

//************************************************************************************//
//<Botton(xboxType=1)>  xboxValue= 1/ON,0/OFF(default=0)
//xboxNum↓
//A...0
//B...1
//X...3
//Y...4
//LB...6
//RB...7
//menu...11
//LS...13
//RS...14
//
//<Axis(xboxType=2)> xboxValue= -7F~7F(default=0)
//xboxNum↓
//RS(Left~Right)...0
//RS(Up~Down)...1
//LS(Left~Right)...2
//LS(Up~Down)...3
//RT...4 (xboxValue default= -7F)
//LT...5 (xboxValue default= -7F)
//Cross(Left~Right)...6
//Cross(UP~Down)...7
//************************************************************************************//
