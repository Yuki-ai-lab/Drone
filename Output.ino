//出力処理の関数群
#define Duty_threshold 5 //0.5%の急な変化まで許容
void PWMoutput(){
  
  //Servo motor1,motor2,motor3,motor4;  // create servo object to control a servo 
  //int a1,a2,a3,a4;
  //int val;
  static int now_duty[4]={duty[0],duty[1],duty[2],duty[3]},now_out[4]={1000,1000,1000,1000};
  int output[4];
  
  /*duty[0]=10;
  duty[1]=10;
  duty[2]=10;
  duty[3]=10;*/
  
  //0.01秒でmaxで0.5%変化 = 1秒で50%変化ペース
  for(int i=0;i<4;i++){
    /*if((duty[i]-now_duty[i]) > Duty_threshold)       duty[i] = now_duty[i]+Duty_threshold;
    else if((now_duty[i]-duty[i]) > Duty_threshold)  duty[i] = now_duty[i]-Duty_threshold;
    now_duty[i]=duty[i];*/
    output[i]=duty[i]+1000;
    if((output[i]-now_out[i]) > Duty_threshold) now_out[i]+=Duty_threshold;
    else if(now_out[i]-output[i] > Duty_threshold) now_out[i]-=Duty_threshold;
    else now_out[i]=output[i];
  }
  
  motor1.writeMicroseconds(now_out[0]);
  motor2.writeMicroseconds(now_out[1]);
  motor3.writeMicroseconds(now_out[2]);
  motor4.writeMicroseconds(now_out[3]);
  
  /*
  a1=duty[0]*10+1000;
  a2=duty[1]*10+1000;
  a3=duty[2]*10+1000;
  a4=duty[3]*10+1000;
  */
  
  Serial.print(duty[0]);  Serial.print(","); 
  Serial.print(duty[1]);  Serial.print(","); 
  Serial.print(duty[2]);  Serial.print(","); 
  Serial.print(duty[3]);  Serial.println(","); 
  /*
  motor1.writeMicroseconds(a1);
  motor2.writeMicroseconds(a2);
  motor3.writeMicroseconds(a3);
  motor4.writeMicroseconds(a4);*/
  
  //delay(1);
}
