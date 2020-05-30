//その他の関数群

//コントローラのデータを受け取ってそれが開始の合図かどうかを確認する関数。
//合図ならTrue,合図じゃないならFalseを返す。
bool checkStart(){

  //合図のボタン(= Y)かどうか判定する
  if((controller[0]==1) && (controller[1]==3) &&(controller[2]==1)) {
    return true;
  }
  else return false;
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
void Emstop(){
  //合図のボタン(= menu)かどうか判定する
  if((controller[0]==1) && (controller[1]==7) &&(controller[2]==1)) {
    int D = duty[0]+duty[1]+duty[2]+duty[3];
    while(D){
      Serial.print(duty[0]);  Serial.print(","); 
      Serial.print(duty[1]);  Serial.print(","); 
      Serial.print(duty[2]);  Serial.print(","); 
      Serial.print(duty[3]);  Serial.println(",");    
      
      /*Serial.print(d[0]);  Serial.print(","); 
      Serial.print(d[1]);  Serial.print(","); 
      Serial.print(d[2]);  Serial.print(","); 
      Serial.print(d[3]);  Serial.println(","); */
      //1秒周期で5%減少
      duty[0]-=50;
      duty[1]-=50;
      duty[2]-=50;
      duty[3]-=50;
      
      duty[0] = max(0, duty[0]);
      duty[1] = max(0, duty[1]);
      duty[2] = max(0, duty[2]);
      duty[3] = max(0, duty[3]);
      
      motor1.writeMicroseconds(duty[0]+1000);
      motor2.writeMicroseconds(duty[1]+1000);
      motor3.writeMicroseconds(duty[2]+1000);
      motor4.writeMicroseconds(duty[3]+1000);
      
      delay(1000);
  
      D = duty[0]+duty[1]+duty[2]+duty[3];
  
    }
    while(1);
  }
}

/*void clearController(){

}*/
