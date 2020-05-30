//計算のための関数群

#define Z_speedmax 1 //[m/s]
#define ROLLmax 20 //[deg]
#define PITCHmax 20 //[deg]
#define YAW_speedmax 60 //[deg/s]


//コントローラのデータから目標状態を計算する関数
void calc_Goalstate() {  
  static double A_roll=0,A_pitch=0,A_yaw_speed=0;
  //スティックの入力かどうか確認
  if(controller[0]==2){
      double Gz = -0.00392156, Groll = 0.078, Gpitch = -0.078;//Gz = Z_speedmax/255, Groll = ROLLmax/255, Gpitch = PITCHmax/255;
      //RS(Left~Right)...左右の場合
      if(controller[1]==0){
        //計算した値を格納してrollを更新
        A_roll = controller[2] * Groll;
      }
      //RS(Up~Down)...前後の場合
      else if(controller[1]==1){
        //計算した値を格納してpitchを更新
        A_pitch = controller[2] * Gpitch;
      }
      //LS(Up~Down)...上下の場合
      else if(controller[1]==3){
        //計算した値を格納してz_speedを更新
        goal_state[0] = controller[2] * Gz;
      }     
  }
  //あるいはボタンの入力かどうか確認
  else if(controller[0]==1){
      //LB...左旋回の場合
      if(controller[1]==4){
          //計算した値を格納してyaw_speedを更新
          A_yaw_speed = controller[2] * YAW_speedmax;
      }
      //RB...右旋回の場合
      if(controller[1]==5){
          //計算した値を格納してyaw_speedを更新
          A_yaw_speed = -1 * controller[2] * YAW_speedmax;
      }
  }
 
  //スティックの入力じゃないなら変化なし
  else return;
 /* double a=5,b=0,c;
c=b/a;
Serial.print(c);*/
  /*
  Serial.print(controller[0]);Serial.print(",");
  Serial.print(controller[1]);Serial.print(",");
  Serial.println(controller[2]);Serial.print(",");*/
    
  //ここからyaw角の算出 = yaw角速度の積分
  //前回実行時のプログラム開始からの時間[ms]。初回実行時だけ現在時間で初期化
  static unsigned long beforetime1 = millis();
  //この関数の実行周期[s]
  double samplingtime;

  //実行周期[s] = (現在時間[ms] - 前回実行時間[ms]) / 1000.0
  samplingtime = (double)(millis() - beforetime1) / 1000.0; 
  //前回実行時間を現在時間に更新
  beforetime1 = millis();

  //積分 ※goal_state[3]=yaw_speed 
  static double A_yaw=0;  //[deg]
  A_yaw += A_yaw_speed * samplingtime;
  /*
  Serial.print(goal_state[0]);Serial.print(",");  
  Serial.print(goal_state[1]);Serial.print(",");
  Serial.print(goal_state[2]);Serial.print(",");
  Serial.println(goal_state[3]);Serial.print(",");*/

  //算出したyaw角をつかってA座標系からB座標系に変換(変換1)
  goal_state[1] = (A_roll*cos(A_yaw*(2*PI/360)) + A_pitch*sin(A_yaw*(2*PI/360)));
  goal_state[2] = (A_roll*sin(A_yaw*(2*PI/360)) - A_pitch*cos(A_yaw*(2*PI/360)));
  goal_state[3] = -1 * A_yaw_speed;
  /*
  Serial.print(goal_state[0]);Serial.print(",");  
  Serial.print(goal_state[1]);Serial.print(",");
  Serial.print(goal_state[2]);Serial.print(",");
  Serial.print(goal_state[3]);Serial.print(",");
  Serial.print(cos(-1*A_yaw*(2*PI/360)));Serial.print(",");
  Serial.print(sin(-1*A_yaw*(2*PI/360)));Serial.print(",");
  Serial.println(A_yaw);*/
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

//3*3の正方行列を計算する関数。
void matrix_mul(double c[][3], double a[][3], double b[][3])
{
  int i, j, k ;

  for(i=0;i<3;i++) {
    for(j=0;j<3;j++) {
      c[i][j] = 0.0 ;
      for(k=0;k<3;k++) c[i][j] += a[i][k]*b[k][j] ;
    }
  }
  return ;
}

//回転行列を計算する関数。
void calc_transMatrix(double aRb[][3],double r,double p,double y) {
  double aRb_r[3][3]={{1,0,0},{0,cos(r),-1*sin(r)},{0,sin(r),cos(r)}} ;
  double aRb_p[3][3]={{cos(p),0,sin(p)},{0,1,0},{-1*sin(p),0,cos(p)}} ;
  double aRb_y[3][3]={{cos(y),-1*sin(y),0},{sin(y),cos(y),0},{0,0,1}} ;
  double result[3][3];
  
  matrix_mul(result, aRb_r, aRb_p) ;
  matrix_mul(aRb, result, aRb_y) ;
}

//回転行列を用いて座標変換(B→A)する関数。
void calc_Aspeed(double aRb[][3],double Bv[],double Av[]){
  Av[0] = aRb[0][0]*Bv[0] + aRb[0][1]*Bv[1] + aRb[0][2]*Bv[2];
  Av[1] = aRb[1][0]*Bv[0] + aRb[1][1]*Bv[1] + aRb[1][2]*Bv[2];
  Av[2] = aRb[2][0]*Bv[0] + aRb[2][1]*Bv[1] + aRb[2][2]*Bv[2];
}

//センサの値から現在状態を計算する関数。
void calc_Nowstate() {
  double z_speed,B_roll,B_pitch,B_yaw,B_yaw_speed;
  //B座標系の速度を格納する配列。 0:vx 1:vy 2:vz
  static double Bv[3]={0},Bga[3]={0};
  //A座標系の速度を格納する配列。 0:vx 1:vy 2:vz
  static double Av[3]={0},Aga[3]={0,0,-9.806};

  //値を格納する
  B_roll = sensor[3];
  B_pitch = sensor[4];
  B_yaw = sensor[5];

  //座標変換するための回転行列を計算する
  double bRa[3][3];
  calc_transMatrix(bRa,B_roll*2*PI/360,B_pitch*2*PI/360,B_yaw*2*PI/360);
  
  //A座標系の重力加速度を座標変換(A→B)して加速度センサの重力成分を引く
  calc_Aspeed(bRa,Aga,Bga);

  //前回実行時のプログラム開始からの時間[ms]。初回実行時だけ現在時間で初期化
  static unsigned long beforetime2 = millis();
  //この関数の実行周期[s]
  double samplingtime;
  
  //実行周期[s] = (現在時間[ms] - 前回実行時間[ms]) / 1000.0
  samplingtime = (double)(millis() - beforetime2) / 1000.0; 
  //前回実行時間を現在時間に更新
  beforetime2 = millis();

  //ドローンにかかる加速度(重力キャンセル後)の前回値
  static double beforesensor[3]={0};

  //NED座標系へのつじつま合わせ
  sensor[0] *= -1;
  sensor[2] *= -1;
  
  //加速度を積分してそれぞれの軸方向の速度を算出
  for(int i=0;i<3;i++){
    Bv[i] += (beforesensor[i] + (sensor[i]-Bga[i]))/2 * samplingtime;
    beforesensor[i]=(sensor[i]-Bga[i]);
    //Serial.print(sensor[i]-Bga[i]);        Serial.print(",");
  }
  //Serial.println();
  /*Serial.print(sensor[2]); Serial.print(",");
  Serial.print(Bga[2]); Serial.print(",");
  Serial.print(beforesensor[2]); Serial.print(",");
  Serial.println(Bv[2]);         */
  
  //B座標系からA座標系に変換(変換2)
  double A_roll,A_pitch,A_yaw;
  A_roll =  (B_roll*cos(-1*B_yaw*(2*PI/360)) + B_pitch*sin(-1*B_yaw*(2*PI/360)));
  A_pitch = (B_roll*sin(-1*B_yaw*(2*PI/360)) - B_pitch*cos(-1*B_yaw*(2*PI/360)));
  A_yaw = -1 * B_yaw;
/*
  Serial.print(B_roll);        Serial.print(",");
  Serial.print(B_pitch);          Serial.print(",");
  Serial.print(B_yaw);        Serial.print(",");
  Serial.print(A_roll);        Serial.print(",");
  Serial.print(A_pitch);          Serial.print(",");
  Serial.println(A_yaw);         // Serial.println(",");*/

  
  //座標変換するための回転行列を計算する
  double aRb[3][3];
  calc_transMatrix(aRb,A_roll,A_pitch,A_yaw);
  
  //算出した速度を座標変換(B→A)してA座標系のz軸方向速度を取り出す
  calc_Aspeed(aRb,Bv,Av);
  z_speed = Av[2];

  //前回実行時のプログラム開始からの時間[ms]。初回実行時だけ現在時間で初期化
  static unsigned long beforetime3 = millis();
  //前回実行時のプログラム開始からのyaw角。初回実行時だけ0で初期化
  static double before_yaw=0;

  //実行周期[s] = (現在時間[ms] - 前回実行時間[ms]) / 1000.0
  samplingtime = (double)(millis() - beforetime3) / 1000.0; 
  //前回実行時間を現在時間に更新
  beforetime3 = millis();

  //目標値のYAWは角速度なので微分する
  B_yaw_speed = (B_yaw-before_yaw) / samplingtime;
  before_yaw = B_yaw;
  
  /*
  Serial.print(B_yaw);        Serial.print(",");
  Serial.print(before_yaw);          Serial.print(",");
  Serial.println(B_yaw_speed);         // Serial.println(",");
  */
  
  //値を格納する
  now_state[0] = z_speed;
  now_state[1] = B_roll;
  now_state[2] = B_pitch;
  now_state[3] = B_yaw_speed;

  /*Serial.println(now_state[0]);       // Serial.print(",");
  Serial.print(now_state[1]);          Serial.print(",");
  Serial.print(now_state[2]);        Serial.print(",");
  Serial.println(now_state[3]);      */  
}

//PID制御により目標状態と現在状態の差からduty比を計算する関数。
void PIDcontroll() {     
  //前回実行時のプログラム開始からの時間[ms]。初回実行時だけ現在時間で初期化
  static unsigned long beforetime4 = millis();
  //この関数の実行周期[s]
  double samplingtime;

  //実行周期[s] = (現在時間[ms] - 前回実行時間[ms]) / 1000.0
  samplingtime = (double)(millis() - beforetime4) / 1000.0; 
  //前回実行時間を現在時間に更新
  beforetime4 = millis();
  
  //now_state[3]が初回infになっているので回避
  if(!isinf(now_state[3])) {
    //影響要素電圧格納配列
    double V[4];
    for(int i=0;i<4;i++){
        static double now_error[4]={0,0,0,0},before_error[4]={0},I[4]={0,0,0,0}; 
        //ゲイン格納配列
        const double Kp[4]={8,0,0.0025,0},Ki[4]={0,0,0,0},Kd[4]={0,0,0.00001,0}; 
        double P,D;
        //Zの現在値を無視
        now_state[0] = 0;
        //現在誤差を格納
        now_error[i] = goal_state[i]-now_state[i];
        //Serial.print(now_error[i]);          Serial.print(","); 
  
        //影響要素電圧計算
        P     = Kp[i]*now_error[i];
        I[i] += Ki[i]*now_error[i]*samplingtime;  
        D     = Kd[i]*(now_error[i]-before_error[i])/samplingtime;
        V[i] = P + I[i] + D;

        //Serial.print(P);          Serial.print(","); 
        //Serial.print(I[i]);       Serial.print(","); 
        //Serial.print(D);          Serial.print(","); 
        //Serial.print(V[i]);       Serial.print(","); 
           
        //前回誤差を現在誤差に更新
        before_error[i] = now_error[i];
    }
    //Serial.println();
    /********これを入れると発熱した*************************************************************************/
    //V[0]=3;
    /*********************************************************************************/
  
    duty[0]=(V[0]+V[1]+V[2]-V[3])/11.1*1000;
    duty[1]=(V[0]-V[1]+V[2]+V[3])/11.1*1000;
    duty[2]=(V[0]-V[1]-V[2]-V[3])/11.1*1000;
    duty[3]=(V[0]+V[1]-V[2]+V[3])/11.1*1000;
  
    for(int i=0;i<4;i++){
        //Serial.print(duty[i]);          Serial.print(","); 
        //リミットカット
        if(duty[i] > 1000) duty[i] = 1000;
        else if(duty[i] < 0) duty[i] = 0;
      //  Serial.print(duty[i]);       Serial.print(","); 
    }
    Serial.println();
  }
}
