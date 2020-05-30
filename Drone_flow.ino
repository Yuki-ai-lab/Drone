//ドローンのメインフロー
#include <Servo.h>
#include <Wire.h>
#include <MadgwickAHRS.h>

//コントローラのデータを入れる配列。0:type 1:num 2:value
int controller[3]={0};
//コントローラのデータを受信完了したことを示すフラグ
bool receive_flag = false;
//センサのデータを入れる配列。 0:ax 1:ay 2:az 3:roll 4:pitch 5:yaw
double sensor[6]={0};
//目標状態を格納する配列。 0:z 1:roll 2:pitch 3:yaw
double goal_state[4]={0};
//現在状態を格納する配列。 0:z 1:roll 2:pitch 3:yaw
double now_state[4]={0};
//duty比(0.1~100%)を格納する配列。 0:v1 1:v2 2:v3 3:v4
int duty[4]={0};


//初期設定。1回しか実行されない。
void setup() {
   //シリアル通信やPWM、I2Cなどの初期設定
   Init();

   //センサの初期誤差をなくすためにある程度読み出しておく  
   for(int i=0;i<500;i++) getSensor();
   
   //コントローラからの合図があったことを示すフラグ
   bool start_flag = false;
   //コントローラからの合図があるまで待機 →　モータのセットアップがあるから
   while(!start_flag){

        //コントローラの値を取得(受信した場合のみ実行)
        receive_flag = getController(); // → controller
        
        //特定のボタンが押されたらループを抜ける(受信した場合のみ実行)
        if(receive_flag){
            start_flag = checkStart(); // ← controller
            if(start_flag) Serial.println("OK");
        }

        //受信完了フラグをクリア
        receive_flag = false;
   }
   //モータのセットアップ信号を送信
   initMotor();

   //コントローラのデータを初期化
   //while(!Serial.available());
   //receive_flag = getController(); // → controller
   //clearController();
   
   //デバック用のLED
   //pinMode(3,OUTPUT);
}

//ここがメインループ処理
void loop() {
  
    //コントローラの値を取得(受信した場合のみ実行)
    receive_flag = getController(); // → controller
    Emstop();
    
    
    //センサからの値を取得(ここで停止することがあります)
    getSensor();  //　→　sensor
    
    
    //コントローラのデータから目標状態を計算（受信した場合のみ実行）
    calc_Goalstate();  //controller → goal_state

    //受信完了フラグをクリア
    receive_flag = false;        
    
    //センサのデータから現在状態を計算(注意！　電源を入れてから十分な時間(5秒ほど)置いてから関数を実行しないとと正しい値を計算できません。)
    calc_Nowstate();  //sensor → now_state 
    
    //目標状態と現在状態の誤差からPIDを通して入力電圧(duty比)を計算
    PIDcontroll();  //goal_state - now_state → duty
    
    //計算したduty比でPWM出力
    PWMoutput();  //　← duty
    
}
