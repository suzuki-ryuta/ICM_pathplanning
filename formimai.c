#include <stdio.h>   // 標準ヘッダー
#include <windows.h> // Windows API用ヘッダー
#include <stdlib.h>  //読み込み用のヘッダファイル
#include <string.h>  //読み込み用のヘッダファイル
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <stdbool.h> //bool用
#include "Relay.h"  //リレー制御
#include "dynamixel_sdk.h" // Dynamixel SDKのヘッダファイルをインクルード

#define ADDR_GOAL_POSITION 116  // 目標位置のアドレス
#define LEN_GOAL_POSITION 4      // 目標位置の長さ
#define ADDR_PRESENT_POSITION 132  // 現在位置のアドレス
#define LEN_PRESENT_POSITION 4     // 現在位置の長さ
#define PROTOCOL_VERSION 2.0     // プロトコルバージョン
#define BAUDRATE  4500000        // 通信速度
#define port_numICENAME  "COM9"       // ポート名
#define ADDR_OPERATING_MODE  11       // Operating Mode のアドレス（モデルによって異なる）
#define POSITION_CONTROL_MODE 3       // 位置制御モードの番号
#define ADDR_TORQUE_ENABLE  64    // トルクON/OFFのアドレス
#define TORQUE_ENABLE  1          // トルクON
#define TORQUE_DISABLE  0         // トルクOFF
#define Min_ID 1               // モータIDの最小値
#define Max_ID 6               // モータIDの最大値
#define NUM 6                   // モータの数
#define ARRAY_NMAX 1000         // ノードの最大許容値。動作計画はノードがこの数値未満になるまで行う
#define DEG_TO_DXL_VALUE(deg) ((int)((deg + 180.0) * 4096.0 / 360.0))  //度数からDynamixelの指令値に変換するマクロ
#define DXL_VALUE_TO_DEG(data) ((double)((data) * 360.0 / 4096.0 - 180.0))  // Dynamixelからの取得値を度数に変換するマクロ
#define JAMMING_ANGLE_THRESHOLD 9.0 //ジャミング判定の閾値

#define VERO 5
#define N 1 // 動作繰り返し数

#pragma warning(disable : 4996) // C4996の警告を無視する

double path[ARRAY_NMAX][NUM];
int whole_node_count = 0;
double MAE = 1.0; //最終姿勢時の平均絶対誤差を格納するためのグローバル変数

// 角度指令関数
void setangles(int port_num, const uint8_t* ids, const double* angles) 
{
    // Sync Writeインスタンスの作成
    int group_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
    if (group_num == -1) 
    {
        fprintf(stderr, "Failed to create Sync Write instance\n");
        return;
    }

    for (int i = 0; i < NUM; i++) 
    {
        // 目標角度を取得し、Dynamixelの指令値に変換
        uint32_t param_goal_position = DEG_TO_DXL_VALUE(angles[i]); // 目標角度をDynamixelの指令値に変換
        printf("SET VALUE is %d\n", param_goal_position);

        // モータに目標位置を追加
        if (groupSyncWriteAddParam(group_num, ids[i], param_goal_position, LEN_GOAL_POSITION) != 1) 
        {
            fprintf(stderr, "Failed to add parameter for ID: %d\n", ids[i]);
            return;
        }
    }

    // すべての目標位置を一斉送信
    groupSyncWriteTxPacket(group_num);
    
    // エラーチェック
    int dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        fprintf(stderr, "Failed to transmit Sync Write packet: %d\n", dxl_comm_result);
        return;
    }

    // グループのパラメータをクリア
    groupSyncWriteClearParam(group_num);
}

// 角度取得関数
void getangles(int port_num, const uint8_t* ids, double* angles)
{
    // Sync Readインスタンスの作成
    int group_num = groupSyncRead(port_num, PROTOCOL_VERSION, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    if (group_num == -1)
    {
        fprintf(stderr, "Failed to create Sync Read instance\n");
        return;
    }

    // すべてのモータIDをSync Readのグループに追加
    for (int i = 0; i < NUM; i++)
    {
        if (groupSyncReadAddParam(group_num, ids[i]) != 1)
        {
            fprintf(stderr, "Failed to add parameter for ID: %d\n", ids[i]);
            return;
        }
    }

    // すべての角度を一斉取得
    groupSyncReadTxRxPacket(group_num);

    // エラーチェック
    int dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        fprintf(stderr, "Failed to transmit Sync Read packet: %d\n", dxl_comm_result);
        return;
    }


    // 各モータの現在位置を取得し、角度に変換
    for (int i = 0; i < NUM; i++)
    {
        if (groupSyncReadIsAvailable(group_num, ids[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION))
        {
            uint32_t present_position = groupSyncReadGetData(group_num, ids[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            angles[i] = DXL_VALUE_TO_DEG(present_position);  // データを角度に変換
            printf("GET VALUE is %d\n", present_position);
        }
        else
        {
            fprintf(stderr, "Data not available for ID: %d\n", ids[i]);
        }
    }

    // グループのパラメータをクリア
    groupSyncReadClearParam(group_num);
}

// 角度指令関数（移動時間指定付き）
void setanglestime(int port_num, const uint8_t* ids, const double* angles, double time_sec) 
{
    // Sync Writeインスタンスの作成
    int group_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
    if (group_num == -1) 
    {
        fprintf(stderr, "Failed to create Sync Write instance\n");
        return;
    }

    // 現在の位置を取得するための変数
    double current_positions[NUM];

    // 現在の位置を取得
    getangles(port_num, ids, current_positions);

    // 指令ステップ数の設定（例として100ステップ）
    int steps = 100;
    double step_time = (time_sec / steps) * 1000; // ミリ秒に変換

    for (int step = 0; step <= steps; step++) {
        // Sync Write用のグループをクリア
        groupSyncWriteClearParam(group_num);

        for (int i = 0; i < NUM; i++) 
        {
            // 現在のステップにおける目標角度を計算
            double intermediate_angle = current_positions[i] + (angles[i] - current_positions[i]) * (double)step / steps;
            uint32_t param_goal_position = DEG_TO_DXL_VALUE(intermediate_angle);

            // モータに目標位置を追加
            if (groupSyncWriteAddParam(group_num, ids[i], param_goal_position, LEN_GOAL_POSITION) != 1) 
            {
                fprintf(stderr, "Failed to add parameter for ID: %d\n", ids[i]);
                return;
            }
        }

        // すべての目標位置を一斉送信
        groupSyncWriteTxPacket(group_num);

        // エラーチェック
        int dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            fprintf(stderr, "Failed to transmit Sync Write packet: %d\n", dxl_comm_result);
            return;
        }

        // 次のステップまで待機
        Sleep(step_time);
    }

    // グループのパラメータをクリア
    groupSyncWriteClearParam(group_num);
}



// 指定したcsvファイルから，関節角の時系列を読み込むための関数
/*
 *	関数：void read_path_from_file()
 *	引数：
 *	    無し
 *	戻り値：
 *		0以上			成功
 *		0未満			エラー
*/
void read_path_from_file()
{
    FILE *fp;
    char str[1000] = {0};
    char input[1000];
    char filename[1004];

    //指定したファイルを読み込む
    printf("type filename\n");
    scanf("%s", input);
    sprintf(filename, "%s.csv", input);
    fp = fopen((filename), "r");
    if (fp == NULL)
    {
        printf("ERROR(can't open file)");
        exit(-1);
    }

    while (fgets(str, ARRAY_NMAX, fp) != NULL)
    {
        path[whole_node_count][0] = -atof(strtok(str, ","));
        path[whole_node_count][1] =  atof(strtok(NULL, ","));
        path[whole_node_count][2] = -atof(strtok(NULL, ","));
        path[whole_node_count][3] = -atof(strtok(NULL, ","));
        path[whole_node_count][4] =  atof(strtok(NULL, ","));
        path[whole_node_count][5] = -atof(strtok(NULL, ","));
        whole_node_count++;
    }

    fclose(fp);
}

/*----------------------------------------------------------------------------*/
/*
 *  サーボに，目標の角度コマンドを送信するための関数
 * 
*/


void send_command_to_servos(int port_num, const uint8_t* ids, int num, int current_node_index)
{
    if (current_node_index == 0)
    {
        //サーボを動かす
        setanglestime(port_num, ids, &path[current_node_index][0], 1.0);
    }
    else if (current_node_index + 1 == whole_node_count)//csvの追加手入力角度（大きな移動）用
    {
        //サーボを動かす
        setanglestime(port_num, ids, &path[current_node_index][0], 1.0);
    }
    else
    {
        //サーボを動かす
        setangles(port_num, ids, &path[current_node_index][0]);
    }

    return;
}

/*----------------------------------------------------------------------------*/
/*
 *  サーボを，open位置に動かす角度コマンドを送信するための関数
 *
*/
void move_to_open(int port_num, const uint8_t* ids, int num)
{
    double angles[6] = { -73.0, 19.0, 0.0, -73.0, 19.0, 0.0 }; 

    //サーボを動かす
    setanglestime(port_num, ids, angles, 1.0);
    
    printf("move_to_open OK\n");

    return;
}

//ジャミングチェックを行う関数．ジャミングを検知したら数秒間静止したのち,1を返す
//各関節を1つずつ順番にジャミングチェックする
int jamming_check(int port_num, const uint8_t* ids, int num, int current_node_index)
{
    int i = current_node_index % 6; //6関節
    double expected_angle = path[current_node_index][i];
    double angles[6] = { 0.0 };
    getangles(port_num, ids, angles);
    double actual_angle = angles[i];

    //printf("|expected[%d]-actual[%d]|, %f\n", i, i, fabs(expected_angle - actual_angle));
    printf("|expected[%d]-actual[%d]|, %f\n", 1, 1, fabs(path[current_node_index][1] - angles[1])); 
    for (i = 0; i < 6; i++)
    {
        printf("node:%d, servo ID:%d, expect%f actual%f\n", current_node_index, ids[i], path[current_node_index][i], angles[i]);        
    }
    if (fabs(expected_angle - actual_angle) > JAMMING_ANGLE_THRESHOLD)
    {
        printf("on joint %d: expected: %f but actual: %f\n", ids[i], expected_angle, actual_angle);
        Sleep(5000);
        return 1;
    }   

    return 0;
}

//移動待機時間(manipulation_time)を各stepの最大値に応じて変える関数
int Manipulation(int port_num, const uint8_t* ids, int num, int current_node_index)
{
    int i;
    int servo;
    double step[NUM];
    double step_max = 0.0;
    //double angles[6];

    //bool result = DXL_GetPresentAngles(dvid, ids, angles, num);

    for (i = 0; i < 6; i++)
    {
        step[i] = fabs(path[current_node_index][i] - path[current_node_index - 1][i]);
        //printf("node:%d, servo ID:%d, step:%f\n", current_node_index, ids[i], step[i]);
        if (step_max < step[i])
        {
            step_max = step[i];
            servo = i;
        }
    }
    if (step_max == 0.0)
    {
        servo = 6;//存在しないサーボ
    }
    /*else if (step_max > 0.0 && step_max <1.0)//細かく設定
    {
        Sleep(10);
    }
    else if (step_max >= 1.0 && step_max < 1.2)
    {
        Sleep(12);
    }
    else if (step_max >= 1.2 && step_max < 1.4)
    {
        Sleep(14);
    }
    else if (step_max >= 1.4 && step_max < 1.6)
    {
        Sleep(16);
    }
    else if (step_max >= 1.6 && step_max < 1.8)
    {
        Sleep(18);
    }
    else if (step_max >= 1.8 && step_max < 2.0)
    {
        Sleep(20);
    }
    else if (step_max >= 2.0 && step_max <= 3.0)//以降、0709-3.csv用
    {
        Sleep(50);
    }
    else*/
    {
        Sleep(5); //元々は200。
    }
    
    return servo;
}

/*----------------------------------------------------------------------------*/
/*
 *  サーボ駆動のためのメインプログラム
 * 
*/

#define FORWARD 1
#define BACKWARD 0

int main()
{
    char PortName[] = "COM6";// リレーの通信ポート

    LARGE_INTEGER frequency, start, end;
    int servo_number[ARRAY_NMAX];
    servo_number[0] = 7;//存在しないID//初期値
    int n = 0;
    uint8_t IDs[NUM] = { 1, 2, 3, 4, 5, 6 };   // IDのリスト

    // 経路をファイルから読み込む
    read_path_from_file();
    printf("node count, %d\n", whole_node_count);    
 
    // ポートを開く
    int port_num = portHandler(port_numICENAME);
    packetHandler();

    // ポートを開いてボーレートを設定
    if (openPort(port_num) && setBaudRate(port_num, BAUDRATE))
    {
        printf("Succeeded to open the port and set the baudrate!\n");
    }
    else
    {
        printf("Failed to open the port or set the baudrate!\n");
        return -1;
    }

    // モータの検索
    for (int id = Min_ID; id <= Max_ID; id++)
    {
        // ping コマンドを送信
        int dxl_comm_result = pingGetModelNum(port_num, PROTOCOL_VERSION, id);
        if (pingGetModelNum(port_num, PROTOCOL_VERSION, id) != -1)
        {
            printf("Dynamixel with ID %d found!\n", id); //応答あり
        }
        else
        {
            printf("No Dynamixel found with ID %d\n", id);
            return -1;
        }
    }

    // 全軸を位置制御モードに設定
    for (int id = Min_ID; id <= Max_ID; id++)
    {
        // モータを位置制御モードに設定
        write1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_OPERATING_MODE, POSITION_CONTROL_MODE);
        // エラーチェック
        int dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            fprintf(stderr, "Failed to set operating mode for Dynamixel ID %d\n", id);
            return -1;
        }
        else
        {
            printf("Set Dynamixel ID %d to Position Control Mode\n", id);
        }
    }

    // モータにトルクを有効化
    for (int i = 0; i < NUM; i++)
    {
        write1ByteTxRx(port_num, PROTOCOL_VERSION, IDs[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
        // エラーチェック
        int dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            fprintf(stderr, "Failed to disable torque for ID: %d\n", IDs[i]);
            return -1;
        }
        else
        {
            printf("Torque enabled for ID: %d\n", IDs[i]);
        }
    }

    //トルクをONにしてから、move_to_catchまでの時間を確保
    Sleep(50);

    // ケージをcatch位置に動かす
    move_to_open(port_num, IDs, NUM);
    Sleep(1000); // catch待機時間

    /*
    // リレーの通信ポートを開く
    HANDLE hComPort = RelayComOpen(PortName);
    //リレー0をonにする 
    RelayOn(hComPort, 0);
    //リレー0の状態を読む 
    RelayRead(hComPort, 0);
    //printf("\n");
    //待機 
    Sleep(5000);
    //リレー0をoffにする 
    RelayOff(hComPort, 0);
    // リレー0の状態を読む 
    RelayRead(hComPort, 0);
    printf("\n");
    */
    

    // マニピュレーション動作をN回繰り返す
    for (int j = 0; j < N; j++) {

        int current_node_index = 0;
        int current_direction = FORWARD;
        int remaining_backward_count = 0;

        //// ケージをcatch位置に動かす
        //move_to_catch(hComm);
        //Sleep(5000); // キャッチ移動とコンベア流入のための待機時間


        // 周波数を取得
        QueryPerformanceFrequency(&frequency);
        // 計測開始
        QueryPerformanceCounter(&start);
        while (current_node_index < whole_node_count)
        {
            /*double* expected_angles = path[current_node_index];
            double actual_angles[NUM];*/
            
            // 計測開始
           // QueryPerformanceCounter(&start);
            send_command_to_servos(port_num, IDs, NUM, current_node_index);
            // 計測終了
           // QueryPerformanceCounter(&end);

            // サーボの移動を待つ
           // printf("Waiting on node, , %d  send to servo_time : %lf[ms]\n", current_node_index, (double)(end.QuadPart - start.QuadPart) / frequency.QuadPart * 1000.0);
            if (current_node_index == 0)
            {
                /*Sleep(1500)*/; // 定数で設定←fooで時間指定してるからいらなそう
            }
            else if (current_direction == FORWARD)
            {
                
                // 計測開始
                //QueryPerformanceCounter(&start); // 各ノードに対するマニピュレーション待機前の時刻を取得
                //移動待機時間(manipulation_time)を決める
                servo_number[current_node_index] = Manipulation(port_num,IDs,NUM, current_node_index);
                if (current_node_index + 1 == whole_node_count)//csvの追加手入力角度（大きな移動）用
                {
                    printf("current node is %d\n", current_node_index);
                  
                }
                printf("node:%d, servo_number[current_node_index]:%d, servo_number[current_node_index - 1]:%d\n", current_node_index, servo_number[current_node_index], servo_number[current_node_index - 1]);
                if (servo_number[current_node_index] == servo_number[current_node_index - 1])
                {
                    if (servo_number[current_node_index] != 6)//存在しないサーボでない
                    {
                        n++;
                    }
                    printf("n=%d\n", n);
                    if (n > 1)//n=2で3回同じservo_nunmverが返されている
                    {
                        Sleep(20);
                        printf("n>=2 & Sleep(20)\n");
                    }
                }
                else if (servo_number[current_node_index] != servo_number[current_node_index - 1])
                {
                    n = 0;
                    printf("n=0\n");
                }
                // 計測終了
                //QueryPerformanceCounter(&end); 
               // printf("node:%d, , manipulation_time : %lf[ms]\n", current_node_index, (double)(end.QuadPart - start.QuadPart) / frequency.QuadPart * 1000.0); // [ミリ秒]に直す
            }
            else
            {
                Sleep(10);
            }

            //backwardを解除
            if (current_direction == BACKWARD && remaining_backward_count == 0)
            {
                current_direction = FORWARD;
                printf("backward ended on node %d\n", current_node_index);
            }

            //ジャミングチェック　//現在角度の取得をしており，時間がかかる　//ここを改良して，整列速度を上げる
            if (current_direction == FORWARD)
            {
                if (servo_number[current_node_index] != 6)//存在しないサーボでない
                {
                    if (jamming_check(port_num, IDs, NUM, current_node_index)) // fabs(expected_angle - actual_angle)を計算して表示させる.ジャミングなら中に入る。
                    {
                        current_direction = BACKWARD;
                        remaining_backward_count = 30;
                        printf("Jamming occurred on node %d\n", current_node_index); 
                        return 0;
                    }                   
                }
            }
            // 次のノードへ
            //current_nodeが最終ノード未満 or 最終ノード時の平均絶対誤差が0.01未満なら，current_node_indexを1だけ増加
            if (current_direction == FORWARD)
            {                              
                current_node_index++;
                
            }            
            //動作方向が逆方向なら，current_node_indexを1だけ減少かつ，remaining_back_wordを1だけ減少
            else
            {
                current_node_index--;
                remaining_backward_count--;
                if (current_node_index < 0)
                {
                    printf("modori sugita\n");
                    current_direction = FORWARD;
                    current_node_index = 0;
                }
            }
        }
        // 計測終了
        QueryPerformanceCounter(&end);
        printf("finish time : %lf[s]\n", (double)(end.QuadPart - start.QuadPart) / frequency.QuadPart);
        //Sleep(100);//場合によって変更

        ///* トルクの解除 */
        //    // トルクをOFFする
        //    // トルクをOFFするとサーボが外力で回転するようになります
        //for (int i = 0; i < NUM; i++)
        //{
        //    printf("SEND Torque OFF\n");
        //    RSTorqueOnOff(hComm, 0, ID_LIST[i]);
        //}
        //Sleep(100);//待機

        //// トルクをONする
        //// トルク ON=1/OFF=0
        //for (int i = 0; i < NUM; i++)
        //{
        //    printf("SEND Torque ON\n");
        //    ret = RSTorqueOnOff(hComm, 1, ID_LIST[i]);
        //    if (ret < 0)
        //    {
        //        printf("ERROR:Torque ON failed[%x]\n", ret);
        //        goto End;
        //    }
        //}
        //Sleep(50);//トルクon待機

        if (j == 0 && N > 1)
        {
            // ケージをフルオープン
            move_to_open(port_num, IDs, NUM);
            Sleep(1000); // 待機時間
            /*
            // リレー1をonにする 
            RelayOn(hComPort, 1);
            // リレー1の状態を読む 
            RelayRead(hComPort, 1);
            // リレー0をonにする 
            RelayOn(hComPort, 0);
            // リレー0の状態を読む 
            RelayRead(hComPort, 0);
            // 待機 
            Sleep(5000);
            // リレー0をoffにする 
            RelayOff(hComPort, 0);
            // リレー0の状態を読む 
            RelayRead(hComPort, 0);
            printf("\n");
            // リレー1をoffにする 
            RelayOff(hComPort, 1);
            // リレー1の状態を読む 
            RelayRead(hComPort, 1);
            printf("\n");

            Sleep(2000); // 初期位置を手動で変える

            // リレー0をonにする 
            RelayOn(hComPort, 0);
            // リレー0の状態を読む 
            RelayRead(hComPort, 0);
            //printf("\n");
            // 待機 
            Sleep(5000);
            // リレー0をoffにする 
            RelayOff(hComPort, 0);
            // リレー0の状態を読む 
            RelayRead(hComPort, 0);
            printf("\n");
            */
        }

        else
        {
            /*
            // ケージをrelease位置に動かす
            move_to_open(port_num, IDs, NUM);
            Sleep(1000); // release待機時間
            
            // リレー0をonにする 
            RelayOn(hComPort, 0);
            // リレー0の状態を読む 
            RelayRead(hComPort, 0);
            //printf("\n");
            //待機 
            Sleep(4000);
            // リレー0をoffにする 
            RelayOff(hComPort, 0);
            // リレー0の状態を読む 
            RelayRead(hComPort, 0);
            printf("\n");
            // リレーの通信ポートを閉じる
            RelayComClose(hComPort);
            */
        }

        //// ケージをrelease位置に動かす
        //move_to_release(hComm);
        //Sleep(5000); // コンベア流出・流入のための待機時間

    }

    /*
    // 各モータのトルクを無効化
    for (int i = 0; i < NUM; i++) 
    {
        write1ByteTxRx(port_num, PROTOCOL_VERSION, IDs[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
        // エラーチェック
        int dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        if (dxl_comm_result != COMM_SUCCESS) {
            fprintf(stderr, "Failed to disable torque for ID: %d\n", IDs[i]);
            return -1;
        }        
        else {
            printf("Torque disabled for ID: %d\n", IDs[i]);
        }
    }*/

    // ポートを閉じる
    closePort(port_num);
    printf("Port closed successfully.\n");
    return 0;
}