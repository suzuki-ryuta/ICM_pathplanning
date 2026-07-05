#pragma warning(disable : 4996) // C4996の警告を無視する

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
#define ADDR_DRIVE_MODE  10           // Drive Mode のアドレス
#define POSITION_CONTROL_MODE 3       // 位置制御モードの番号
#define TIME_BASED_PROFILE_MODE 4     // Drive Mode bit2=1: Time-based Profile
#define ADDR_TORQUE_ENABLE  64    // トルクON/OFFのアドレス
#define ADDR_HOMING_OFFSET  20     // オフセット調整（内部角度ずれ修正）
#define TORQUE_ENABLE  1          // トルクON
#define TORQUE_DISABLE  0         // トルクOFF
#define Min_ID 1               // モータIDの最小値
#define Max_ID 6               // モータIDの最大値
#define NUM 6                   // モータの数
#define ARRAY_NMAX 1000         // ノードの最大許容値。動作計画はノードがこの数値未満になるまで行う
#define DEG_TO_DXL_VALUE(deg) ((int)((deg + 180.0) * 4096.0 / 360.0))  //度数からDynamixelの指令値に変換するマクロ
#define DXL_VALUE_TO_DEG(data) ((double)((data) * 360.0 / 4096.0 - 180.0))  // Dynamixelからの取得値を度数に変換するマクロ
#define JAMMING_ANGLE_THRESHOLD 5.0 //ジャミング判定の閾値

#define VERO 5
#define N 1 // 動作繰り返し数

//追加
#define ADDR_PROFILE_ACCELERATION 108
#define ADDR_PROFILE_VELOCITY     112
#define BACKTRACK_NODES 5   // ジャミング継続時に戻るノード数（必要に応じて調整）
#define SYNC_SPEED_DEG_PER_SEC 400.0
#define MIN_NODE_TIME_MS 30
#define JAM_CHECK_INTERVAL_MS 20
#define PROFILE_ACCEL_RATIO 0.25
#define DEBUG_DXL_IO 0
#define DEBUG_MOTION_LOG 0


double path[ARRAY_NMAX][NUM];
int whole_node_count = 0;
double MAE = 1.0; //最終姿勢時の平均絶対誤差を格納するためのグローバル変数

//[追加]
int set_profile_params(int port_num, uint8_t id, int vel, int acc)
{
    write4ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_PROFILE_VELOCITY, vel);
    int dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    if (dxl_comm_result != COMM_SUCCESS) {
        fprintf(stderr, "Failed to set profile velocity for ID %d\n", id);
        return 0;
    }

    write4ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_PROFILE_ACCELERATION, acc);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    if (dxl_comm_result != COMM_SUCCESS) {
        fprintf(stderr, "Failed to set profile acceleration for ID %d\n", id);
        return 0;
    }
    return 1;
}


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
        if (DEBUG_DXL_IO) printf("SET VALUE is %d\n", param_goal_position);

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
            int32_t present_position = (int32_t)groupSyncReadGetData(group_num, ids[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            angles[i] = DXL_VALUE_TO_DEG(present_position);  // データを角度に変換
            if (DEBUG_DXL_IO) printf("GET VALUE is %d\n", present_position);
        }
        else
        {
            fprintf(stderr, "Data not available for ID: %d\n", ids[i]);
        }
    }

    // グループのパラメータをクリア
    groupSyncReadClearParam(group_num);
}

static int elapsed_ms(LARGE_INTEGER start, LARGE_INTEGER frequency)
{
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return (int)((now.QuadPart - start.QuadPart) * 1000 / frequency.QuadPart);
}

static int calc_profile_time_ms(double time_sec)
{
    int time_ms = (int)(time_sec * 1000.0 + 0.5);
    if (time_ms < MIN_NODE_TIME_MS) time_ms = MIN_NODE_TIME_MS;
    if (time_ms > 32737) time_ms = 32737;
    return time_ms;
}

static int apply_time_profile_to_all(int port_num, const uint8_t* ids, int time_ms)
{
    int acc_ms = (int)(time_ms * PROFILE_ACCEL_RATIO + 0.5);
    if (acc_ms < 0) acc_ms = 0;
    if (acc_ms > time_ms / 2) acc_ms = time_ms / 2;

    for (int i = 0; i < NUM; i++) {
        if (!set_profile_params(port_num, ids[i], time_ms, acc_ms)) {
            return 0;
        }
    }
    return 1;
}

// [変更]角度指令関数（同期移動・移動時間指定付き）
int setanglestime(int port_num, const uint8_t* ids, const double* angles, double time_sec, int monitor_jamming)
{
    int group_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
    if (group_num == -1) {
        fprintf(stderr, "Failed to create Sync Write instance\n");
        return 1;
    }

    double current_positions[NUM];
    getangles(port_num, ids, current_positions);

    // 各関節の移動量を計算
    double delta[NUM];
    for (int i = 0; i < NUM; i++) {
        delta[i] = angles[i] - current_positions[i];
    }

    int time_ms = calc_profile_time_ms(time_sec);
    if (!apply_time_profile_to_all(port_num, ids, time_ms)) {
        return 1;
    }

    for (int i = 0; i < NUM; i++) {
        uint32_t param_goal_position = DEG_TO_DXL_VALUE(angles[i]);

        if (groupSyncWriteAddParam(group_num, ids[i], param_goal_position, LEN_GOAL_POSITION) != 1) {
            fprintf(stderr, "Failed to add parameter for ID: %d\n", ids[i]);
            return 1;
        }
    }

    groupSyncWriteTxPacket(group_num);
    int dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    if (dxl_comm_result != COMM_SUCCESS) {
        fprintf(stderr, "Failed to transmit Sync Write packet: %d\n", dxl_comm_result);
        return 1;
    }

    LARGE_INTEGER frequency, start;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);

    while (1) {
        int elapsed = elapsed_ms(start, frequency);
        if (elapsed >= time_ms) break;

        if (monitor_jamming) {
            double actual[NUM] = { 0.0 };
            double ratio = (double)elapsed / (double)time_ms;
            getangles(port_num, ids, actual);

            for (int i = 0; i < NUM; i++) {
                double expected = current_positions[i] + delta[i] * ratio;
                double error = fabs(expected - actual[i]);
                if (DEBUG_MOTION_LOG) {
                    printf("moving t:%d/%d ms, ID:%d, expect:%f actual:%f error:%f\n",
                        elapsed, time_ms, ids[i], expected, actual[i], error);
                }
                if (error > JAMMING_ANGLE_THRESHOLD) {
                    printf("Jamming during move on joint %d: expected: %f but actual: %f\n",
                        ids[i], expected, actual[i]);
                    groupSyncWriteClearParam(group_num);
                    Sleep(3000);
                    return 1;
                }
            }
        }

        Sleep(JAM_CHECK_INTERVAL_MS);
    }

    groupSyncWriteClearParam(group_num);
    return 0;
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
void read_path_from_file(char* inp)
{
    FILE* fp;
    char str[1000] = { 0 };
    //char input[1000];
    char filename[1004];

    //指定したファイルを読み込む
    printf("type filename\n");
    scanf("%s", inp);
    sprintf(filename, "%s.csv", inp);
    fp = fopen((filename), "r");
    if (fp == NULL)
    {
        printf("ERROR(can't open file)");
        exit(-1);
    }

    while (fgets(str, ARRAY_NMAX, fp) != NULL)
    {
        path[whole_node_count][0] = -atof(strtok(str, ","));
        path[whole_node_count][1] = atof(strtok(NULL, ","));
        path[whole_node_count][2] = -atof(strtok(NULL, ","));
        path[whole_node_count][3] = -atof(strtok(NULL, ","));
        path[whole_node_count][4] = atof(strtok(NULL, ","));
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
//[変更]
int send_command_to_servos(int port_num, const uint8_t* ids, int num, int current_node_index)
{
    if (current_node_index == 0) {
        return setanglestime(port_num, ids, &path[current_node_index][0], 1.0, 0);
    }
    else {
        // ノード間の角度変化の最大値を計算
        double max_delta = 0.0;
        for (int i = 0; i < NUM; i++) {
            double diff = fabs(path[current_node_index][i] - path[current_node_index - 1][i]);
            if (diff > max_delta) max_delta = diff;
        }

        // 移動時間を決定（高ければ速くなる．例: 基準速度400 deg/s）
        double time_sec = max_delta / SYNC_SPEED_DEG_PER_SEC;
        //if (time_sec > 2.0)time_sec = 2.0;

        return setanglestime(port_num, ids, &path[current_node_index][0], time_sec, 1);
    }
}
//void send_command_to_servos(int port_num, const uint8_t* ids, int num, int current_node_index)
//{
//    if (current_node_index == 0)
//    {
//        //サーボを動かす
//        setanglestime(port_num, ids, &path[current_node_index][0], 1.0);
//    }
//    else if (current_node_index + 1 == whole_node_count)//csvの追加手入力角度（大きな移動）用
//    {
//        //サーボを動かす
//        setanglestime(port_num, ids, &path[current_node_index][0], 1.0);
//    }
//    else
//    {
//        //サーボを動かす
//        setangles(port_num, ids, &path[current_node_index][0]);
//    }
//
//    return;
//}

/*----------------------------------------------------------------------------*/
/*
 *  サーボを，open位置に動かす角度コマンドを送信するための関数
 *
*/
void move_to_open(int port_num, const uint8_t* ids, int num)
{
    //double angles[6] = { -73.0, 19.0, 0.0, -73.0, 19.0, 0.0 };
    double angles[6] = { -66.2, 36.0, 1.1, -66.2, 36.0, 1.1 };
    //サーボを動かす
    setanglestime(port_num, ids, angles, 1.0, 0);

    printf("move_to_open OK\n");

    return;
}

//void move_to_release_T(int port_num, const uint8_t* ids, int num)
//{
//    double angles[6] = { -62.314453, -139.306641, -75.146484, -73.0, -86.396484, -30.32226 }; //Tの最終姿勢を手打ち。非効率。
//    //サーボを動かす
//    setanglestime(port_num, ids, angles, 1.0);
//    printf("move_to_release_T OK\n");
//
//    return;
//}

//ジャミングチェックを行う関数．ジャミングを検知したら数秒間静止したのち,1を返す
//各関節を1つずつ順番にジャミングチェックする
int jamming_check(int port_num, const uint8_t* ids, int num, int current_node_index)
{
    //int i = current_node_index % 6; //6関節
    double angles[NUM] = { 0.0 };
    getangles(port_num, ids, angles);

    //printf("|expected[%d]-actual[%d]|, %f\n", i, i, fabs(expected_angle - actual_angle));
    //printf("|expected[%d]-actual[%d]|, %f\n", 1, 1, fabs(path[current_node_index][1] - angles[1]));
    for (int i = 0; i < NUM; i++)
    {
        double expected_angle = path[current_node_index][i];
        double actual_angle = angles[i];
        double error = fabs(expected_angle - actual_angle);

        if (DEBUG_MOTION_LOG) {
            printf("node:%d, servo ID:%d, expect%f actual%f\n", current_node_index, ids[i], expected_angle, angles[i]);
        }

        if (error > JAMMING_ANGLE_THRESHOLD)
        {
            printf("on joint %d: expected: %f but actual: %f\n", ids[i], expected_angle, actual_angle);
            Sleep(3000);
            return 1;
        }
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

    // 各モータのHoming Offset
// 単位はDynamixel内部値
// 例：10deg ≒ 114
    int32_t homing_offset[NUM] = {
        -2,    // ID1
        -0,    // ID2
        8,    // ID3
        8,    // ID4
        -10,    // ID5
        4     // ID6
    };

    // 経路をファイルから読み込む
    char input[1000];
    read_path_from_file(input);
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
        //int dxl_comm_result = pingGetModelNum(port_num, PROTOCOL_VERSION, id);
        int model_num = pingGetModelNum(port_num, PROTOCOL_VERSION, id);
        //if (pingGetModelNum(port_num, PROTOCOL_VERSION, id) != -1)
        if (model_num != -1)
        {
            printf("Dynamixel with ID %d found! Model=%d\n", id, model_num); //応答あり
        }
        else
        {
            printf("No Dynamixel found with ID %d\n", id);
            return -1;
        }
    }

    // Operating Mode / Drive Mode / Homing Offset はトルクOFF中に設定する
    for (int i = 0; i < NUM; i++)
    {
        write1ByteTxRx(port_num, PROTOCOL_VERSION, IDs[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
    }

    // 全軸を位置制御モードに設定
    for (int id = Min_ID; id <= Max_ID; id++)
    {
        write1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_OPERATING_MODE, POSITION_CONTROL_MODE);
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

    // 全軸を Time-based Profile に設定（Profile Velocity の単位が ms になる）
    for (int i = 0; i < NUM; i++)
    {
        write1ByteTxRx(port_num, PROTOCOL_VERSION, IDs[i], ADDR_DRIVE_MODE, TIME_BASED_PROFILE_MODE);
        int dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            fprintf(stderr, "Failed to set time-based profile for Dynamixel ID %d\n", IDs[i]);
            return -1;
        }
        else
        {
            printf("Set Dynamixel ID %d to Time-based Profile\n", IDs[i]);
        }
    }

    // =========================
// Homing Offset設定
// =========================

// 念のため全軸トルクOFF
    for (int i = 0; i < NUM; i++)
    {
        write1ByteTxRx(
            port_num,
            PROTOCOL_VERSION,
            IDs[i],
            ADDR_TORQUE_ENABLE,
            TORQUE_DISABLE
        );
    }

    // 各モータへHoming Offset書き込み
    for (int i = 0; i < NUM; i++)
    {
        write4ByteTxRx(
            port_num,
            PROTOCOL_VERSION,
            IDs[i],
            ADDR_HOMING_OFFSET,
            homing_offset[i]
        );

        int dxl_comm_result =
            getLastTxRxResult(port_num, PROTOCOL_VERSION);

        if (dxl_comm_result != COMM_SUCCESS)
        {
            fprintf(stderr,
                "Failed to set Homing Offset for ID: %d\n",
                IDs[i]);
        }
        else
        {
            printf(
                "Set Homing Offset %d for ID: %d\n",
                homing_offset[i],
                IDs[i]
            );
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


    //// リレーの通信ポートを開く
    //HANDLE hComPort = RelayComOpen(PortName);
    ////リレー0をonにする 
    //RelayOn(hComPort, 0);
    ////リレー0の状態を読む 
    //RelayRead(hComPort, 0);
    ////printf("\n");
    ////待機 
    //Sleep(3500);//ベルトコンベア流入時間
    ////リレー0をoffにする 
    //RelayOff(hComPort, 0);
    //// リレー0の状態を読む 
    //RelayRead(hComPort, 0);
    //printf("\n");



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
            int move_jammed = send_command_to_servos(port_num, IDs, NUM, current_node_index);
            // 計測終了
           // QueryPerformanceCounter(&end);

            // サーボの移動を待つ
           // printf("Waiting on node, , %d  send to servo_time : %lf[ms]\n", current_node_index, (double)(end.QuadPart - start.QuadPart) / frequency.QuadPart * 1000.0);
            if (current_node_index == 0)
            {
                //Sleep(1500);
            }
            else if (current_direction == FORWARD)
            {

                // 計測開始
                //QueryPerformanceCounter(&start); // 各ノードに対するマニピュレーション待機前の時刻を取得
                //移動待機時間(manipulation_time)を決める
                servo_number[current_node_index] = Manipulation(port_num, IDs, NUM, current_node_index);
                if (current_node_index + 1 == whole_node_count)//csvの追加手入力角度（大きな移動）用
                {
                    printf("current node is 194\n");
                    //Sleep(3000);
                }
                if (DEBUG_MOTION_LOG) {
                    printf("node:%d, servo_number[current_node_index]:%d, servo_number[current_node_index - 1]:%d\n", current_node_index, servo_number[current_node_index], servo_number[current_node_index - 1]);
                }
                if (servo_number[current_node_index] == servo_number[current_node_index - 1])
                {
                    if (servo_number[current_node_index] != 6)//存在しないサーボでない
                    {
                        n++;
                    }
                    if (DEBUG_MOTION_LOG) printf("n=%d\n", n);
                    if (n > 1)//n=2で3回同じservo_nunmverが返されている
                    {
                        Sleep(20);
                        if (DEBUG_MOTION_LOG) printf("n>=2 & Sleep(20)\n");
                    }
                }
                else if (servo_number[current_node_index] != servo_number[current_node_index - 1])
                {
                    n = 0;
                    if (DEBUG_MOTION_LOG) printf("n=0\n");
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
                /*以下のコードを追加するとなぜかエラーC1075が発生
                //逆転２秒
                //リレー1をonにする
                RelayOn(hComPort, 1);
                // リレー1の状態を読む
                RelayRead(hComPort, 1);
                // リレー0をonにする
                RelayOn(hComPort, 0);
                // リレー0の状態を読む
                RelayRead(hComPort, 0);
                // 待機
                Sleep(2000);
                //リレー0をoffにする
                RelayOff(hComPort, 0);
                // リレー0の状態を読む
                RelayRead(hComPort, 0);
                printf("\n");
                // リレー1をoffにする
                RelayOff(hComPort, 1);
                //リレー1の状態を読む
                RelayRead(hComPort, 1);

                //正転２秒
                // リレー0をonにする
                RelayOn(hComPort, 0);
                // リレー0の状態を読む
                RelayRead(hComPort, 0);
                //printf("\n");
                // 待機
                Sleep(2000);
                // リレー0をoffにする
                RelayOff(hComPort, 0);
                // リレー0の状態を読む
                RelayRead(hComPort, 0);
                */

                current_direction = FORWARD; //正常系へ帰還
                printf("backward ended on node %d\n", current_node_index);
            }

            //ジャミングチェック　//現在角度の取得をしており，時間がかかる　//ここを改良して，整列速度を上げる
            if (current_direction == FORWARD)
            {
                if (servo_number[current_node_index] != 6)//存在しないサーボでない
                {
                    if (move_jammed || jamming_check(port_num, IDs, NUM, current_node_index)) // fabs(expected_angle - actual_angle)を計算して表示させる.ジャミングなら中に入る。
                    {
                        //[変更]
                        printf("Jamming detected! Reversing conveyor...\n");

                        //// リレー1: 逆回転モードをON
                        //RelayOn(hComPort, 1);
                        //// リレー0: モータ駆動ON
                        //RelayOn(hComPort, 0);

                        Sleep(200); // 0.2秒逆転

                        //// 停止
                        //RelayOff(hComPort, 0);
                        //RelayOff(hComPort, 1);

                        Sleep(100);//0.1秒停止

                        //// 再度正転
                        //RelayOn(hComPort, 0);
                        //Sleep(200);//0.2秒正転して元に戻す
                        //RelayOff(hComPort, 0);

                        // --- 復帰後の再チェック ---
                        Sleep(50); // 念のため少し待ってから再計測
                        if (jamming_check(port_num, IDs, NUM, current_node_index))
                        {
                            // まだ詰まっている → ハンドを数ノード戻す
                            int back = BACKTRACK_NODES;
                            if (back > current_node_index) back = current_node_index; // 0未満防止
                            remaining_backward_count = back;
                            current_direction = BACKWARD;

                            printf("Conveyor recovery failed. Backtracking %d nodes: %d -> %d\n",
                                back, current_node_index, current_node_index - back);
                            // 以降の処理（インデックス更新など）は既存のBACKWARDロジックに任せる
                        }
                        else
                        {
                            // 復帰成功：状態をFORWARDに戻す
                            current_direction = FORWARD;
                            printf("Conveyor recovery done, resuming motion\n");
                            /* current_direction = BACKWARD;
                             remaining_backward_count = 30;
                             printf("Jamming occurred on node %d\n", current_node_index);*/
                        }
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

        if (j == 0 && N > 1)
        {
            //繰り返すならコメントオフを外す
            // ケージをフルオープン
            move_to_open(port_num, IDs, NUM);
            Sleep(1000); // 待機時間

            ///* リレー1をonにする */
            //RelayOn(hComPort, 1);
            ///* リレー1の状態を読む */
            //RelayRead(hComPort, 1);
            ///* リレー0をonにする */
            //RelayOn(hComPort, 0);
            ///* リレー0の状態を読む */
            //RelayRead(hComPort, 0);
            ///* 待機 */
            //Sleep(5000);
            ///* リレー0をoffにする */
            //RelayOff(hComPort, 0);
            ///* リレー0の状態を読む */
            //RelayRead(hComPort, 0);
            //printf("\n");
            ///* リレー1をoffにする */
            //RelayOff(hComPort, 1);
            ///* リレー1の状態を読む */
            //RelayRead(hComPort, 1);
            //printf("\n");

            //Sleep(2000); // 初期位置を手動で変える

            //// リレー0をonにする 
            //RelayOn(hComPort, 0);
            //// リレー0の状態を読む 
            //RelayRead(hComPort, 0);
            ////printf("\n");
            //// 待機 
            //Sleep(5000);
            //// リレー0をoffにする 
            //RelayOff(hComPort, 0);
            //// リレー0の状態を読む 
            //RelayRead(hComPort, 0);
            //printf("\n");

        }

        else
        {
            //    // T字形のケージを開放
                //if (strcmp(input, "t") == 0)
                //{
                //    move_to_release_T(port_num, IDs, NUM);
                //}
            //    // ケージをrelease位置に動かす
            move_to_open(port_num, IDs, NUM);
            Sleep(1000); // release待機時間

            //    //// リレー0をonにする 
            //    //RelayOn(hComPort, 0);
            //    //// リレー0の状態を読む 
            //    //RelayRead(hComPort, 0);
            //    ////printf("\n");
            //    ////待機 
            //    //Sleep(3300);//ベルトコンベア流出時間
            //    //// リレー0をoffにする 
            //    //RelayOff(hComPort, 0);
            //    //// リレー0の状態を読む 
            //    //RelayRead(hComPort, 0);
            //    //printf("\n");
            //    //// リレーの通信ポートを閉じる
            //    //RelayComClose(hComPort);

        }

        //// ケージをrelease位置に動かす
        //move_to_release(hComm);
        //Sleep(5000); // コンベア流出・流入のための待機時間

    }


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
    }

    // ポートを閉じる
    closePort(port_num);
    printf("Port closed successfully.\n");
    return 0;
}
