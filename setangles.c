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
#define POSITION_CONTROL_MODE 3       // 位置制御モードの番号
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
#define JAMMING_ANGLE_THRESHOLD 9.0 //ジャミング判定の閾値
#define OPEN_REACHED_THRESHOLD_DEG 1.5 //move_to_openの到達判定のしきい値
#define OPEN_MOVE_TIMEOUT_MS 30000 //move_to_openのタイムアウト
#define OPEN_MOVE_CHECK_INTERVAL_MS 50 //move_to_openのチェック間隔

#define VERO 5
#define N 1 // 動作繰り返し数



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

int wait_until_angles_reached(int port_num, const uint8_t* ids, int num, const double* target_angles, double threshold_deg, int timeout_ms)
{
    DWORD start_time = GetTickCount();

    while (1)
    {
        double current_angles[NUM] = { 0.0 };
        double max_error = 0.0;
        int max_error_id = -1;

        getangles(port_num, ids, current_angles);

        for (int i = 0; i < num; i++)
        {
            double error = fabs(target_angles[i] - current_angles[i]);
            if (error > max_error)
            {
                max_error = error;
                max_error_id = ids[i];
            }
        }

        if (max_error <= threshold_deg)
        {
            printf("target reached. max_error:%f[deg]\n", max_error);
            return 1;
        }

        if ((int)(GetTickCount() - start_time) >= timeout_ms)
        {
            fprintf(stderr, "Timeout waiting for target. max_error:%f[deg], ID:%d\n", max_error, max_error_id);
            for (int i = 0; i < num; i++)
            {
                fprintf(stderr,
                    "ID:%d target:%f[deg], current:%f[deg], error:%f[deg]\n",
                    ids[i],
                    target_angles[i],
                    current_angles[i],
                    fabs(target_angles[i] - current_angles[i])
                );
            }
            return 0;
        }

        Sleep(OPEN_MOVE_CHECK_INTERVAL_MS);
    }
}

/*----------------------------------------------------------------------------*/
/*
 *  サーボを，open位置に動かす角度コマンドを送信するための関数
 *
*/
void move_to_open(int port_num, const uint8_t* ids, int num)
{
    double angles[6] = { 68.018359, -121.291797, 50.839844, 0.142578, 41.1875, -89.026953
    }; //☆☆☆ここに角度を入力して挙動を確認☆☆☆

    angles[0] = -angles[0];
    angles[2] = -angles[2];
    angles[3] = -angles[3];
    angles[5] = -angles[5];

    //サーボを動かす
    setanglestime(port_num, ids, angles, 1.0);
    if (wait_until_angles_reached(port_num, ids, num, angles, OPEN_REACHED_THRESHOLD_DEG, OPEN_MOVE_TIMEOUT_MS))
    {
        printf("move_to_open OK\n");
    }
    else
    {
        fprintf(stderr, "move_to_open target was not reached\n");
    }

    return;
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

    // 各モータのHoming Offsetオフセット量調整
// 単位はDynamixel内部値
// 例：10deg ≒ 114
    int32_t homing_offset[NUM] = {
        -2,    // ID1
        -0,    // ID2
        8,     // ID3
        8,     // ID4
        -10,   // ID5
        4      // ID6
    };
 
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

    // ポートを閉じる
    closePort(port_num);
    printf("Port closed successfully.\n");
    return 0;
}
