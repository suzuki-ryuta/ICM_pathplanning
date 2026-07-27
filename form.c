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
#define JAMMING_READ_INTERVAL_MS 200       // ジャミング判定用のエンコーダ読み取り周期
#define JAMMING_STUCK_DELTA_DEG 3       // この角度未満の変化なら停止気味とみなす
#define JAMMING_COMMAND_STEP_THRESHOLD_DEG 2 // CSV上でこの角度以上動く関節だけ判定する
#define JAMMING_STUCK_COUNT_LIMIT 4       // 停止気味がこの回数連続したらジャミング

#define VERO 5
#define N 1 // 動作繰り返し数

//追加
#define ADDR_PROFILE_ACCELERATION 108
#define ADDR_PROFILE_VELOCITY     112
#define BACKTRACK_NODES 5   // ジャミング継続時に戻るノード数（必要に応じて調整）
#define CSV_PROFILE_VELOCITY 150 //manipulation中のサーボ速度
#define CSV_PROFILE_ACCELERATION 50 //manipulation中のサーボ加速度
#define OPEN_PROFILE_VELOCITY 30 //move_to_openのサーボ速度
#define OPEN_PROFILE_ACCELERATION 10 //move_to_openのサーボ加速度
#define OPEN_REACHED_THRESHOLD_DEG 2.0 //move_to_openの到達判定のしきい値
#define OPEN_MOVE_TIMEOUT_MS 30000 //move_to_openのタイムアウト
#define OPEN_MOVE_CHECK_INTERVAL_MS 50 //move_to_openのチェック間隔
#define CSV_REACHED_THRESHOLD_DEG 2.0 //CSV上の到達判定のしきい値
#define CSV_FINAL_REACHED_THRESHOLD_DEG 0.5//最終姿勢周辺の到達判定のしきい値
#define CSV_FINAL_FINE_NODE_COUNT 2 //最終姿勢から何個前のノードを正確にするか
#define CSV_REACHED_CHECK_INTERVAL_MS 5 //CSV上の到達判定のチェック間隔
#define CSV_FINAL_REACHED_STABLE_COUNT 5 //最終姿勢が何回連続で閾値内なら到達とみなすか
#define CSV_FINAL_COMMAND_RESEND_INTERVAL_MS 500 //最終姿勢待機中の指令再送間隔
#define CSV_FINAL_HOLD_MS 3000 //最終姿勢到達後に姿勢を保持する時間
#define CSV_MOVE_TO_OPEN_AFTER_FINISH 0 //1なら終了後にopen位置へ戻す
#define CSV_FINAL_APPROACH_TIME_SEC 1.0 //最終ノードだけsetangles.cと同じ補間指令で近づく時間


double path[ARRAY_NMAX][NUM];
int whole_node_count = 0;
double MAE = 1.0; //最終姿勢時の平均絶対誤差を格納するためのグローバル変数

static double previous_jamming_angles[NUM] = { 0.0 };
static int jamming_stuck_count[NUM] = { 0 };
static int jamming_has_previous_angles = 0;
static int previous_jamming_node_index[NUM] = { 0 };
static DWORD last_jamming_read_time = 0;

void reset_jamming_check_state(void)
{
    for (int i = 0; i < NUM; i++)
    {
        previous_jamming_angles[i] = 0.0;
        jamming_stuck_count[i] = 0;
        previous_jamming_node_index[i] = 0;
    }

    jamming_has_previous_angles = 0;
    last_jamming_read_time = 0;
}

//[追加]
void set_profile_params(int port_num, uint8_t id, int vel, int acc)
{
    write4ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_PROFILE_VELOCITY, vel);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_PROFILE_ACCELERATION, acc);

    int dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    if (dxl_comm_result != COMM_SUCCESS) {
        fprintf(stderr, "Failed to set profile for ID %d\n", id);
    }
}

void set_profile_params_all(int port_num, const uint8_t* ids, int num, int vel, int acc)
{
    for (int i = 0; i < num; i++)
    {
        set_profile_params(port_num, ids[i], vel, acc);
    }
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
    static int group_num = -1;
    static int group_ready = 0;
    static int initialized_port_num = -1;

    if (group_num == -1 || initialized_port_num != port_num)
    {
        group_num = groupSyncRead(port_num, PROTOCOL_VERSION, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
        if (group_num == -1)
        {
            fprintf(stderr, "Failed to create Sync Read instance\n");
            return;
        }

        initialized_port_num = port_num;
        group_ready = 0;
    }

    if (!group_ready)
    {
        // すべてのモータIDをSync Readのグループに追加
        for (int i = 0; i < NUM; i++)
        {
            if (groupSyncReadAddParam(group_num, ids[i]) != 1)
            {
                fprintf(stderr, "Failed to add parameter for ID: %d\n", ids[i]);
                groupSyncReadClearParam(group_num);
                group_ready = 0;
                return;
            }
        }

        group_ready = 1;
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
        }
        else
        {
            fprintf(stderr, "Data not available for ID: %d\n", ids[i]);
        }
    }
}

// [変更]角度指令関数（同期移動）
void setangles_profile(int port_num, const uint8_t* ids, const double* angles)
{
    int group_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
    if (group_num == -1) {
        fprintf(stderr, "Failed to create Sync Write instance\n");
        return;
    }

    for (int i = 0; i < NUM; i++) {
        uint32_t param_goal_position = DEG_TO_DXL_VALUE(angles[i]);

        if (groupSyncWriteAddParam(group_num, ids[i], param_goal_position, LEN_GOAL_POSITION) != 1) {
            fprintf(stderr, "Failed to add parameter for ID: %d\n", ids[i]);
            groupSyncWriteClearParam(group_num);
            return;
        }
    }

    // 目標角度だけ送信し，移動はモータ側のProfileに任せる
    groupSyncWriteTxPacket(group_num);
    int dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    if (dxl_comm_result != COMM_SUCCESS) {
        fprintf(stderr, "Failed to transmit Sync Write packet: %d\n", dxl_comm_result);
        groupSyncWriteClearParam(group_num);
        return;
    }

    groupSyncWriteClearParam(group_num);
}

void setangles_time(int port_num, const uint8_t* ids, const double* angles, double time_sec)
{
    int group_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
    if (group_num == -1)
    {
        fprintf(stderr, "Failed to create Sync Write instance\n");
        return;
    }

    double current_positions[NUM] = { 0.0 };
    getangles(port_num, ids, current_positions);

    int steps = 100;
    DWORD step_time_ms = (DWORD)((time_sec / steps) * 1000.0);
    if (step_time_ms < 1)
    {
        step_time_ms = 1;
    }

    for (int step = 0; step <= steps; step++)
    {
        groupSyncWriteClearParam(group_num);

        for (int i = 0; i < NUM; i++)
        {
            double intermediate_angle = current_positions[i] +
                (angles[i] - current_positions[i]) * (double)step / steps;
            uint32_t param_goal_position = DEG_TO_DXL_VALUE(intermediate_angle);

            if (groupSyncWriteAddParam(group_num, ids[i], param_goal_position, LEN_GOAL_POSITION) != 1)
            {
                fprintf(stderr, "Failed to add parameter for ID: %d\n", ids[i]);
                groupSyncWriteClearParam(group_num);
                return;
            }
        }

        groupSyncWriteTxPacket(group_num);

        int dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            fprintf(stderr, "Failed to transmit Sync Write packet: %d\n", dxl_comm_result);
            groupSyncWriteClearParam(group_num);
            return;
        }

        Sleep(step_time_ms);
    }

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
void send_command_to_servos(int port_num, const uint8_t* ids, int num, int current_node_index)
{
    (void)num;
    setangles_profile(port_num, ids, &path[current_node_index][0]);
}

int wait_until_angles_reached(int port_num, const uint8_t* ids, int num, const double* target_angles, double threshold_deg, int timeout_ms)
{
    DWORD start_time = GetTickCount();

    while (1)
    {
        double current_angles[NUM] = { 0.0 };
        double max_error = 0.0;

        getangles(port_num, ids, current_angles);

        for (int i = 0; i < num; i++)
        {
            double error = fabs(target_angles[i] - current_angles[i]);
            if (error > max_error)
            {
                max_error = error;
            }
        }

        if (max_error <= threshold_deg)
        {
            printf("target reached. max_error:%f[deg]\n", max_error);
            return 1;
        }

        if ((int)(GetTickCount() - start_time) >= timeout_ms)
        {
            fprintf(stderr, "Timeout waiting for target. max_error:%f[deg]\n", max_error);
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
    //double angles[6] = { -73.0, 19.0, 0.0, -73.0, 19.0, 0.0 };
    double angles[6] = { -66.2, 36.0, 1.1, -66.2, 36.0, 1.1 };
    set_profile_params_all(port_num, ids, num, OPEN_PROFILE_VELOCITY, OPEN_PROFILE_ACCELERATION);
    //サーボを動かす
    setangles_profile(port_num, ids, angles);
    wait_until_angles_reached(port_num, ids, num, angles, OPEN_REACHED_THRESHOLD_DEG, OPEN_MOVE_TIMEOUT_MS);

    printf("move_to_open OK\n");

    return;
}

//void move_to_release_T(int port_num, const uint8_t* ids, int num)
//{
//    double angles[6] = { -62.314453, -139.306641, -75.146484, -73.0, -86.396484, -30.32226 }; //Tの最終姿勢を手打ち。非効率。
//    //サーボを動かす
//    setangles_profile(port_num, ids, angles);
//    printf("move_to_release_T OK\n");
//
//    return;
//}

double get_csv_reached_threshold(int current_node_index)
{
    int fine_start_index = whole_node_count - CSV_FINAL_FINE_NODE_COUNT;
    if (fine_start_index < 0)
    {
        fine_start_index = 0;
    }

    if (current_node_index >= fine_start_index)
    {
        return CSV_FINAL_REACHED_THRESHOLD_DEG;
    }

    return CSV_REACHED_THRESHOLD_DEG;
}

int is_final_csv_node(int current_node_index)
{
    return current_node_index + 1 == whole_node_count;
}

int update_jamming_check_from_angles(const uint8_t* ids, int num, int current_node_index, const double* angles, DWORD now, double threshold_deg)
{
    if (!jamming_has_previous_angles)
    {
        for (int i = 0; i < num; i++)
        {
            previous_jamming_angles[i] = angles[i];
            previous_jamming_node_index[i] = current_node_index;
        }

        last_jamming_read_time = now;
        jamming_has_previous_angles = 1;
        return 0;
    }

    for (int i = 0; i < num; i++)
    {
        double command_delta = fabs(path[current_node_index][i] - path[previous_jamming_node_index[i]][i]);
        double target_error = fabs(path[current_node_index][i] - angles[i]);
        double actual_delta = fabs(angles[i] - previous_jamming_angles[i]);
        double required_delta = command_delta;

        if (required_delta < JAMMING_COMMAND_STEP_THRESHOLD_DEG &&
            target_error > threshold_deg)
        {
            required_delta = target_error;
        }

        if (target_error <= threshold_deg)
        {
            jamming_stuck_count[i] = 0;
            previous_jamming_angles[i] = angles[i];
            previous_jamming_node_index[i] = current_node_index;
        }
        else if (actual_delta <= JAMMING_STUCK_DELTA_DEG)
        {
            if (required_delta >= JAMMING_COMMAND_STEP_THRESHOLD_DEG)
            {
                jamming_stuck_count[i]++;
            }
            else
            {
                jamming_stuck_count[i] = 0;
            }
        }
        else
        {
            jamming_stuck_count[i] = 0;
            previous_jamming_angles[i] = angles[i];
            previous_jamming_node_index[i] = current_node_index;
        }

        printf("node:%d, servo ID:%d, command_delta:%f target_error:%f actual_delta:%f stuck_count:%d\n",
            current_node_index, ids[i], command_delta, target_error, actual_delta, jamming_stuck_count[i]);

        if (jamming_stuck_count[i] >= JAMMING_STUCK_COUNT_LIMIT)
        {
            printf("Jamming on joint %d: target_error:%f but actual_delta:%f for %d checks\n",
                ids[i], target_error, actual_delta, jamming_stuck_count[i]);

            last_jamming_read_time = now;
            return 1;
        }
    }

    last_jamming_read_time = now;
    return 0;
}

//ジャミングチェックを行う関数．動く指令が出ている関節の現在角度が変化しない状態を検知したら1を返す
int jamming_check(int port_num, const uint8_t* ids, int num, int current_node_index)
{
    DWORD now = GetTickCount();
    double angles[NUM] = { 0.0 };

    if (jamming_has_previous_angles &&
        (DWORD)(now - last_jamming_read_time) < JAMMING_READ_INTERVAL_MS)
    {
        return 0;
    }

    getangles(port_num, ids, angles);
    return update_jamming_check_from_angles(ids, num, current_node_index, angles, now, get_csv_reached_threshold(current_node_index));
}

int wait_until_csv_node_reached_or_jammed(int port_num, const uint8_t* ids, int num, int current_node_index, double threshold_deg)
{
    while (1)
    {
        DWORD now = GetTickCount();
        double current_angles[NUM] = { 0.0 };
        double max_error = 0.0;

        getangles(port_num, ids, current_angles);

        for (int i = 0; i < num; i++)
        {
            double error = fabs(path[current_node_index][i] - current_angles[i]);
            if (error > max_error)
            {
                max_error = error;
            }
        }

        if (max_error <= threshold_deg)
        {
            reset_jamming_check_state();
            return 1;
        }

        if (!jamming_has_previous_angles ||
            (DWORD)(now - last_jamming_read_time) >= JAMMING_READ_INTERVAL_MS)
        {
            if (update_jamming_check_from_angles(ids, num, current_node_index, current_angles, now, threshold_deg))
            {
                return 0;
            }
        }

        Sleep(CSV_REACHED_CHECK_INTERVAL_MS);
    }
}

void wait_until_csv_final_node_reached(int port_num, const uint8_t* ids, int num, int current_node_index)
{
    DWORD last_log_time = 0;
    DWORD last_command_time = 0;
    int stable_count = 0;

    printf("waiting final csv node:%d threshold:%f[deg]\n",
        current_node_index,
        CSV_FINAL_REACHED_THRESHOLD_DEG);

    while (1)
    {
        DWORD now = GetTickCount();
        double current_angles[NUM] = { 0.0 };
        double max_error = 0.0;
        int max_error_id = -1;
        int outside_count = 0;

        getangles(port_num, ids, current_angles);

        for (int i = 0; i < num; i++)
        {
            double error = fabs(path[current_node_index][i] - current_angles[i]);
            if (error != error || error > CSV_FINAL_REACHED_THRESHOLD_DEG)
            {
                outside_count++;
            }

            if (error == error && error > max_error)
            {
                max_error = error;
                max_error_id = ids[i];
            }
        }

        if (outside_count == 0)
        {
            stable_count++;

            if (stable_count >= CSV_FINAL_REACHED_STABLE_COUNT)
            {
                printf("final target reached. max_error:%f[deg], stable_count:%d\n",
                    max_error,
                    stable_count);
                for (int i = 0; i < num; i++)
                {
                    printf(
                        "final ID:%d target:%f[deg], current:%f[deg], error:%f[deg]\n",
                        ids[i],
                        path[current_node_index][i],
                        current_angles[i],
                        fabs(path[current_node_index][i] - current_angles[i])
                    );
                }

                reset_jamming_check_state();
                return;
            }
        }
        else
        {
            stable_count = 0;
        }

        if (last_command_time == 0 ||
            (DWORD)(now - last_command_time) >= CSV_FINAL_COMMAND_RESEND_INTERVAL_MS)
        {
            setangles_profile(port_num, ids, &path[current_node_index][0]);
            last_command_time = now;
        }

        if (last_log_time == 0 ||
            (DWORD)(now - last_log_time) >= 1000)
        {
            fprintf(stderr,
                "waiting final target. threshold:%f[deg], max_error:%f[deg], ID:%d, outside_count:%d, stable_count:%d/%d\n",
                CSV_FINAL_REACHED_THRESHOLD_DEG,
                max_error,
                max_error_id,
                outside_count,
                stable_count,
                CSV_FINAL_REACHED_STABLE_COUNT);
            for (int i = 0; i < num; i++)
            {
                fprintf(stderr,
                    "final ID:%d target:%f[deg], current:%f[deg], error:%f[deg]\n",
                    ids[i],
                    path[current_node_index][i],
                    current_angles[i],
                    fabs(path[current_node_index][i] - current_angles[i])
                );
            }
            last_log_time = now;
        }

        Sleep(CSV_REACHED_CHECK_INTERVAL_MS);
    }
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

    // 各モータのHoming Offsetオフセット量調整
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
    printf("FORM final wait settings: threshold=%f[deg], stable_count=%d, final_approach=%f[s], move_to_open_after_finish=%d\n",
        CSV_FINAL_REACHED_THRESHOLD_DEG,
        CSV_FINAL_REACHED_STABLE_COUNT,
        CSV_FINAL_APPROACH_TIME_SEC,
        CSV_MOVE_TO_OPEN_AFTER_FINISH);

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
    int jamming_detection_count = 0;

    for (int j = 0; j < N; j++) {

        int current_node_index = 0;
        int current_direction = FORWARD;
        int remaining_backward_count = 0;
        reset_jamming_check_state();

        //// ケージをcatch位置に動かす
        //move_to_catch(hComm);
        //Sleep(5000); // キャッチ移動とコンベア流入のための待機時間


        // 最初のcsv行まではopen速度のまま計測せずに移動し，到達後1秒待機する
        if (whole_node_count > 0)
        {
            send_command_to_servos(port_num, IDs, NUM, current_node_index);
            Sleep(1000);
            set_profile_params_all(port_num, IDs, NUM, CSV_PROFILE_VELOCITY, CSV_PROFILE_ACCELERATION);
            current_node_index = 1;
        }

        // 周波数を取得
        QueryPerformanceFrequency(&frequency);
        // 計測開始(csvの2行目から最終行まで)
        QueryPerformanceCounter(&start);
        while (current_node_index < whole_node_count)
        {
            int jamming_detected_on_node = 0;
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
                        printf("n>=2\n");
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
            if (current_direction == FORWARD && is_final_csv_node(current_node_index))
            {
                printf("final csv node:%d slow approach start. time:%f[s]\n",
                    current_node_index,
                    CSV_FINAL_APPROACH_TIME_SEC);
                setangles_time(port_num, IDs, &path[current_node_index][0], CSV_FINAL_APPROACH_TIME_SEC);
                wait_until_csv_final_node_reached(port_num, IDs, NUM, current_node_index);
            }
            else
            {
                double threshold_deg = get_csv_reached_threshold(current_node_index);
                if (!wait_until_csv_node_reached_or_jammed(port_num, IDs, NUM, current_node_index, threshold_deg))
                {
                    jamming_detected_on_node = 1;
                }
            }

            //backwardを解除
            if (!jamming_detected_on_node && current_direction == BACKWARD && remaining_backward_count == 0)
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

            if (jamming_detected_on_node)
            {
                //[変更]
                printf("Jamming detected! Backtracking motion...\n");
                jamming_detection_count++;

                int back = BACKTRACK_NODES;
                if (back > current_node_index) back = current_node_index; // 0未満防止
                remaining_backward_count = back;
                current_direction = BACKWARD;
                reset_jamming_check_state();

                if (current_node_index > 0)
                {
                    send_command_to_servos(port_num, IDs, NUM, current_node_index - 1);
                    printf("Jamming recovery command sent: %d -> %d\n",
                        current_node_index, current_node_index - 1);
                }

                //// リレー1: 逆回転モードをON
                //RelayOn(hComPort, 1);
                //// リレー0: モータ駆動ON
                //RelayOn(hComPort, 0);

                //Sleep(200); // 0.2秒逆転

                //// 停止
                //RelayOff(hComPort, 0);
                //RelayOff(hComPort, 1);

                //Sleep(100);//0.1秒停止

                //// 再度正転
                //RelayOn(hComPort, 0);
                //Sleep(200);//0.2秒正転して元に戻す
                //RelayOff(hComPort, 0);

                // --- 復帰後の再チェック ---
                //Sleep(50); // 念のため少し待ってから再計測
                printf("Backtracking %d nodes: %d -> %d\n",
                    back, current_node_index, current_node_index - back);
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
        Sleep(CSV_FINAL_HOLD_MS);

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
            if (CSV_MOVE_TO_OPEN_AFTER_FINISH)
            {
                move_to_open(port_num, IDs, NUM);
            }
            else
            {
                printf("final pose held. move_to_open skipped after finish.\n");
            }
            if (j + 1 == N)
            {
                printf("Jamming detected count: %d\n", jamming_detection_count);
            }
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
