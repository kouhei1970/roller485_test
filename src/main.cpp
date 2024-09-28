#include <Arduino.h>
#include "unit_rolleri2c.hpp"
#include <M5Unified.h>
#include <MadgwickAHRS.h>

UnitRollerI2C RollerI2C_RIGHT;  // Create a UNIT_ROLLERI2C object
UnitRollerI2C RollerI2C_LEFT;
//I2Cドライバの多重初期化を防ぐためのフラグ
bool UnitRollerI2C::initialized = false;

float Pitch_ahrs, Roll_ahrs, Yaw_ahrs;
float Gyro_x, Gyro_y, Gyro_z;
float Acc_x, Acc_y, Acc_z;
int32_t Imu_time,_Imu_time, Imu_dtime;
int32_t Current_ref_r, Current_ref_l;
int32_t Current_r, Current_l;
int32_t Pos_r, Pos_l;
int32_t Speed_r, Speed_l;
int32_t St, _St, Et, Dt;
TaskHandle_t xHandle = NULL;

void dummyTask(void *pvParameters) {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(25));
        printf("time:%6.3fms roll:%7.3f pitch:%7.3f yaw:%7.3f C_r:%7d C_l:%7d P_r:%8.3f P_l:%8.3f S_r:%8.3f S_l:%8.3f\n",
        (float)(Dt)/1.0e3, Roll_ahrs, Pitch_ahrs, Yaw_ahrs,
        Current_r, Current_l, (float)Pos_r/100.0, (float)Pos_l/100.0, (float)Speed_r/100.0, (float)Speed_l/100.0);
    }
}

void taskFunction(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 5ms の周期

    // 初期化
    xLastWakeTime = xTaskGetTickCount();

    while (true) {
        // 次の周期まで待機
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        _St = St;
        St = micros();
        Dt = St - _St;
        M5.Imu.update();
        auto imudata = M5.Imu.getImuData();
        Gyro_x = imudata.gyro.x;
        Gyro_y = imudata.gyro.y;
        Gyro_z = imudata.gyro.z;
        Acc_x = imudata.accel.x;
        Acc_y = imudata.accel.y;
        Acc_z = imudata.accel.z;
        _Imu_time = Imu_time;
        Imu_time = imudata.usec;
        Imu_dtime = Imu_time - _Imu_time;
        MadgwickAHRSupdateIMU(Gyro_x * DEG_TO_RAD, Gyro_y * DEG_TO_RAD, Gyro_z * DEG_TO_RAD, Acc_x, Acc_y, Acc_z, &Pitch_ahrs, &Roll_ahrs, &Yaw_ahrs);
        
        // current mode
        Current_ref_r = 10000;
        Current_ref_l = 10000;
        RollerI2C_RIGHT.setCurrent(Current_ref_r);
        RollerI2C_LEFT.setCurrent(-Current_ref_l);
        Current_r = RollerI2C_RIGHT.getCurrentReadback();
        Current_l = RollerI2C_LEFT.getCurrentReadback();
        Pos_r = RollerI2C_RIGHT.getPosReadback();
        Pos_l = RollerI2C_LEFT.getPosReadback();
        Speed_r = RollerI2C_RIGHT.getSpeedReadback();
        Speed_l = RollerI2C_LEFT.getSpeedReadback();
        Et = micros();
    }
}

void setup(){
    auto cfg = M5.config();     
    M5.begin(cfg);
    M5.Display.setTextSize(3);               // テキストサイズを変更
    M5.Display.print("Hello World!!");       // 画面にHello World!!と1行表示
    delay(2000);
    printf("Start\n");
    if(RollerI2C_RIGHT.begin(0x64, 2, 1, 400000)==true){
        printf("RollerI2C_RIGHT begin success\n");
    }
    else{
        printf("RollerI2C_RIGHT begin failed\n");
    }
    if(RollerI2C_LEFT.begin(0x65, 2, 1, 400000)==true){
        printf("RollerI2C_LEFT begin success\n");
    }
    else{
        printf("RollerI2C_LEFT begin failed\n");
    }

    RollerI2C_RIGHT.setMode(3);
    RollerI2C_LEFT.setMode(3);
    RollerI2C_RIGHT.setCurrent(0);
    RollerI2C_LEFT.setCurrent(0);
    RollerI2C_RIGHT.setOutput(1);
    RollerI2C_LEFT.setOutput(1);
    delay(1000);


    // FreeRTOSタスクの作成
    BaseType_t result = xTaskCreateUniversal(
        taskFunction,
        "5ms Periodic Task",
        8192,
        NULL,
        23,
        NULL,
        APP_CPU_NUM
    );

    if (result != pdPASS) {
        printf("Task creation failed: %d\n", result);
        while (1); // 無限ループで停止
    }

    result = xTaskCreateUniversal(
        dummyTask,
        "Dummy Task",
        8192,
        NULL,
        1,
        NULL,
        APP_CPU_NUM
    );

    if (result != pdPASS) {
        printf("Task creation failed: %d\n", result);
        while (1); // 無限ループで停止
    }
}

void loop() {
    // FreeRTOSを使用する場合、loop関数は空にします
    delay(1);
}