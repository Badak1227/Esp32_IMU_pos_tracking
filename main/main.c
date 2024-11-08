//왜 안됭
#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// I2C 설정
#define I2C_MASTER_SCL_IO 9               // SCL 핀 (GPIO9)
#define I2C_MASTER_SDA_IO 8               // SDA 핀 (GPIO8)
#define I2C_MASTER_FREQ_HZ 100000         // I2C 클럭 주파수 (100kHz)
#define I2C_MASTER_NUM I2C_NUM_0          // I2C 포트 번호

//imu 디바이스 주소
#define MPU9250_ADDR 0x68                 // MPU9250 기본 I2C 주소
#define AK8963_ADDR 0x0C                  // AK8963 마그네토미터 주소
#define AK8963_CNTL 0x0A                  // AK8963 제어 레지스터

//imu 레지스터 주서
#define ACCEL_XOUT_H 0x3B                 // 가속도 X축 상위 바이트 레지스터 주소
#define GYRO_XOUT_H 0x43                  // 자이로 X축 상위 바이트 레지스터 주소
#define MAG_XOUT_L 0x03                   // 지자기 X축 하위 바이트 레지스터 주소

#define DIM_STATE 9  // 상태 벡터의 차원: [x, y, z, v_x, v_y, v_z, phi, theta, psi]
#define DIM_MEAS 9   // 측정 벡터의 차원: [a_x, a_y, a_z, omega_x, omega_y, omega_z]


// I2C 초기화
void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}
// 마그네토미터 초기화 함수
void init_magnetometer() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AK8963_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, AK8963_CNTL, true);  // 제어 레지스터
    i2c_master_write_byte(cmd, 0x16, true);         // 100Hz, 16비트 모드 설정
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}
// IMU 레지스터 읽기 함수
int16_t read_imu_data(uint8_t device_addr, uint8_t reg_addr) {
    uint8_t data[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);  // 반복 시작 조건
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return (data[0] << 8) | data[1];  // 상위 바이트와 하위 바이트 결합
}
/*
void matrix_multiply(float a[DIM_STATE][DIM_STATE], float b[DIM_STATE][DIM_STATE], float result[DIM_STATE][DIM_STATE]) {
    for (int i = 0; i < DIM_STATE; i++) {
        for (int j = 0; j < DIM_STATE; j++) {
            result[i][j] = 0;
            for (int k = 0; k < DIM_STATE; k++) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}

void matrix_transpose(float m[DIM_STATE][DIM_STATE], float result[DIM_STATE][DIM_STATE]) {
    for (int i = 0; i < DIM_STATE; i++) {
        for (int j = 0; j < DIM_STATE; j++) {
            result[j][i] = m[i][j];
        }
    }
}

void matrix_add(float a[DIM_STATE][DIM_STATE], float b[DIM_STATE][DIM_STATE], float result[DIM_STATE][DIM_STATE]) {
    for (int i = 0; i < DIM_STATE; i++) {
        for (int j = 0; j < DIM_STATE; j++) {
            result[i][j] = a[i][j] + b[i][j];
        }
    }
}

// 상태 예측 함수
void kalman_predict() {
    // 상태 예측 단계: F * x
    float x_pred[DIM_STATE] = {0};
    for (int i = 0; i < DIM_STATE; i++) {
        for (int j = 0; j < DIM_STATE; j++) {
            x_pred[i] += kf->F[i][j] * kf->x[j];
        }
    }

    // 상태 벡터 업데이트
    for (int i = 0; i < DIM_STATE; i++) {
        kf->x[i] = x_pred[i];
    }

    // 오차 공분산 예측: P = F * P * F^T + Q
    float P_pred[DIM_STATE][DIM_STATE] = {0};
    for (int i = 0; i < DIM_STATE; i++) {
        for (int j = 0; j < DIM_STATE; j++) {
            for (int k = 0; k < DIM_STATE; k++) {
                P_pred[i][j] += kf->F[i][k] * kf->P[k][j];
            }
        }
    }

    // P_pred * F^T 계산 및 Q 더하기
    for (int i = 0; i < DIM_STATE; i++) {
        for (int j = 0; j < DIM_STATE; j++) {
            kf->P[i][j] = 0.0f;
            for (int k = 0; k < DIM_STATE; k++) {
                kf->P[i][j] += P_pred[i][k] * kf->F[j][k];
            }
            kf->P[i][j] += kf->Q[i][j];
        }
    }
}

// 상태 업데이트 함수 (보정 단계)
void kalman_update() {
    // 잔차 계산: y = z - H * x
    float y[DIM_MEAS] = {0};
    for (int i = 0; i < DIM_MEAS; i++) {
        for (int j = 0; j < DIM_STATE; j++) {
            y[i] += kf->H[i][j] * kf->x[j];
        }
        y[i] = z[i] - y[i];
    }

    // 칼만 이득 계산: K = P * H^T * (H * P * H^T + R)^-1
    float S[DIM_MEAS][DIM_MEAS] = {0};
    for (int i = 0; i < DIM_MEAS; i++) {
        for (int j = 0; j < DIM_MEAS; j++) {
            for (int k = 0; k < DIM_STATE; k++) {
                S[i][j] += kf->H[i][k] * kf->P[k][j];
            }
            S[i][j] += kf->R[i][j];
        }
    }

    // 역행렬 계산은 단순화를 위해 생략 (실제 구현에서는 수치 해석 라이브러리를 사용해야 함)
    // 대신 단순히 S의 대각 성분을 나누는 형태로 간주
    float K[DIM_STATE][DIM_MEAS] = {0};
    for (int i = 0; i < DIM_STATE; i++) {
        for (int j = 0; j < DIM_MEAS; j++) {
            for (int k = 0; k < DIM_MEAS; k++) {
                K[i][j] += kf->P[i][k] * kf->H[j][k] / S[j][j];
            }
        }
    }

    // 상태 보정: x = x + K * y
    for (int i = 0; i < DIM_STATE; i++) {
        for (int j = 0; j < DIM_MEAS; j++) {
            kf->x[i] += K[i][j] * y[j];
        }
    }

    // 오차 공분산 보정: P = (I - K * H) * P
    float KH[DIM_STATE][DIM_STATE] = {0};
    for (int i = 0; i < DIM_STATE; i++) {
        for (int j = 0; j < DIM_STATE; j++) {
            for (int k = 0; k < DIM_MEAS; k++) {
                KH[i][j] += K[i][k] * kf->H[k][j];
            }
        }
    }

    // (I - KH) * P 계산
    for (int i = 0; i < DIM_STATE; i++) {
        for (int j = 0; j < DIM_STATE; j++) {
            kf->P[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];
        }
    }
}*/

void app_main(void) {
    i2c_master_init();  // I2C 초기화

    float dt = 0.5;

    float accel[3];
    float gyro[3];
    float mag[3];
    
    float pos_x = 0, pos_y = 0, pos_z = 0;
    float vel_x = 0, vel_y = 0, vel_z = 0;

    float F[DIM_STATE][DIM_STATE] = {
        {1, 0, 0, dt, 0, 0, 0, 0, 0},
        {0, 1, 0, 0, dt, 0, 0, 0, 0},
        {0, 0, 1, 0, 0, dt, 0, 0, 0},
        {0, 0, 0, 1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 1}
    };  // 상태 전이 행렬
    float H[DIM_MEAS][DIM_STATE] = {
        {0, 0, 0, 1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 1}
    };   // 측정 행렬
    float Q[DIM_STATE][DIM_STATE] = {
        {6.157e-6, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 6.157e-6, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 6.157e-6, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 6.157e-4, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 6.157e-4, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 6.157e-4, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 3.046e-8, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 3.046e-8, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 3.046e-8}
    };  // 과정 잡음 공분산 행렬
    float R[DIM_MEAS][DIM_MEAS] = {
        {6.157e-3, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 6.157e-3, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 6.157e-3, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 3.046e-6, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 3.046e-6, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 3.046e-6, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 3e-7, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 3e-7, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 3e-7}
    };    // 측정 잡음 공분산 행렬
    float P[DIM_STATE][DIM_STATE] = {
        {1, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 1, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 1, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 1}
    };  // 오차 공분산 행렬
    float x[DIM_STATE] = {0};

    while (1) {
        accel[0] = read_imu_data(MPU9250_ADDR, ACCEL_XOUT_H) / 16384.0;  // X축 가속도 데이터 읽기
        accel[1] = read_imu_data(MPU9250_ADDR, ACCEL_XOUT_H + 2) / 16384.0;
        accel[2] = read_imu_data(MPU9250_ADDR, ACCEL_XOUT_H + 4) / 16384.0;

        gyro[0] = read_imu_data(MPU9250_ADDR, GYRO_XOUT_H) / 131.0;  // X축 가속도 데이터 읽기
        gyro[1] = read_imu_data(MPU9250_ADDR, GYRO_XOUT_H + 2) / 131.0;  // X축 가속도 데이터 읽기
        gyro[2] = read_imu_data(MPU9250_ADDR, GYRO_XOUT_H + 4) / 131.0;  // X축 가속도 데이터 읽기
        
        mag[0] = read_imu_data(AK8963_ADDR, MAG_XOUT_L) ;  // X축 가속도 데이터 읽기
        mag[1] = read_imu_data(AK8963_ADDR, MAG_XOUT_L + 2) ;  // X축 가속도 데이터 읽기
        mag[2] = read_imu_data(AK8963_ADDR, MAG_XOUT_L + 4);  // X축 가속도 데이터 읽기

        printf("%f %f %f\n\n", accel[0], accel[1], accel[2]);

        // Y, Z축 가속도 및 자이로, 자기장 데이터도 비슷한 방식으로 읽기 가능
        vTaskDelay((int16_t)(dt * 1000) / portTICK_PERIOD_MS);
    }
}
