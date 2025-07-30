#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

// ===================== DEFINIÇÕES DO MPU6050 =====================
#define MPU6050_ADDR_DEFAULT 0x68
#define MPU6050_ADDR_ALT 0x69
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H 0x43
#define MPU6050_REG_TEMP_OUT_H 0x41
#define MPU6050_REG_WHO_AM_I 0x75

// ===================== ESTRUTURAS DE DADOS =====================
typedef struct {
    float x, y, z;
} imu_vector_t;

typedef struct {
    imu_vector_t accel;  // Aceleração em g
    imu_vector_t gyro;   // Velocidade angular em °/s
    float temperature;   // Temperatura em °C
} imu_data_t;

// ===================== FUNÇÕES PÚBLICAS =====================

/**
 * @brief Inicializa o sensor MPU6050
 * @param i2c_port Porta I2C a ser utilizada
 * @param sda_pin Pino SDA
 * @param scl_pin Pino SCL
 * @return true se inicialização foi bem-sucedida, false caso contrário
 */
bool imu_init(i2c_inst_t *i2c_port, uint sda_pin, uint scl_pin);

/**
 * @brief Reseta o sensor MPU6050
 * @return true se reset foi bem-sucedido, false caso contrário
 */
bool imu_reset(void);

/**
 * @brief Lê dados brutos do sensor
 * @param accel Array para armazenar dados de aceleração (3 eixos)
 * @param gyro Array para armazenar dados de giroscópio (3 eixos)
 * @param temp Ponteiro para armazenar temperatura
 * @return true se leitura foi bem-sucedida, false caso contrário
 */
bool imu_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);

/**
 * @brief Lê dados processados do sensor (em unidades físicas)
 * @param data Estrutura para armazenar os dados processados
 * @return true se leitura foi bem-sucedida, false caso contrário
 */
bool imu_read_data(imu_data_t *data);

/**
 * @brief Verifica se o sensor está respondendo corretamente
 * @return true se sensor está funcionando, false caso contrário
 */
bool imu_is_available(void);

/**
 * @brief Converte dados brutos de aceleração para g
 * @param raw_value Valor bruto do sensor
 * @return Valor em g
 */
float imu_accel_raw_to_g(int16_t raw_value);

/**
 * @brief Converte dados brutos de giroscópio para °/s
 * @param raw_value Valor bruto do sensor
 * @return Valor em graus por segundo
 */
float imu_gyro_raw_to_dps(int16_t raw_value);

/**
 * @brief Converte dados brutos de temperatura para °C
 * @param raw_value Valor bruto do sensor
 * @return Valor em graus Celsius
 */
float imu_temp_raw_to_celsius(int16_t raw_value);

#endif // IMU_H