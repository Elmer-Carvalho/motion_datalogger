#include "imu.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>

// ===================== VARIÁVEIS GLOBAIS PRIVADAS =====================
static i2c_inst_t *i2c_port = NULL;
static bool imu_initialized = false;
static uint8_t mpu6050_addr = MPU6050_ADDR_DEFAULT; // Endereço detectado

// ===================== CONSTANTES DE CONVERSÃO =====================
#define ACCEL_SCALE_FACTOR (2.0f / 32768.0f)  // ±2g range
#define GYRO_SCALE_FACTOR (250.0f / 32768.0f)  // ±250°/s range
#define TEMP_OFFSET 36.53f
#define TEMP_SCALE_FACTOR (1.0f / 340.0f)

// ===================== FUNÇÕES PRIVADAS =====================

/**
 * @brief Escreve um byte em um registrador do MPU6050
 * @param reg Registrador a ser escrito
 * @param data Dado a ser escrito
 * @return true se escrita foi bem-sucedida, false caso contrário
 */
static bool imu_write_register(uint8_t reg, uint8_t data) {
    if (!i2c_port) {
        printf("[IMU] Erro: I2C não inicializado\n");
        return false;
    }
    
    uint8_t buffer[2] = {reg, data};
    int result = i2c_write_blocking(i2c_port, mpu6050_addr, buffer, 2, false);
    
    if (result != 2) {
        printf("[IMU] Erro ao escrever reg 0x%02X = 0x%02X (resultado: %d)\n", reg, data, result);
        return false;
    }
    
    printf("[IMU] Escrita OK: reg 0x%02X = 0x%02X\n", reg, data);
    return true;
}

/**
 * @brief Lê bytes de um registrador do MPU6050
 * @param reg Registrador inicial a ser lido
 * @param buffer Buffer para armazenar os dados lidos
 * @param len Número de bytes a serem lidos
 * @return true se leitura foi bem-sucedida, false caso contrário
 */
static bool imu_read_registers(uint8_t reg, uint8_t *buffer, size_t len) {
    if (!i2c_port || !buffer) {
        printf("[IMU] Erro: parâmetros inválidos para leitura\n");
        return false;
    }
    
    // Escreve o registrador que queremos ler
    int result = i2c_write_blocking(i2c_port, mpu6050_addr, &reg, 1, true);
    if (result != 1) {
        printf("[IMU] Erro ao selecionar registrador 0x%02X (resultado: %d)\n", reg, result);
        return false;
    }
    
    // Lê os dados do registrador
    result = i2c_read_blocking(i2c_port, mpu6050_addr, buffer, len, false);
    if (result != (int)len) {
        printf("[IMU] Erro ao ler %zu bytes do reg 0x%02X (resultado: %d)\n", len, reg, result);
        return false;
    }
    
    return true;
}

// ===================== FUNÇÕES PÚBLICAS =====================

bool imu_init(i2c_inst_t *port, uint sda_pin, uint scl_pin) {
    if (port == NULL) {
        printf("[IMU] Erro: porta I2C inválida\n");
        return false;
    }
    
    i2c_port = port;
    
    // Inicializa I2C com velocidade mais baixa para maior compatibilidade
    uint actual_baudrate = i2c_init(i2c_port, 100 * 1000); // 100kHz
    printf("[IMU] I2C inicializado a %u Hz\n", actual_baudrate);
    
    // Configura os pinos I2C
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    
    printf("[IMU] Pinos I2C configurados: SDA=%d, SCL=%d\n", sda_pin, scl_pin);
    
    // Aguarda estabilização maior
    sleep_ms(500);
    
    // PRIMEIRO: Tenta detectar o endereço correto do MPU6050
    printf("[IMU] Procurando MPU6050...\n");
    uint8_t temp_data;
    int result;
    bool sensor_found = false;
    
    // Testa endereço padrão 0x68
    result = i2c_read_blocking(i2c_port, MPU6050_ADDR_DEFAULT, &temp_data, 1, false);
    if (result == 1) {
        mpu6050_addr = MPU6050_ADDR_DEFAULT;
        sensor_found = true;
        printf("[IMU] MPU6050 encontrado no endereço padrão 0x68\n");
    } else {
        // Testa endereço alternativo 0x69
        result = i2c_read_blocking(i2c_port, MPU6050_ADDR_ALT, &temp_data, 1, false);
        if (result == 1) {
            mpu6050_addr = MPU6050_ADDR_ALT;
            sensor_found = true;
            printf("[IMU] MPU6050 encontrado no endereço alternativo 0x69\n");
        }
    }
    
    if (!sensor_found) {
        printf("[IMU] MPU6050 não encontrado nos endereços 0x68 ou 0x69\n");
        
        // Faz scan completo I2C
        printf("[IMU] Fazendo scan I2C completo...\n");
        bool found_devices = false;
        for (int addr = 0x08; addr < 0x78; addr++) {
            int ret = i2c_read_blocking(i2c_port, addr, &temp_data, 1, false);
            if (ret == 1) {
                printf("[IMU] --> Dispositivo I2C encontrado no endereço 0x%02X\n", addr);
                found_devices = true;
            }
        }
        
        if (!found_devices) {
            printf("[IMU] ERRO: Nenhum dispositivo I2C encontrado!\n");
            printf("[IMU] Possíveis problemas:\n");
            printf("[IMU] - Conexões SDA (pino %d) e SCL (pino %d)\n", sda_pin, scl_pin);
            printf("[IMU] - Alimentação do sensor (VCC = 3.3V, GND conectado)\n");
            printf("[IMU] - Sensor com defeito\n");
            printf("[IMU] - Resistores pull-up externos necessários\n");
        } else {
            printf("[IMU] Dispositivos I2C encontrados, mas nenhum é MPU6050\n");
        }
        
        return false;
    }
    
    // AGORA marca como inicializado para permitir funções de leitura/escrita
    imu_initialized = true;
    
    // Agora tenta resetar o sensor
    printf("[IMU] Resetando sensor...\n");
    if (!imu_reset()) {
        printf("[IMU] Erro: falha ao resetar o sensor\n");
        imu_initialized = false;
        return false;
    }
    
    // Verifica novamente após reset
    if (!imu_is_available()) {
        printf("[IMU] Erro: sensor parou de responder após reset\n");
        imu_initialized = false;
        return false;
    }
    
    printf("[IMU] Inicialização concluída com sucesso\n");
    return true;
}

bool imu_reset(void) {
    if (!imu_initialized || !i2c_port) {
        printf("[IMU] Erro: IMU não inicializado para reset\n");
        return false;
    }
    
    printf("[IMU] Iniciando reset do sensor...\n");
    
    // Reset do dispositivo (bit 7 do registrador 0x6B)
    if (!imu_write_register(MPU6050_REG_PWR_MGMT_1, 0x80)) {
        printf("[IMU] Erro: falha ao enviar comando de reset\n");
        return false;
    }
    
    printf("[IMU] Comando de reset enviado, aguardando...\n");
    // Aguarda reset e estabilização
    sleep_ms(200);
    
    // Sai do modo sleep (bit 6 = 0)
    if (!imu_write_register(MPU6050_REG_PWR_MGMT_1, 0x00)) {
        printf("[IMU] Erro: falha ao sair do modo sleep\n");
        return false;
    }
    
    printf("[IMU] Saída do modo sleep, aguardando estabilização...\n");
    // Aguarda estabilização após sair do sleep
    sleep_ms(100);
    
    printf("[IMU] Reset concluído com sucesso\n");
    return true;
}

bool imu_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    if (!imu_initialized || accel == NULL || gyro == NULL || temp == NULL) {
        return false;
    }
    
    uint8_t buffer[6];
    
    // Lê dados de aceleração (6 bytes a partir do reg 0x3B)
    if (!imu_read_registers(MPU6050_REG_ACCEL_XOUT_H, buffer, 6)) {
        return false;
    }
    
    // Converte bytes em valores de 16 bits (big-endian)
    for (int i = 0; i < 3; i++) {
        accel[i] = (int16_t)((buffer[i * 2] << 8) | buffer[(i * 2) + 1]);
    }
    
    // Lê dados de giroscópio (6 bytes a partir do reg 0x43)
    if (!imu_read_registers(MPU6050_REG_GYRO_XOUT_H, buffer, 6)) {
        return false;
    }
    
    // Converte bytes em valores de 16 bits (big-endian)
    for (int i = 0; i < 3; i++) {
        gyro[i] = (int16_t)((buffer[i * 2] << 8) | buffer[(i * 2) + 1]);
    }
    
    // Lê dados de temperatura (2 bytes a partir do reg 0x41)
    if (!imu_read_registers(MPU6050_REG_TEMP_OUT_H, buffer, 2)) {
        return false;
    }
    
    // Converte bytes em valor de 16 bits (big-endian)
    *temp = (int16_t)((buffer[0] << 8) | buffer[1]);
    
    return true;
}

bool imu_read_data(imu_data_t *data) {
    if (data == NULL) {
        return false;
    }
    
    int16_t raw_accel[3], raw_gyro[3], raw_temp;
    
    // Lê dados brutos
    if (!imu_read_raw(raw_accel, raw_gyro, &raw_temp)) {
        return false;
    }
    
    // Converte para unidades físicas
    data->accel.x = imu_accel_raw_to_g(raw_accel[0]);
    data->accel.y = imu_accel_raw_to_g(raw_accel[1]);
    data->accel.z = imu_accel_raw_to_g(raw_accel[2]);
    
    data->gyro.x = imu_gyro_raw_to_dps(raw_gyro[0]);
    data->gyro.y = imu_gyro_raw_to_dps(raw_gyro[1]);
    data->gyro.z = imu_gyro_raw_to_dps(raw_gyro[2]);
    
    data->temperature = imu_temp_raw_to_celsius(raw_temp);
    
    return true;
}

bool imu_is_available(void) {
    if (!i2c_port) {
        return false;
    }
    
    // Testa comunicação com um scan básico do endereço I2C detectado
    uint8_t temp_data;
    int result = i2c_read_blocking(i2c_port, mpu6050_addr, &temp_data, 1, false);
    
    if (result == 1) {
        printf("[IMU] Sensor responde no endereço 0x%02X\n", mpu6050_addr);
        return true;
    } else {
        printf("[IMU] Sensor não responde no endereço 0x%02X (resultado: %d)\n", mpu6050_addr, result);
        return false;
    }
}

float imu_accel_raw_to_g(int16_t raw_value) {
    return (float)raw_value * ACCEL_SCALE_FACTOR;
}

float imu_gyro_raw_to_dps(int16_t raw_value) {
    return (float)raw_value * GYRO_SCALE_FACTOR;
}

float imu_temp_raw_to_celsius(int16_t raw_value) {
    return ((float)raw_value * TEMP_SCALE_FACTOR) + TEMP_OFFSET;
}