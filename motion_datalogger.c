#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/rtc.h"
#include "ff.h"
#include "diskio.h"
#include "f_util.h"
#include "hw_config.h"
#include "my_debug.h"
#include "rtc.h"
#include "sd_card.h"
#include "imu.h"

// ===================== DEFINIÇÕES DE HARDWARE =====================
#define I2C_PORT_SENSORES i2c0
#define I2C_SENS_SDA 0
#define I2C_SENS_SCL 1
#define I2C_PORT_DISPLAY i2c1
#define I2C_DISP_SDA 14
#define I2C_DISP_SCL 15
#define DISPLAY_ADDRESS 0x3C

#define LED_RED_PIN 13
#define LED_GREEN_PIN 11
#define LED_BLUE_PIN 12
#define BUZZER_PIN 21
#define PWM_DIVISOR 50
#define PWM_WRAP_VALUE 4000

#define BTN_1 5  // Iniciar/Parar captura
#define BTN_2 6  // Montar/Desmontar SD
#define BTN_3 22 // Modo BOOTSEL

#define BOOTSEL_BTN BTN_3 // Pino 22 para BOOTSEL

// ===================== CONSTANTES DO SISTEMA =====================
#define MAX_FILENAME_LEN 32
#define CSV_HEADER "numero_amostra,accel_x,accel_y,accel_z,giro_x,giro_y,giro_z\n"
#define SAMPLE_PERIOD_MS 100  // 10Hz de amostragem
#define DEBOUNCE_TIME_MS 200  // Aumentado para melhor debounce
#define BEEP_DURATION_MS 100

// ===================== ESTADOS DO SISTEMA =====================
typedef enum {
    STATE_INITIALIZING,
    STATE_WAITING,
    STATE_CAPTURING,
    STATE_SD_ACCESS,
    STATE_ERROR
} system_state_t;

typedef enum {
    COLOR_OFF,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_YELLOW,
    COLOR_PURPLE
} led_color_t;

// ===================== ESTRUTURA PARA FILA DE BEEPS =====================
typedef struct {
    uint16_t frequency;
    uint32_t duration;
} beep_t;

#define MAX_BEEP_QUEUE 5
static beep_t beep_queue[MAX_BEEP_QUEUE];
static volatile int beep_queue_head = 0;
static volatile int beep_queue_tail = 0;
static volatile int beep_queue_count = 0;

// ===================== ESTRUTURA PARA EVENTOS DE BOTÃO =====================
typedef enum {
    BTN_EVENT_NONE,
    BTN_EVENT_START_STOP,
    BTN_EVENT_MOUNT_UNMOUNT,
    BTN_EVENT_BOOTSEL
} button_event_t;

// ===================== VARIÁVEIS GLOBAIS =====================
static volatile system_state_t current_state = STATE_INITIALIZING;
static volatile system_state_t previous_state = STATE_INITIALIZING;
static volatile bool sd_mounted = false;
static volatile bool capturing = false;
static volatile uint32_t sample_count = 0;
static volatile uint32_t last_btn1_time = 0;
static volatile uint32_t last_btn2_time = 0;

// Variável para eventos de botão (processamento assíncrono)
static volatile button_event_t pending_button_event = BTN_EVENT_NONE;
static volatile bool button_handler_busy = false;
static volatile uint32_t last_event_time = 0;  // Tempo do último evento processado
static volatile uint32_t event_processing_delay = 500;  // Delay mínimo entre eventos (ms)

// Variáveis para controle de piscadas do LED
static volatile bool blinking = false;
static volatile led_color_t blink_color = COLOR_OFF;
static volatile int blink_times = 0;
static volatile int current_blink_count = 0;
static volatile bool blink_led_state = false;
static volatile uint32_t last_blink_time = 0;

// Variáveis para controle do buzzer
static volatile bool buzzing = false;
static volatile uint32_t buzz_start_time = 0;
static volatile uint32_t buzz_duration = 0;

static char csv_filename[MAX_FILENAME_LEN];
static FIL csv_file;
static absolute_time_t next_sample_time;

// ===================== FUNÇÕES DE HARDWARE =====================

/**
 * @brief Configura o LED RGB
 */
void setup_rgb_led(void) {
    gpio_init(LED_RED_PIN);
    gpio_init(LED_GREEN_PIN);
    gpio_init(LED_BLUE_PIN);
    
    gpio_set_dir(LED_RED_PIN, GPIO_OUT);
    gpio_set_dir(LED_GREEN_PIN, GPIO_OUT);
    gpio_set_dir(LED_BLUE_PIN, GPIO_OUT);
    
    // Inicializa LEDs apagados
    gpio_put(LED_RED_PIN, 0);
    gpio_put(LED_GREEN_PIN, 0);
    gpio_put(LED_BLUE_PIN, 0);
}

/**
 * @brief Define a cor do LED RGB
 * @param color Cor desejada
 */
void set_led_color(led_color_t color) {
    // Apaga todos os LEDs primeiro
    gpio_put(LED_RED_PIN, 0);
    gpio_put(LED_GREEN_PIN, 0);
    gpio_put(LED_BLUE_PIN, 0);
    
    switch (color) {
        case COLOR_RED:
            gpio_put(LED_RED_PIN, 1);
            break;
        case COLOR_GREEN:
            gpio_put(LED_GREEN_PIN, 1);
            break;
        case COLOR_BLUE:
            gpio_put(LED_BLUE_PIN, 1);
            break;
        case COLOR_YELLOW:
            gpio_put(LED_RED_PIN, 1);
            gpio_put(LED_GREEN_PIN, 1);
            break;
        case COLOR_PURPLE:
            gpio_put(LED_RED_PIN, 1);
            gpio_put(LED_BLUE_PIN, 1);
            break;
        case COLOR_OFF:
        default:
            // Já apagados
            break;
    }
}

/**
 * @brief Inicia um ciclo de piscadas do LED
 * @param color Cor do LED
 * @param times Número de piscadas
 */
void start_blinking(led_color_t color, int times) {
    if (!blinking) {
        blinking = true;
        blink_color = color;
        blink_times = times * 2; // Multiplica por 2 para contar liga/desliga
        current_blink_count = 0;
        blink_led_state = false;
        last_blink_time = to_ms_since_boot(get_absolute_time());
        set_led_color(COLOR_OFF); // Garante que começa apagado
    }
}

/**
 * @brief Atualiza o estado do LED para piscadas
 */
void update_blinking(void) {
    if (!blinking) {
        return;
    }

    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (current_time - last_blink_time >= 300) {
        last_blink_time = current_time;

        if (blink_led_state) {
            set_led_color(COLOR_OFF);
            blink_led_state = false;
        } else {
            set_led_color(blink_color);
            blink_led_state = true;
        }
        current_blink_count++;

        if (current_blink_count >= blink_times) {
            blinking = false;
            set_led_color(COLOR_OFF);
            current_blink_count = 0;
        }
    }
}

/**
 * @brief Configura o buzzer
 */
void setup_buzzer(void) {
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_set_clkdiv(slice_num, PWM_DIVISOR);
    pwm_set_wrap(slice_num, PWM_WRAP_VALUE);
    pwm_set_enabled(slice_num, true);
}

/**
 * @brief Adiciona um beep na fila
 * @param frequency Frequência em Hz
 * @param duration Duração em ms
 */
void queue_beep(uint16_t frequency, uint32_t duration) {
    if (beep_queue_count < MAX_BEEP_QUEUE) {
        beep_queue[beep_queue_tail].frequency = frequency;
        beep_queue[beep_queue_tail].duration = duration;
        beep_queue_tail = (beep_queue_tail + 1) % MAX_BEEP_QUEUE;
        beep_queue_count++;
    }
}

/**
 * @brief Inicia o próximo beep da fila
 */
void start_next_beep(void) {
    if (!buzzing && beep_queue_count > 0) {
        beep_t next_beep = beep_queue[beep_queue_head];
        beep_queue_head = (beep_queue_head + 1) % MAX_BEEP_QUEUE;
        beep_queue_count--;
        
        buzzing = true;
        buzz_start_time = to_ms_since_boot(get_absolute_time());
        buzz_duration = next_beep.duration;

        uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
        uint16_t level = (next_beep.frequency > 0) ? PWM_WRAP_VALUE / 2 : 0;
        pwm_set_gpio_level(BUZZER_PIN, level);
    }
}

/**
 * @brief Atualiza o estado do buzzer
 */
void update_buzzer(void) {
    if (buzzing) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        if (current_time - buzz_start_time >= buzz_duration) {
            pwm_set_gpio_level(BUZZER_PIN, 0);
            buzzing = false;
            // Inicia o próximo beep da fila, se houver
            start_next_beep();
        }
    } else {
        // Se não está tocando, verifica se há beeps na fila
        start_next_beep();
    }
}

/**
 * @brief Configura os botões com interrupções
 */
void setup_buttons(void);

/**
 * @brief Handler de interrupção para botões
 */
void button_irq_handler(uint gpio, uint32_t events);

// ===================== FUNÇÕES DO CARTÃO SD =====================

/**
 * @brief Obtém o ponteiro do cartão SD por nome
 */
static sd_card_t *sd_get_by_name(const char *const name) {
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return sd_get_by_num(i);
    printf("[SD] Nome desconhecido: %s\n", name);
    return NULL;
}

/**
 * @brief Obtém o sistema de arquivos por nome
 */
static FATFS *sd_get_fs_by_name(const char *name) {
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return &sd_get_by_num(i)->fatfs;
    printf("[SD] Nome desconhecido: %s\n", name);
    return NULL;
}

/**
 * @brief Monta o cartão SD
 * @return true se montagem foi bem-sucedida
 */
bool mount_sd_card(void) {
    printf("[SD] Montando cartão SD...\n");
    
    // Verifica se já está montado
    if (sd_mounted) {
        printf("[SD] Cartão SD já está montado\n");
        return true;
    }
    
    const char *drive_name = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(drive_name);
    
    if (!p_fs) {
        printf("[SD] Erro: drive desconhecido\n");
        return false;
    }
    
    printf("[SD] Executando f_mount...\n");
    FRESULT fr = f_mount(p_fs, drive_name, 1);
    if (FR_OK != fr) {
        printf("[SD] Erro f_mount: %s (%d)\n", FRESULT_str(fr), fr);
        return false;
    }
    
    printf("[SD] f_mount concluído, atualizando status...\n");
    sd_card_t *pSD = sd_get_by_name(drive_name);
    if (pSD) {
        pSD->mounted = true;
        sd_mounted = true;
        printf("[SD] Montagem concluída: %s\n", pSD->pcName);
        return true;
    }
    
    printf("[SD] Erro: ponteiro do cartão SD é nulo\n");
    return false;
}

/**
 * @brief Desmonta o cartão SD
 * @return true se desmontagem foi bem-sucedida
 */
bool unmount_sd_card(void) {
    printf("[SD] Desmontando cartão SD...\n");
    
    // Verifica se já está desmontado
    if (!sd_mounted) {
        printf("[SD] Cartão SD já está desmontado\n");
        return true;
    }
    
    const char *drive_name = sd_get_by_num(0)->pcName;
    
    printf("[SD] Executando f_unmount...\n");
    FRESULT fr = f_unmount(drive_name);
    if (FR_OK != fr) {
        printf("[SD] Erro f_unmount: %s (%d)\n", FRESULT_str(fr), fr);
        return false;
    }
    
    printf("[SD] f_unmount concluído, atualizando status...\n");
    sd_card_t *pSD = sd_get_by_name(drive_name);
    if (pSD) {
        pSD->mounted = false;
        pSD->m_Status |= STA_NOINIT;
        sd_mounted = false;
        printf("[SD] Desmontagem concluída: %s\n", pSD->pcName);
        return true;
    }
    
    printf("[SD] Erro: ponteiro do cartão SD é nulo\n");
    return false;
}

// ===================== FUNÇÕES DE CAPTURA DE DADOS =====================

/**
 * @brief Gera nome único para arquivo CSV
 */
void generate_csv_filename(void) {
    static int file_counter = 1;
    snprintf(csv_filename, MAX_FILENAME_LEN, "imu_data_%03d.csv", file_counter++);
}

/**
 * @brief Inicializa arquivo CSV
 * @return true se inicialização foi bem-sucedida
 */
bool initialize_csv_file(void) {
    if (!sd_mounted) {
        printf("[CSV] Erro: SD não montado\n");
        return false;
    }
    
    generate_csv_filename();
    
    FRESULT res = f_open(&csv_file, csv_filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK) {
        printf("[CSV] Erro ao criar arquivo: %s (%d)\n", FRESULT_str(res), res);
        return false;
    }
    
    // Escreve cabeçalho CSV
    UINT bytes_written;
    res = f_write(&csv_file, CSV_HEADER, strlen(CSV_HEADER), &bytes_written);
    if (res != FR_OK || bytes_written != strlen(CSV_HEADER)) {
        printf("[CSV] Erro ao escrever cabeçalho: %s (%d)\n", FRESULT_str(res), res);
        f_close(&csv_file);
        return false;
    }
    
    printf("[CSV] Arquivo criado: %s\n", csv_filename);
    return true;
}

/**
 * @brief Salva uma amostra no arquivo CSV
 * @param sample_num Número da amostra
 * @param imu_data Dados do IMU
 * @return true se salvamento foi bem-sucedido
 */
bool save_sample_to_csv(uint32_t sample_num, const imu_data_t *imu_data) {
    if (!sd_mounted) {
        return false;
    }
    
    char line_buffer[128];
    snprintf(line_buffer, sizeof(line_buffer), 
             "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
             sample_num,
             imu_data->accel.x, imu_data->accel.y, imu_data->accel.z,
             imu_data->gyro.x, imu_data->gyro.y, imu_data->gyro.z);
    
    UINT bytes_written;
    FRESULT res = f_write(&csv_file, line_buffer, strlen(line_buffer), &bytes_written);
    
    if (res != FR_OK || bytes_written != strlen(line_buffer)) {
        printf("[CSV] Erro ao escrever amostra: %s (%d)\n", FRESULT_str(res), res);
        return false;
    }
    
    return true;
}

/**
 * @brief Finaliza e fecha arquivo CSV
 */
void finalize_csv_file(void) {
    if (sd_mounted) {
        f_close(&csv_file);
        printf("[CSV] Arquivo finalizado: %s (%lu amostras)\n", csv_filename, sample_count);
    }
}

// ===================== MÁQUINA DE ESTADOS =====================

/**
 * @brief Atualiza o estado do sistema baseado no estado atual
 */
void update_system_state(void) {
    if (current_state == previous_state) {
        return;
    }
    
    switch (current_state) {
        case STATE_INITIALIZING:
            set_led_color(COLOR_YELLOW);
            printf("[SISTEMA] Estado: Inicializando...\n");
            break;
            
        case STATE_WAITING:
            if (!blinking) { // Só muda LED se não estiver piscando
                set_led_color(COLOR_GREEN);
            }
            printf("[SISTEMA] Estado: Aguardando comando\n");
            break;
            
        case STATE_CAPTURING:
            if (!blinking) { // Só muda LED se não estiver piscando
                set_led_color(COLOR_RED);
            }
            printf("[SISTEMA] Estado: Capturando dados...\n");
            break;
            
        case STATE_SD_ACCESS:
            start_blinking(COLOR_BLUE, 1);
            printf("[SISTEMA] Estado: Acessando cartão SD...\n");
            break;
            
        case STATE_ERROR:
            start_blinking(COLOR_PURPLE, 3);
            printf("[SISTEMA] Estado: ERRO - verifique conexões\n");
            break;
    }
    
    previous_state = current_state;
}

/**
 * @brief Inicia captura de dados
 * @return true se início foi bem-sucedido
 */
bool start_data_capture(void) {
    if (capturing) {
        printf("[CAPTURA] Já está capturando\n");
        return false;
    }
    
    if (!sd_mounted) {
        printf("[CAPTURA] Erro: SD não montado\n");
        current_state = STATE_ERROR;
        update_system_state();
        return false;
    }
    
    if (!imu_is_available()) {
        printf("[CAPTURA] Erro: IMU não disponível\n");
        current_state = STATE_ERROR;
        update_system_state();
        return false;
    }
    
    printf("[CAPTURA] Inicializando arquivo CSV...\n");
    current_state = STATE_SD_ACCESS;
    update_system_state();
    
    if (!initialize_csv_file()) {
        printf("[CAPTURA] Erro ao inicializar arquivo CSV\n");
        current_state = STATE_ERROR;
        update_system_state();
        return false;
    }
    
    sample_count = 0;
    capturing = true;
    current_state = STATE_CAPTURING;
    update_system_state();
    next_sample_time = make_timeout_time_ms(SAMPLE_PERIOD_MS);
    
    queue_beep(1000, BEEP_DURATION_MS);
    printf("[CAPTURA] Captura iniciada com sucesso\n");
    
    return true;
}

/**
 * @brief Para captura de dados
 */
void stop_data_capture(void) {
    if (!capturing) {
        printf("[CAPTURA] Não está capturando\n");
        return;
    }
    
    capturing = false;
    current_state = STATE_SD_ACCESS;
    update_system_state();
    
    finalize_csv_file();
    
    current_state = STATE_WAITING;
    
    // Beeps duplos para indicar parada - agora usando fila
    queue_beep(1000, BEEP_DURATION_MS);
    queue_beep(1000, BEEP_DURATION_MS);
    
    printf("[CAPTURA] Captura finalizada (%lu amostras)\n", sample_count);
}

// ===================== HANDLERS DE INTERRUPÇÃO =====================

// ===================== HANDLERS DE INTERRUPÇÃO =====================

void button_irq_handler(uint gpio, uint32_t events) {
    // Handler simplificado - apenas registra o evento
    if (button_handler_busy || pending_button_event != BTN_EVENT_NONE) {
        return; // Ignora se já está processando ou há evento pendente
    }
    
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    if (gpio == BTN_1) {
        if (current_time - last_btn1_time < DEBOUNCE_TIME_MS) {
            return;
        }
        last_btn1_time = current_time;
        pending_button_event = BTN_EVENT_START_STOP;
        printf("[IRQ] Evento BTN_1 registrado\n");
        
    } else if (gpio == BTN_2) {
        if (current_time - last_btn2_time < DEBOUNCE_TIME_MS) {
            return;
        }
        last_btn2_time = current_time;
        pending_button_event = BTN_EVENT_MOUNT_UNMOUNT;
        printf("[IRQ] Evento BTN_2 registrado\n");
        
    } else if (gpio == BTN_3) {
        pending_button_event = BTN_EVENT_BOOTSEL;
        printf("[IRQ] Evento BTN_3 registrado\n");
    }
}

/**
 * @brief Processa eventos de botão de forma assíncrona
 */
void process_button_events(void) {
    if (pending_button_event == BTN_EVENT_NONE || button_handler_busy) {
        return;
    }
    
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    // Delay mínimo entre processamento de eventos
    if (current_time - last_event_time < event_processing_delay) {
        return;
    }
    
    button_handler_busy = true;
    button_event_t event = pending_button_event;
    pending_button_event = BTN_EVENT_NONE;
    last_event_time = current_time;
    
    printf("[EVENTO] Processando evento tipo %d\n", event);
    
    switch (event) {
        case BTN_EVENT_START_STOP:
            printf("[BOTAO] Processando: Iniciar/Parar captura\n");
            if (!capturing) {
                printf("[BOTAO] Estado atual: NÃO capturando - iniciando...\n");
                if (start_data_capture()) {
                    printf("[BOTAO] Captura iniciada com sucesso\n");
                } else {
                    printf("[BOTAO] Erro ao iniciar captura\n");
                }
            } else {
                printf("[BOTAO] Estado atual: CAPTURANDO - parando...\n");
                stop_data_capture();
                printf("[BOTAO] Captura finalizada\n");
            }
            break;
            
        case BTN_EVENT_MOUNT_UNMOUNT:
            printf("[BOTAO] Processando: Montar/Desmontar SD\n");
            if (!sd_mounted) {
                printf("[BOTAO] Tentando montar SD...\n");
                current_state = STATE_SD_ACCESS;
                update_system_state();
                
                if (mount_sd_card()) {
                    current_state = STATE_WAITING;
                    update_system_state();
                    queue_beep(800, BEEP_DURATION_MS);
                    printf("[BOTAO] SD montado com sucesso\n");
                } else {
                    current_state = STATE_ERROR;
                    update_system_state();
                    queue_beep(400, 200);
                    printf("[BOTAO] Erro ao montar SD\n");
                }
            } else {
                if (!capturing) {
                    printf("[BOTAO] Tentando desmontar SD...\n");
                    current_state = STATE_SD_ACCESS;
                    update_system_state();
                    
                    if (unmount_sd_card()) {
                        current_state = STATE_WAITING;
                        update_system_state();
                        queue_beep(600, BEEP_DURATION_MS);
                        printf("[BOTAO] SD desmontado com sucesso\n");
                    } else {
                        current_state = STATE_ERROR;
                        update_system_state();
                        queue_beep(400, 200);
                        printf("[BOTAO] Erro ao desmontar SD\n");
                    }
                } else {
                    printf("[BOTAO] Não é possível desmontar durante captura\n");
                    queue_beep(400, 200);
                }
            }
            break;
            
        case BTN_EVENT_BOOTSEL:
            printf("[BOTAO] Ativando modo BOOTSEL...\n");
            reset_usb_boot(0, 0);
            break;
            
        default:
            printf("[EVENTO] Evento desconhecido: %d\n", event);
            break;
    }
    
    printf("[EVENTO] Evento processado. Próximo evento em %lu ms\n", event_processing_delay);
    button_handler_busy = false;
}

void setup_buttons(void) {
    gpio_init(BTN_1);
    gpio_init(BTN_2);
    gpio_init(BTN_3);
    
    gpio_set_dir(BTN_1, GPIO_IN);
    gpio_set_dir(BTN_2, GPIO_IN);
    gpio_set_dir(BTN_3, GPIO_IN);
    
    gpio_pull_up(BTN_1);
    gpio_pull_up(BTN_2);
    gpio_pull_up(BTN_3);
    
    gpio_set_irq_enabled_with_callback(BTN_1, GPIO_IRQ_EDGE_FALL, true, &button_irq_handler);
    gpio_set_irq_enabled_with_callback(BTN_2, GPIO_IRQ_EDGE_FALL, true, &button_irq_handler);
    gpio_set_irq_enabled_with_callback(BTN_3, GPIO_IRQ_EDGE_FALL, true, &button_irq_handler);
    
    printf("[BOTOES] Botões configurados:\n");
    printf("[BOTOES] - Botão 1 (pino %d): Iniciar/Parar captura\n", BTN_1);
    printf("[BOTOES] - Botão 2 (pino %d): Montar/Desmontar SD\n", BTN_2);
    printf("[BOTOES] - Botão 3 (pino %d): Modo BOOTSEL\n", BTN_3);
}

// ===================== LOOP PRINCIPAL DE CAPTURA =====================

/**
 * @brief Processa uma amostra do IMU
 */
void process_imu_sample(void) {
    if (!capturing) {
        return;
    }
    
    if (!time_reached(next_sample_time)) {
        return;
    }
    
    next_sample_time = make_timeout_time_ms(SAMPLE_PERIOD_MS);
    
    imu_data_t imu_data;
    if (!imu_read_data(&imu_data)) {
        printf("[AMOSTRA] Erro ao ler dados do IMU\n");
        current_state = STATE_ERROR;
        capturing = false;
        return;
    }
    
    sample_count++;
    
    current_state = STATE_SD_ACCESS;
    if (!save_sample_to_csv(sample_count, &imu_data)) {
        printf("[AMOSTRA] Erro ao salvar amostra %lu\n", sample_count);
        current_state = STATE_ERROR;
        capturing = false;
        return;
    }
    current_state = STATE_CAPTURING;
    
    if (sample_count % 10 == 0) {
        printf("[AMOSTRA] %lu amostras coletadas\n", sample_count);
        printf("  Accel: X=%.2f Y=%.2f Z=%.2f (g)\n", 
               imu_data.accel.x, imu_data.accel.y, imu_data.accel.z);
        printf("  Gyro:  X=%.2f Y=%.2f Z=%.2f (°/s)\n", 
               imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z);
        printf("  Temp:  %.1f°C\n", imu_data.temperature);
    }
}

// ===================== FUNÇÃO PRINCIPAL =====================

int main(void) {
    stdio_init_all();
    sleep_ms(5000);
    
    printf("\n" 
           "========================================\n"
           "  SISTEMA DE REGISTRO DE MOVIMENTO IMU\n"
           "========================================\n\n");
    
    current_state = STATE_INITIALIZING;
    update_system_state();
    
    printf("[INIT] Configurando hardware...\n");
    setup_rgb_led();
    setup_buzzer();
    setup_buttons();
    
    printf("[INIT] Inicializando IMU...\n");
    printf("[INIT] Verificando configuração I2C...\n");
    printf("[INIT] Porta I2C: %s\n", I2C_PORT_SENSORES == i2c0 ? "i2c0" : "i2c1");
    printf("[INIT] Pinos: SDA=%d, SCL=%d\n", I2C_SENS_SDA, I2C_SENS_SCL);
    printf("[INIT] Endereço MPU6050: 0x%02X\n", 0x68);
    
    if (!imu_init(I2C_PORT_SENSORES, I2C_SENS_SDA, I2C_SENS_SCL)) {
        printf("[INIT] ERRO: Falha ao inicializar IMU\n");
        printf("[INIT] Possíveis causas:\n");
        printf("[INIT] - Sensor não conectado corretamente\n");
        printf("[INIT] - Pinos SDA/SCL incorretos\n");
        printf("[INIT] - Sensor com defeito\n");
        printf("[INIT] - Problema na alimentação do sensor\n");
        current_state = STATE_ERROR;
        update_system_state();
        
        while (true) {
            update_blinking();
            update_buzzer();
            sleep_ms(10);
        }
    }
    
    printf("[INIT] Tentando montar cartão SD...\n");
    current_state = STATE_SD_ACCESS;
    update_system_state();
    
    if (mount_sd_card()) {
        printf("[INIT] Cartão SD montado com sucesso\n");
    } else {
        printf("[INIT] Aviso: Cartão SD não foi montado automaticamente\n");
        printf("[INIT] Use o botão 2 para tentar montar manualmente\n");
    }
    
    current_state = STATE_WAITING;
    update_system_state();
    
    printf("\n[INIT] Sistema inicializado com sucesso!\n");
    printf("Controles:\n");
    printf("  Botão 1 (pino %d): Iniciar/Parar captura\n", BTN_1);
    printf("  Botão 2 (pino %d): Montar/Desmontar SD\n", BTN_2);
    printf("  Botão 3 (pino %d): Reiniciar no modo USB (BOOTSEL)\n", BTN_3);
    printf("\n");
    
    queue_beep(1500, 200);
    
    printf("[MAIN] Entrando no loop principal...\n");
    uint32_t last_status_time = 0;
    const uint32_t STATUS_INTERVAL_MS = 30000;
    
    while (true) {
        update_system_state();
        update_blinking();
        update_buzzer();
        process_button_events(); // Processa eventos de botão de forma assíncrona
        process_imu_sample();
        
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        if (current_time - last_status_time > STATUS_INTERVAL_MS) {
            last_status_time = current_time;
            if (current_state != STATE_ERROR) {
                printf("[STATUS] Sistema ativo - Estado: %s, SD: %s, Amostras: %lu\n",
                       current_state == STATE_WAITING ? "Aguardando" :
                       current_state == STATE_CAPTURING ? "Capturando" :
                       current_state == STATE_SD_ACCESS ? "Acessando SD" : "Desconhecido",
                       sd_mounted ? "Montado" : "Desmontado",
                       sample_count);
            }
        }
        
        sleep_ms(10);
    }
    
    return 0;
}