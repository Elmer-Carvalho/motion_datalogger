# Motion Datalogger

## 📘 Introdução

O **Motion Datalogger** é um sistema embarcado baseado no microcontrolador **RP2040** que realiza a aquisição de dados de movimento através de um sensor IMU (Unidade de Medida Inercial) e os armazena em um cartão microSD. O projeto inclui um firmware para a placa RP2040 e um script Python para visualização dos dados coletados.

### ✨ Principais Recursos
- Amostragem de dados de aceleração e giroscópio
- Armazenamento em arquivos CSV no cartão microSD
- Interface simples com 3 botões para controle
- Feedback visual através de LED RGB
- Script Python para visualização dos dados coletados

## 🛠️ Componentes Necessários

### Hardware
- Placa com RP2040 (ex: Raspberry Pi Pico, BitDogLab)
- Módulo IMU MPU6050 (acelerômetro + giroscópio)
- Módulo leitor de cartão microSD
- Cartão microSD formatado como FAT32
- 3 botões push-button
- LED RGB
- Buzzer passivo
- Protoboard e jumpers
- Cabo micro-USB

### Software
- [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)
- [CMake](https://cmake.org/) (versão 3.12 ou superior)
- [GNU Arm Embedded Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm)
- [Python 3.8+](https://www.python.org/downloads/)
- Bibliotecas Python (instaláveis via `pip install -r requirements.txt`):
  - pandas
  - matplotlib
  - seaborn

## 📁 Estrutura do Projeto

```
Motion_Datalogger/
├── motion_datalogger.c    # Código principal do firmware
├── graph_plotter.py      # Script Python para visualização de dados
├── CMakeLists.txt        # Configuração de build
├── pico_sdk_import.cmake # Configuração do Pico SDK
├── requirements.txt      # Dependências Python
├── arquivos_csv/         # Diretório para armazenar os dados coletados
└── lib/                  # Bibliotecas auxiliares
    ├── ssd1306.[ch]     # Driver para display OLED
    ├── imu.[ch]         # Driver para o sensor IMU
    ├── hw_config.c       # Configuração de hardware
    └── font.h           # Fonte para o display
```

## 🔌 Conexões do Hardware

| Componente        | Pino RP2040 | Função                     |
|-------------------|-------------|----------------------------|
| IMU SDA           | GP0         | Dados I2C                  |
| IMU SCL           | GP1         | Clock I2C                  |
| Display OLED SDA   | GP14        | Dados I2C                  |
| Display OLED SCL   | GP15        | Clock I2C                  |
| LED Vermelho       | GP13        |                            |
| LED Verde          | GP11        |                            |
| LED Azul           | GP12        |                            |
| Buzzer             | GP21        |                            |
| Botão 1            | GP5         | Iniciar/Parar captura      |
| Botão 2            | GP6         | Montar/Desmontar cartão SD |
| Botão 3            | GP22        | Modo BOOTSEL (reset/flash) |

## 🚀 Compilação e Upload

### Pré-requisitos
1. Instale o Pico SDK seguindo as [instruções oficiais](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf)
2. Configure as variáveis de ambiente necessárias:
   ```bash
   export PICO_SDK_PATH=/caminho/para/pico-sdk
   ```

### Compilação
1. Crie um diretório de build e entre nele:
   ```bash
   mkdir build
   cd build
   ```
2. Execute o CMake e compile o projeto:
   ```bash
   cmake ..
   make -j4
   ```

### Upload para a placa
1. Mantenha pressionado o botão BOOTSEL na placa e conecte-a ao computador via USB
2. Solte o botão BOOTSEL
3. Um dispositivo de armazenamento aparecerá no seu computador
4. Copie o arquivo `motion_datalogger.uf2` do diretório `build` para o dispositivo

## 🎮 Como Usar

### Controles
- **Botão 1 (GP5)**: Inicia/para a captura de dados
- **Botão 2 (GP6)**: Monta/desmonta o cartão SD
- **Botão 3 (GP22)**: Entra no modo BOOTSEL para upload de firmware

### LED de Status
- **Vermelho**: Erro (cartão SD não montado, IMU não encontrado)
- **Azul**: Pronto para captura
- **Verde**: Capturando dados
- **Roxo**: Salvando dados no cartão

## 📊 Visualização dos Dados

### Pré-requisitos
Instale as dependências Python:
```bash
pip install -r requirements.txt
```

### Executando o Visualizador
1. Copie o arquivo CSV do cartão SD para o diretório `arquivos_csv/`
2. Edite a variável `CSV_FILE_PATH` no script `graph_plotter.py` para apontar para seu arquivo CSV
3. Execute o script:
   ```bash
   python graph_plotter.py
   ```

### Funcionalidades do Visualizador
- Gráficos de aceleração (X, Y, Z)
- Gráficos de velocidade angular (X, Y, Z)
- Estatísticas básicas dos dados
- Interface simples para navegação

## 🔧 Solução de Problemas

### Cartão SD não é detectado
- Verifique as conexões do módulo SD
- Certifique-se de que o cartão está formatado como FAT32
- Tente outro cartão SD

### IMU não é detectado
- Verifique as conexões I2C
- Certifique-se de que os pull-ups estão presentes nos barramentos SDA/SCL
- Verifique o endereço I2C do dispositivo

### Erros ao compilar
- Verifique se todas as dependências estão instaladas
- Confirme que o PICO_SDK_PATH está configurado corretamente
- Tente limpar o diretório de build e recompilar

## 📝 Licença

Este projeto está licenciado sob a licença MIT - veja o arquivo [LICENSE](LICENSE) para detalhes.

## 🙋‍♂️ Suporte

Para suporte, por favor abra uma issue no repositório do projeto.