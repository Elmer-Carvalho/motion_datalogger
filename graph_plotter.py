import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os

# Caminho do arquivo CSV (modifique conforme necessário)
CSV_FILE_PATH = "C:\\Users\\Elmer Carvalho\\Documents\\Projetos-DEV\\EmbarcaTech\\EmbarcaTech_Fase_2\\Motion_Datalogger_2\\arquivos_csv\\imu_data_001.csv"

def read_csv_file(file_path):
    """
    Lê o arquivo CSV e retorna um DataFrame.
    """
    try:
        # Verifica se o arquivo existe
        if not os.path.isfile(file_path):
            raise FileNotFoundError(f"Arquivo '{file_path}' não encontrado.")
        
        # Lê o CSV com pandas
        df = pd.read_csv(file_path)
        
        # Verifica se as colunas esperadas estão presentes
        expected_columns = ['numero_amostra', 'accel_x', 'accel_y', 'accel_z', 'giro_x', 'giro_y', 'giro_z']
        if not all(col in df.columns for col in expected_columns):
            raise ValueError("O arquivo CSV não contém todas as colunas esperadas: " + ", ".join(expected_columns))
        
        return df
    except Exception as e:
        print(f"Erro ao ler o arquivo CSV: {e}")
        return None

def plot_imu_data(df):
    """
    Plota os dados de aceleração e giroscópio em gráficos separados.
    """
    # Configura o estilo dos gráficos
    plt.style.use('seaborn-v0_8')  # Estilo compatível com versões recentes
    
    # Cria figura para aceleração
    plt.figure(figsize=(10, 6))
    plt.plot(df['numero_amostra'], df['accel_x'], label='Accel X (g)', color='r')
    plt.plot(df['numero_amostra'], df['accel_y'], label='Accel Y (g)', color='g')
    plt.plot(df['numero_amostra'], df['accel_z'], label='Accel Z (g)', color='b')
    plt.xlabel('Número da Amostra')
    plt.ylabel('Aceleração (g)')
    plt.title('Dados de Aceleração do MPU6050')
    plt.legend()
    plt.grid(True)
    
    # Cria figura para giroscópio
    plt.figure(figsize=(10, 6))
    plt.plot(df['numero_amostra'], df['giro_x'], label='Giro X (°/s)', color='r')
    plt.plot(df['numero_amostra'], df['giro_y'], label='Giro Y (°/s)', color='g')
    plt.plot(df['numero_amostra'], df['giro_z'], label='Giro Z (°/s)', color='b')
    plt.xlabel('Número da Amostra')
    plt.ylabel('Velocidade Angular (°/s)')
    plt.title('Dados de Giroscópio do MPU6050')
    plt.legend()
    plt.grid(True)
    
    # Exibe os gráficos
    plt.show()

def main():
    # Lê o arquivo CSV
    df = read_csv_file(CSV_FILE_PATH)
    if df is None:
        return
    
    # Plota os dados
    plot_imu_data(df)

if __name__ == "__main__":
    main()
