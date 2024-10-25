import serial
import serial.tools.list_ports
import numpy as np
import pandas as pd
import time

tempo = int(input("Tempo de teste (s): "))
lenght = 1000*tempo
acel_x = np.zeros(lenght)
altitude = np.zeros(lenght)
altitude_kalman = np.zeros(lenght)
vertical_velocity_kalman = np.zeros(lenght)

ports = serial.tools.list_ports.comports()
ser = serial.Serial(
    port=ports[0].device,
    baudrate=115200
)

print("Conectado em: " + ser.portstr)
print("Começando em...")

cnt = 3
while(cnt > 0):
    print(f"{cnt}...")
    time.sleep(1)
    cnt-=1

for i in range(lenght):
    line = ser.readline()
    data = line.decode().strip().split(" || ")

    # Extraindo os valores de cada variável na ordem esperada
    acel_x[i] = float(data[0].split(": ")[1])
    altitude[i] = float(data[1].split(": ")[1])
    altitude_kalman[i] = float(data[2].split(": ")[1])
    vertical_velocity_kalman[i] = float(data[3].split(": ")[1])

    # Exibindo os valores
    print(f"Acel. X: {acel_x}")
    print(f"Altitude: {altitude}")
    print(f"Altitude Kalman: {altitude_kalman}")
    print(f"Vertical Velocity: {vertical_velocity_kalman}")

choice = int(input("Deseja salvar os dados? [1] Sim [2] Não\n"))
if choice == 1:
    file_name = 'dados\\' + str(input("Nome do arquivo para salvar os dados: ")) + '.csv'

    # Criando o DataFrame com todas as variáveis
    df = pd.DataFrame({
        'Aceleração X': acel_x,
        'Altitude': altitude,
        'Altitude Kalman': altitude_kalman,
        'Velocidade Vertical Kalman': vertical_velocity_kalman
    })
    df.to_csv(file_name, index=False)

    print("Dados salvos em " + file_name)

else:
    print("Encerrando a comunicação serial.")
    ser.close()
