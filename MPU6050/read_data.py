import serial
import serial.tools.list_ports
import numpy as np
import pandas as pd
import time

tempo = int(input("Tempo de teste (s): "))
lenght = 512*tempo
acel_data = np.zeros(lenght)

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
    acel_data[i] = float(line.decode().strip())
    print(acel_data[i])

choice = int(input("Deseja salvar os dados? [1] Sim [2] Não\n"))
if choice == 1:
    file_name = 'dados\\' + str(input("Nome do arquivo para salvar os dados: ")) + '.csv'

    df = pd.DataFrame(acel_data, columns=['Aceleração Vertical'])
    df.to_csv(file_name, index=False)

    print("Dados salvos em " + file_name)

else:
    print("Encerrando a comunicação serial.")
    ser.close()
