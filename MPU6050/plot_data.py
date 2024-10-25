import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from tkinter import Tk
from tkinter.filedialog import askopenfilename

Tk().withdraw() 
nome_arquivo = askopenfilename(title='Selecione um arquivo CSV', filetypes=[('CSV files', '*.csv')])

header = 'Aceleração Vertical'

if nome_arquivo:
    dados = (pd.read_csv(nome_arquivo) - 1)*9.81
    tempo_em_segundos = dados.index / 512
    N = len(dados[header])
    Fs = 512
    yf = np.fft.fft(dados[header])
    xf = np.fft.fftfreq(N, 1/Fs)

    fig, axs = plt.subplots(2, 1, figsize=(10, 8))
    axs[0].plot(tempo_em_segundos, dados[header], marker='o', linestyle='-', color='red')
    axs[0].set_title('Aceleração Vertical')
    axs[0].set_xlabel('Tempo [s]')
    axs[0].set_ylabel('Aceleração [m/s²]')
    axs[0].grid()
    
    axs[1].loglog(xf[:N // 2], np.abs(yf[:N // 2]), 'b')
    axs[1].set_title('Espectro de Frequência')
    axs[1].set_xlabel('Frequência [Hz]')
    axs[1].set_ylabel('Magnitude')
    axs[1].grid()
    mask = xf[:N // 2] > 0.001
    axs[1].set_ylim(min(np.abs(yf[:N // 2][mask])) - 0.1, max(np.abs(yf[:N // 2][mask])) + 0.1)
    axs[1].set_xlim(0, Fs / 2)

    plt.tight_layout()
    plt.show()
else:
    print("Nenhum arquivo foi selecionado.")
