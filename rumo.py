import numpy as np


def cria_estimador_rumo(t0, rumo0, incerteza0, callback, nome_estimador="bussola"):
    if nome_estimador == "bussola":
        return Bussola(callback, t0, rumo0, incerteza0)
    # Adicione novos estimadores aqui:
    # Exemplo:
    # eliif nome_estimador=="estimador2":
    #    return Estimador2(callback, t0, rumo0, incerteza0)
    else:
        raise ValueError(f"Estimador {nome_estimador} desconhecido!")


class Bussola:
    def __init__(self, callback, t0, rumo0, incerteza0):
        self._angulo = rumo0
        self._t = t0
        self._nova_previsao = callback
        f_s = 20
        fc = 1 / 4

        self.alpha = np.exp(-2 * np.pi * fc / f_s)
        self.H = np.array([-111.01913801, 95.68052301, -81.49760907])
        self.W = np.array(
            [
                [-1.12530864, 228.18545831, -36.37830228],
                [-236.60386049, 1.47686114, -6.29477842],
                [5.62134137, 19.76745578, 242.07230095],
            ]
        )

        pass

    def processa_dados(self, t, mx, my, mz):
        m = np.array([mx, my, mz]) - self.H
        corrigido = self.W @ m
        angulo_cur = np.arctan2(corrigido[0], corrigido[1])

        di = (angulo_cur - self._angulo + np.pi) % (2 * np.pi) - np.pi
        self._angulo = self._angulo + (1 - self.alpha) * di
        # t é o timestamp em picossegundos,
        # mx, my, mz são as coletas "brutas" do magnetômetro
        # Calcule o tempo e estimativa de rumo (em radianos!)
        # Notifique a nova estimativa com:
        self._nova_previsao(t / 10e9, self._angulo, 0, 0)
        pass

    def callback(self, data):
        wx = int.from_bytes(data[8:10], byteorder="big", signed=True)
        wy = int.from_bytes(data[10:12], byteorder="big", signed=True)
        wz = int.from_bytes(data[12:14], byteorder="big", signed=True)
        mx = int.from_bytes(data[14:16], byteorder="big", signed=True)
        my = int.from_bytes(data[16:18], byteorder="big", signed=True)
        mz = int.from_bytes(data[18:20], byteorder="big", signed=True)
        t = int.from_bytes(data[-8:], byteorder="little", signed=False)
        self.processa_dados(t, mx, my, mz)
