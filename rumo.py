import numpy as np


def cria_estimador_rumo(t0, rumo0, incerteza0, callback, nome_estimador="kf"):

    if nome_estimador == "bussola":
        return Bussola(callback, t0, rumo0, incerteza0)

    elif nome_estimador == "kf":
        return KalmanFilter(callback, t0, rumo0, incerteza0)

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


class KalmanFilter(object):

    ## Deve ser passado o valor de sigma_omega**2 na incerteza
    def __init__(self, callback, t0, rumo0, incerteza0):

        self._callback = callback

        # Estado inicial: [theta, omega]
        self._mu = np.array([rumo0, 0.0])  # rumo inicial, velocidade angular 0
        self._sigma = np.array(
            [[incerteza0, 0.0], [0.0, 0.0]]
        )  # incerteza no rumo apenas; incerteza0 já está ao quadrado
        self._t = t0  # instante inicial

        # Parâmetros do filtro    refazer os comentários
        self._sigma_omega = 0.000673185929231597
        self._sigma_theta = 6.669357769704175 / 180 * np.pi
        self._Q = self._sigma_theta  # ruído de medida do magnetômetro    ### renomear

        self.H = np.array([-111.01913801, 95.68052301, -81.49760907])
        self.W = np.array(
            [
                [-1.12530864, 228.18545831, -36.37830228],
                [-236.60386049, 1.47686114, -6.29477842],
                [5.62134137, 19.76745578, 242.07230095],
            ]
        )
        self.Qcorr = np.array(
            [
                [
                    9.929423666172735308e-01,
                    2.308806532793533303e-03,
                    1.185754021240587597e-01,
                ],
                [
                    2.185407284892839676e-03,
                    -9.999969267261736849e-01,
                    1.170697743613673299e-03,
                ],
                [
                    1.185777406239773291e-01,
                    -9.032998405262466796e-04,
                    -9.929443607161031471e-01,
                ],
            ]
        )
        self.gyoffset = np.array(
            [
                -3.482366666666666788e01,
                5.474000000000000199e00,
                -1.654949999999999832e01,
            ]
        )
        self.gyscale = 0.001064724
        ## Etapa de predição
        # Aqui precisamos das matrizes A, B, R; trabalhamos com o estado atual e a entrada de controle

    def predict(self, dt, vel_ang):  # CHECK

        # Matriz de dinâmica
        A = np.array([[1, dt / 2.0], [0.0, 0.0]])

        # Matriz de atuação
        B = np.array([[dt / 2.0], [1.0]])

        # Matriz de covariância
        R = self._sigma_omega * np.array([[dt**2 / 4.0, dt / 2.0], [dt / 2.0, 1.0]])

        # Entrada de controle
        u = np.array([vel_ang])

        self._mu = A @ self._mu + B @ u
        self._sigma = A @ self._sigma @ A.T + R

        return self._mu

    def angle_diff(self, a, b):

        d = a - b
        normalized = (d + np.pi) % (2 * np.pi) - np.pi
        return normalized

    def update(self, ang_rumo):

        C = np.array([1.0, 0.0]).reshape((1, 2))

        S = C @ self._sigma @ C.T + self._Q

        K = self._sigma @ C.T @ np.linalg.inv(S)

        theta_predicted = C @ self._mu.T

        obser_diff = self.angle_diff(ang_rumo, theta_predicted)

        self._mu = self._mu + K @ obser_diff
        self._sigma = (np.identity(2) - K @ C) @ self._sigma

        return self._mu

    def processa_dados(self, t, mx, my, mz, wx, wy, wz):
        m = np.array([mx, my, mz]) - self.H
        corrigido = self.W @ m
        theta = np.arctan2(corrigido[0], corrigido[1])

        omega = (
            self.Qcorr @ ((np.array([wx, wy, wz]) - self.gyoffset) * self.gyscale)
        )[2]
        self._angulo, a = self.update(theta)
        self._angulo, a = self.predict(1 / 20, omega)
        # print(self._angulo, omega)
        # t é o timestamp em picossegundos,
        # mx, my, mz são as coletas "brutas" do magnetômetro
        # Calcule o tempo e estimativa de rumo (em radianos!)
        # Notifique a nova estimativa com:
        self._callback(t / 10e9, self._angulo, 0, 0)

    def callback(self, data):
        wx = int.from_bytes(data[8:10], byteorder="big", signed=True)
        wy = int.from_bytes(data[10:12], byteorder="big", signed=True)
        wz = int.from_bytes(data[12:14], byteorder="big", signed=True)
        mx = int.from_bytes(data[14:16], byteorder="big", signed=True)
        my = int.from_bytes(data[16:18], byteorder="big", signed=True)
        mz = int.from_bytes(data[18:20], byteorder="big", signed=True)
        t = int.from_bytes(data[-8:], byteorder="little", signed=False)
        self.processa_dados(t, mx, my, mz, wx, wy, wz)
