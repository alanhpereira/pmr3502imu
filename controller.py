import numpy as np
import motorctrl
import contextlib


# Cria um controlador de orientação.
#   Este gerador é decorado com contextmanager para garantir
#   que o estado do gpio e pwm seja resetado na saída de escopo
@contextlib.contextmanager
def create_yaw_controller(t0, setpoint):
    # Configuração inicial: output com nível inicial baixo
    print("Criando controle de orientação...", end="")
    with motorctrl.cria_controle_motor() as ctrl:
        yawctrl = Controller(t0, setpoint, 0, ctrl)
        try:
            yield yawctrl
        finally:
            yawctrl.stop()
    print("Feito.")


class Controller:
    def __init__(self, t0, setpoint, surge, motorctrl):
        # Controlador do motor
        self._ctrl = motorctrl
        # Setpoint de rumo
        self._setpoint = setpoint
        # Ação de avanço
        self._surge = surge
        # Valor do tempo na última atualização
        self._t = t0
        # Valor do acumulador de integração na última atualização
        self._I = 0

    # Novo setpoint de rumo
    def newyawsetpoint(self, new):
        self._setpoint = new

    # Nova ação de avanço
    def newsurge(self, new):
        self._surge = new

    # Encerra o controlador
    def stop(self):
        print("Parando Controlador")
        self._ctrl.set_lr(0, 0)
        print("Controlador Parado")

    # Nova estimativa
    #   t é o momento da estimativa
    #   theta é a estimativa de rumo (em radianos)
    #   sigma é a covariância da estimativa de rumo
    #   omega é a estimativa da velocidade angular
    def newestimate(self, t, theta, sigma, omega):
        if sigma > 0.1:
            return

        dt = t - self._t

        # Parâmetros do controlador PID
        # ku = 575
        # tu = 0.6
        K = 345  # Ganho proporcional
        Ti = 0.3  # Tempo integral
        Td = 0.75  # Tempo derivativo

        # Calcula erro de rumo
        erro = self._setpoint - theta

        # Garante que o erro fique no intervalo [-pi, pi]
        erro = (erro + np.pi) % (2 * np.pi) - np.pi

        # Termo integral
        I_new = self._I + dt * erro

        # Termo derivativo: usando diretamente o -omega (derivada da saída)
        D = -omega

        # PID
        u = K * (erro + (1 / Ti) * I_new + Td * D)

        # Saturação do controle de rumo (ui)
        if u > 100:
            u = 100
            I_new = self._I  # Anti-windup: não atualiza I
        elif u < -100:
            u = -100
            I_new = self._I  # Anti-windup: não atualiza I
        a = 0
        # Calcula os comandos para as rodas
        l = (a + u) / 2
        r = (a - u) / 2

        # Saturação dos comandos dos motores
        l = max(min(l, 100), -100)
        r = max(min(r, 100), -100)

        # Atualiza o integrador se não houve saturação
        if -100 < u < 100:
            self._I = I_new

        # Aplica o comando nos motores
        self._ctrl.set_lr(l, r)

        # Atualiza o tempo da última iteração
        self._t = t

        # Complete com seu código
        pass
