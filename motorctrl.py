import gpiod
from gpiod.line import Direction, Value
from rpi_hardware_pwm import HardwarePWM
import contextlib


# Cria um controlador de motor.
#   Este gerador é decorado com contextmanager para garantir
#   que o estado do gpio e pwm seja resetado na saída de escopo
@contextlib.contextmanager
def cria_controle_motor():
    # Configuração inicial: output com nível inicial baixo
    print("Criando controle de motores...", end="")
    confoutlow = gpiod.LineSettings(direction=Direction.OUTPUT, output_value = Value.INACTIVE)
    with gpiod.Chip("/dev/gpiochip0") as chip0:
        # Cria acesso às linhas de GPIO 5 e 6
        with chip0.request_lines({5:confoutlow, 6:confoutlow}) as linhas:
            pwma = HardwarePWM(0, 20, 0)
            pwmb = HardwarePWM(1, 20, 0)
            motor = motorCtrl(linhas, 5, 6, pwma, pwmb)
            print("Feito.")
            try:
                yield motor
            finally:
                print("Encerrando controle de motores...", end="")
                pwma.stop()
                pwmb.stop()
    print("Feito.")

class motorCtrl():
    def __init__(self, linhas, canal_direita, canal_esquerda, pwma, pwmb):
        # Acesso a linhas gpio por gpiod
        self._linhas = linhas
        # Numero do canal de GPIO da fase "B" do motor da direita
        self._cdireita = canal_direita
        # Numero do canal de GPIO da fase "B" do motor da esquerda
        self._cesquerda = canal_esquerda
        # PWM conectado à fase "A" do motor da direita
        self._pwma = pwma
        # PWM conectado à fase "B" do motor da direita
        self._pwmb = pwmb
        self._pwma.start(0)
        self._pwmb.start(0)

    def set_lr(self, l, r):
        # Aplica comandos nos motores:
        #   l é o valor (de -100 a 100) para o motor esquerdo
        if(l>100):
            l=100
        if(l<-100):
            l=-100
        if(l>=0):
            self._pwmb.change_duty_cycle(l)
            self._linhas.set_value(self._cesquerda, Value.INACTIVE)
        else:
            self._pwmb.change_duty_cycle(100+l)
            self._linhas.set_value(self._cesquerda, Value.ACTIVE)

        if(r>100):
            r=100
        if(r<-100):
            r=-100
        if(r>=0):
            self._pwma.change_duty_cycle(r)
            self._linhas.set_value(self._cdireita, Value.INACTIVE)
        else:
            self._pwma.change_duty_cycle(100+r)
            self._linhas.set_value(self._cdireita, Value.ACTIVE)
        #   r é o valor (de -100 a 100) para o motor esquerdo
        print("motorCtrl.set_lr(l={}, r={})".format(l, r))




