from machine import Pin, PWM, ADC
import utime

# Classe para controle de servomotor
class Servo:
    # Iniciação
    def __init__(self, pino):
        self.FREQ = 50
        self.pwm = PWM(Pin(pino))
        self.pwm.freq(self.FREQ)
        self.ultpos = 0
        self.duty = 0
        self.pwm.duty_u16(0)
        self.min = 1
        self.max = 2

    # Envia n pulsos com tempo t em ms
    def enviaPulsos(self, t, nPulsos=20):
        self.duty = int(t * 65.535 * self.FREQ)
        self.pwm.duty_u16(self.duty)
        utime.sleep(nPulsos / self.FREQ)
        self.pwm.duty_u16(0)
        #print ("-> "+str(t)+" "+str(self.duty))

    # Informa/Muda posição
    # angulo deve estar entre 0 e 180 graus
    def pos(self, ang=None):
        if ang is None:
            return self.ultpos
        elif 0 <= ang <= 180:
            self.ultpos = ang
            t = self.min + (180 - ang) * (self.max - self.min) / 180
            self.enviaPulsos(t)

    # Informa/muda tempo (em ms) para colocar na posição 180 graus
    def tempoFim(self, val=None):
        if val is None:
            return self.min
        else:
            self.min = val

    # Informa/muda tempo (em ms) para colocar na posição 0 graus
    def tempoInicio(self, val=None):
        if val is None:
            return self.max
        else:
            self.max = val


xAxis = ADC(Pin(27))
servo = Servo(0)

xStatus = "middle"
pressed_time = 0
increment = 10

while True:
    xValue = xAxis.read_u16()

    if xValue <= 600:
        xStatus = "left"
        if xStatus != "left":
            pressed_time = utime.ticks_ms()
        if utime.ticks_diff(utime.ticks_ms(), pressed_time) >= 1000:
            if servo.pos() > 0:
                servo.pos(servo.pos() - increment)  # Move gradualmente para a posição "esquerda" desejada

    elif xValue >= 60000:
        xStatus = "right"
        if xStatus != "right":
            pressed_time = utime.ticks_ms()
        if utime.ticks_diff(utime.ticks_ms(), pressed_time) >= 1000:
            if servo.pos() < 180:
                servo.pos(servo.pos() + increment)  # Move gradualmente para a posição "direita" desejada

    else:
        xStatus = "middle"

    print("Ângulo: " + str(servo.pos()))
    utime.sleep(0.1)
