import machine
from machine import SoftI2C, PWM, Pin, ADC, DAC, Timer
from Makeitnow_class import WiFiConnection, BLE, TimeManager, SerialHandler, NeoBitmap
from Makeitnow_function import mPrint, map_value
from neopixel import NeoPixel
import math
import _thread
import utime
import sys
neoled23 = NeoPixel(Pin(23), 25)
neoled23.fill((0,0,0))
neoled23.write()
from Makeitnow_class import ADS1115

_EC_A7_81_EC_A7_84_EC_86_8D_EB_8F_84 = None
_EC_A7_81_EC_A0_84_EC_84_BC_EC_84_9C = None
_EC_B9_B4_EC_9A_B4_ED_84_B0 = None
_EC_A7_81_EC_A7_84_EC_8B_9C_EA_B0_84 = None
_EB_AA_A8_EB_93_9C = None

def control_motor(pwm_positive, pwm_negative, speed):
    if speed >= 0:
        pwm_positive.duty(speed)
        pwm_negative.duty(0)
    else:
        pwm_positive.duty(0)
        pwm_negative.duty(-speed)

p13 = PWM(Pin(13),freq=500)
p13.duty(0)
p16 = PWM(Pin(16),freq=500)
p16.duty(0)

p14 = PWM(Pin(14),freq=500)
p14.duty(0)
p17 = PWM(Pin(17),freq=500)
p17.duty(0)

p35 = Pin(35, Pin.IN)

p34 = Pin(34, Pin.IN)

p32 = ADC(Pin(32))
p32.atten(ADC.ATTN_11DB)

i2c = SoftI2C(scl=Pin(22), sda=Pin(21))

adsadc = ADS1115(i2c)

pBUZZER = PWM(Pin(27),freq=1)
pBUZZER.duty(0)

# 이 함수를 설명하세요...
def _08_EC_A3_BC_ED_96_89():
    global _EC_A7_81_EC_A7_84_EC_86_8D_EB_8F_84, _EC_A7_81_EC_A0_84_EC_84_BC_EC_84_9C, _EC_B9_B4_EC_9A_B4_ED_84_B0, _EC_A7_81_EC_A7_84_EC_8B_9C_EA_B0_84, _EB_AA_A8_EB_93_9C
    if (adsadc.read_channel(4-1,True)) <= 3700:
        _EC_A7_81_EC_A0_84_EC_84_BC_EC_84_9C = 'R'
        control_motor(p13,p16,180)
        control_motor(p17,p14,0)
    elif (adsadc.read_channel(1-1,True)) <= 3700:
        _EC_A7_81_EC_A0_84_EC_84_BC_EC_84_9C = 'L'
        control_motor(p13,p16,0)
        control_motor(p17,p14,180)
    elif (adsadc.read_channel(2-1,True)) <= 3800 or (adsadc.read_channel(3-1,True)) <= 3800:
        if _EC_A7_81_EC_A0_84_EC_84_BC_EC_84_9C == 'L':
            control_motor(p13,p16,200)
            control_motor(p17,p14,0)
            utime.sleep(0.2)
        elif _EC_A7_81_EC_A0_84_EC_84_BC_EC_84_9C == 'R':
            control_motor(p13,p16,0)
            control_motor(p17,p14,200)
            utime.sleep(0.2)
        _EC_A7_81_EC_A0_84_EC_84_BC_EC_84_9C = 'C'
        control_motor(p13,p16,(int(_EC_A7_81_EC_A7_84_EC_86_8D_EB_8F_84)))
        control_motor(p17,p14,(int(_EC_A7_81_EC_A7_84_EC_86_8D_EB_8F_84)))

# 이 함수를 설명하세요...
def _EC_95_A1_EC_85_981():
    global _EC_A7_81_EC_A7_84_EC_86_8D_EB_8F_84, _EC_A7_81_EC_A0_84_EC_84_BC_EC_84_9C, _EC_B9_B4_EC_9A_B4_ED_84_B0, _EC_A7_81_EC_A7_84_EC_8B_9C_EA_B0_84, _EB_AA_A8_EB_93_9C
    pBUZZER.freq(523)
    pBUZZER.duty(512)
    utime.sleep(0.5)
    pBUZZER.duty(0)

# 이 함수를 설명하세요...
def _EC_95_A1_EC_85_982():
    global _EC_A7_81_EC_A7_84_EC_86_8D_EB_8F_84, _EC_A7_81_EC_A0_84_EC_84_BC_EC_84_9C, _EC_B9_B4_EC_9A_B4_ED_84_B0, _EC_A7_81_EC_A7_84_EC_8B_9C_EA_B0_84, _EB_AA_A8_EB_93_9C
    control_motor(p13,p16,0)
    control_motor(p17,p14,0)
    utime.sleep(1)

# 이 함수를 설명하세요...
def _EC_95_A1_EC_85_983():
    global _EC_A7_81_EC_A7_84_EC_86_8D_EB_8F_84, _EC_A7_81_EC_A0_84_EC_84_BC_EC_84_9C, _EC_B9_B4_EC_9A_B4_ED_84_B0, _EC_A7_81_EC_A7_84_EC_8B_9C_EA_B0_84, _EB_AA_A8_EB_93_9C
    control_motor(p13,p16,200)
    control_motor(p17,p14,(-200))
    utime.sleep(2)
    control_motor(p13,p16,0)
    control_motor(p17,p14,0)


_EC_A7_81_EC_A7_84_EC_86_8D_EB_8F_84 = '150'
_EC_B9_B4_EC_9A_B4_ED_84_B0 = '1'
_EC_A7_81_EC_A7_84_EC_8B_9C_EA_B0_84 = '0.4'
while True:
    _EB_AA_A8_EB_93_9C = 'Stop'
    def IRQ_BUTTON35(pin):
        global _EC_A7_81_EC_A7_84_EC_86_8D_EB_8F_84, _EC_A7_81_EC_A0_84_EC_84_BC_EC_84_9C, _EC_B9_B4_EC_9A_B4_ED_84_B0, _EC_A7_81_EC_A7_84_EC_8B_9C_EA_B0_84, _EB_AA_A8_EB_93_9C
        pin.irq(handler=None)
        _EC_B9_B4_EC_9A_B4_ED_84_B0 = '1'
        utime.sleep(1)
        control_motor(p13,p16,(int(_EC_A7_81_EC_A7_84_EC_86_8D_EB_8F_84)))
        control_motor(p17,p14,(int(_EC_A7_81_EC_A7_84_EC_86_8D_EB_8F_84)))
        utime.sleep(2)
        _EB_AA_A8_EB_93_9C = 'Go'
        pin.irq(trigger=Pin.IRQ_RISING, handler=IRQ_BUTTON35)
    p35.irq(trigger=Pin.IRQ_RISING, handler=IRQ_BUTTON35)
    def IRQ_BUTTON34(pin):
        global _EC_A7_81_EC_A7_84_EC_86_8D_EB_8F_84, _EC_A7_81_EC_A0_84_EC_84_BC_EC_84_9C, _EC_B9_B4_EC_9A_B4_ED_84_B0, _EC_A7_81_EC_A7_84_EC_8B_9C_EA_B0_84, _EB_AA_A8_EB_93_9C
        pin.irq(handler=None)
        _EB_AA_A8_EB_93_9C = 'Stop'
        pin.irq(trigger=Pin.IRQ_RISING, handler=IRQ_BUTTON34)
    p34.irq(trigger=Pin.IRQ_RISING, handler=IRQ_BUTTON34)
    while True:
        if (p32.read()) > 2000:
            control_motor(p13,p16,0)
            control_motor(p17,p14,0)
            utime.sleep(3)
        if (int(_EC_B9_B4_EC_9A_B4_ED_84_B0)) <= 5 and (adsadc.read_channel(1-1,True)) < 4000 and (adsadc.read_channel(4-1,True)) < 4000:
            if (int(_EC_B9_B4_EC_9A_B4_ED_84_B0)) == 1:
                control_motor(p13,p16,0)
                control_motor(p17,p14,0)
                _EC_95_A1_EC_85_981()
                _EC_B9_B4_EC_9A_B4_ED_84_B0 = '2'
                control_motor(p13,p16,(int(_EC_A7_81_EC_A7_84_EC_86_8D_EB_8F_84)))
                control_motor(p17,p14,(int(_EC_A7_81_EC_A7_84_EC_86_8D_EB_8F_84)))
                utime.sleep(float(_EC_A7_81_EC_A7_84_EC_8B_9C_EA_B0_84))
            elif (int(_EC_B9_B4_EC_9A_B4_ED_84_B0)) == 2:
                control_motor(p13,p16,0)
                control_motor(p17,p14,0)
                _EC_95_A1_EC_85_982()
                _EC_B9_B4_EC_9A_B4_ED_84_B0 = '3'
                control_motor(p13,p16,(int(_EC_A7_81_EC_A7_84_EC_86_8D_EB_8F_84)))
                control_motor(p17,p14,(int(_EC_A7_81_EC_A7_84_EC_86_8D_EB_8F_84)))
                utime.sleep(float(_EC_A7_81_EC_A7_84_EC_8B_9C_EA_B0_84))
            elif (int(_EC_B9_B4_EC_9A_B4_ED_84_B0)) == 3:
                control_motor(p13,p16,0)
                control_motor(p17,p14,0)
                _EC_95_A1_EC_85_983()
                _EC_B9_B4_EC_9A_B4_ED_84_B0 = '4'
                control_motor(p13,p16,(int(_EC_A7_81_EC_A7_84_EC_86_8D_EB_8F_84)))
                control_motor(p17,p14,(int(_EC_A7_81_EC_A7_84_EC_86_8D_EB_8F_84)))
                utime.sleep(float(_EC_A7_81_EC_A7_84_EC_8B_9C_EA_B0_84))
            elif (int(_EC_B9_B4_EC_9A_B4_ED_84_B0)) == 4:
                _EC_B9_B4_EC_9A_B4_ED_84_B0 = '5'
                control_motor(p13,p16,(int(_EC_A7_81_EC_A7_84_EC_86_8D_EB_8F_84)))
                control_motor(p17,p14,(int(_EC_A7_81_EC_A7_84_EC_86_8D_EB_8F_84)))
                utime.sleep(float(_EC_A7_81_EC_A7_84_EC_8B_9C_EA_B0_84))
            elif (int(_EC_B9_B4_EC_9A_B4_ED_84_B0)) == 5:
                _EC_B9_B4_EC_9A_B4_ED_84_B0 = '6'
                _EB_AA_A8_EB_93_9C = 'Stop'
                pBUZZER.freq(1047)
                pBUZZER.duty(512)
                utime.sleep(0.5)
                pBUZZER.duty(0)
        if _EB_AA_A8_EB_93_9C == 'Go':
            _08_EC_A3_BC_ED_96_89()
        else:
            _EC_B9_B4_EC_9A_B4_ED_84_B0 = '0'
            control_motor(p13,p16,0)
            control_motor(p17,p14,0)
