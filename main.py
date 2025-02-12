# Програма за демонстрация на контролирано зареждане
# и разреждане на литиева батерия от един елемент
#
# Има Bluetooth свързаност като профил за батерия
#
# Хардуерен модул "Raspberry Pi Pico W with RP2040"
# Интерпретатор "MicroPython v1.24.1 on 2024-11-29"
#
# Декември 2024
#
# Напътствия от OpenAI
# https://chatgpt.com/share/676d1788-d068-8001-834b-b358a6f9f478
#

from machine import Pin, ADC, Timer, PWM
from time import sleep
from ble_advertising import advertising_payload
import bluetooth

# Конфигурация на ADC пинове
voltage_pin = ADC(27)  # GPIO 26 за напрежение на батерията
voltage_drop_pin = ADC(26)  # GPIO 27 за спад на напрежение върху шунтовия резистор
sensor_temp = machine.ADC(4)  # вътрешен сензор за температура

# Коефиценти за ADC
conversion_factor = 3.3 / (65535)   # коефицент за изчисляване на температура от вътрешния сензор

# LED върху платката
led = Pin("LED", Pin.OUT)

# Таймери
tmr_led = Timer()

is_connected = False;      # Глобален флаг, при изградена връзка става на True

# PWM изходи
pwm_charge_pin = PWM(Pin(3))
pwm_discharge_pin = PWM(Pin(4))

# Честота на PWM
pwm_charge_pin.freq(1000)
pwm_discharge_pin.freq(1000)

# Ключ за смяна на режимите (pull-up резистор)
mode_switch = Pin(2, Pin.IN, Pin.PULL_UP)  

# Настройка на PWM duty cycle (0-100%)
def set_pwm_duty(pwm, duty):
    pwm.duty_u16(int(duty * 65535 / 100))
    
# Изключване на PWM, изходът остава в ниско ниво
def stop_pwm(pwm):
    pwm.duty_u16(0)

# Настройка на инвертиран PWM duty cycle (0-100%)
def set_pwm_duty_inverted(pwm, duty):
    inverted_duty = 100 - duty  # Инвертираме duty cycle
    pwm.duty_u16(int(inverted_duty * 65535 / 100))
    
# Изключване на PWM, изходът остава във високо ниво
def stop_pwm_inverted(pwm):
    pwm.duty_u16(65535)    

# Функция за преобразуване на ADC стойности в напрежение (коригирай делителя)
def read_voltage():
    raw = voltage_pin.read_u16()
    raw = raw - 416   # корекция на измерванията за точност
    voltage = (raw / 65535.0) * 3.3 * 2  # Предполагаем делител на напрежение 1:2
    return voltage

# Функция за преобразуване на ADC стойности в ток (коригирай шунтовия резистор)
def read_current():
    raw_drop = voltage_drop_pin.read_u16()
    raw_drop = raw_drop - 416            # калибровъчен коефицент за АЦП
    raw_battery = voltage_pin.read_u16()
    raw_battery = raw_battery - 416      # калибровъчен коефицент за АЦП
    raw = raw_drop - raw_battery
    current = ((raw / 65535.0) * 3.3 * 2)/ 0.94  # Шунтов резистор 2 x 0.47 Ом
    return current

# Функция за контрол на LED върху платката
def led_blink(timer):
    global led
    global is_connected
    if is_connected :
       led.value(1)
    else :
       led.toggle()

# измерване на температура
def read_temperature():
    global conversion_factor
    reading = sensor_temp.read_u16() * conversion_factor
    temperature = 27 - (reading - 0.706)/0.001721
    return temperature

# Bluetooth клас
class BLEServer:
    def __init__(self):
        self.ble = bluetooth.BLE()
        self.ble.active(True)
        self.ble.config(gap_name="PicoBatt")
        self.ble.irq(self.ble_irq)
        self.connections = set()

        # Съкратени UUID
        self.battery_service_uuid = bluetooth.UUID(0x180F)  # Battery Service
        self.battery_char_uuid = bluetooth.UUID(0x2A19)    # Battery Level Characteristic
        

        # Дефиниране на услуги и характеристики
        self.services = (
            (self.battery_service_uuid, (
                (self.battery_char_uuid, bluetooth.FLAG_READ | bluetooth.FLAG_NOTIFY),
            )),
        )
        
        self.handles = self.ble.gatts_register_services(self.services)
        print("Handles:", self.handles)
        # Handles за характеристиките
        self.battery_handle = self.handles[0][0]  # Battery Level
        # Стартираме реклама
        self.start_advertising()

    def ble_irq(self, event, data):
        global is_connected
        if event == 1:         # Свързване
            conn_handle, _, _ = data
            self.connections.add(conn_handle)
            is_connected = True       # Показва наличие на връзка
        elif event == 2:       # Затваряне на връзка
            conn_handle, _, _ = data
            self.connections.remove(conn_handle)
            self.start_advertising()
            is_connected = False 
            
    def start_advertising(self):
        payload = advertising_payload(name="PicoBatt", services=[self.battery_service_uuid]) #, self.data_service_uuid])
        self.ble.gap_advertise(100, payload)

    def send_data(self, data):
        # Изпращане на Notification
        for conn_handle in self.connections:
            self.ble.gatts_notify(conn_handle, self.battery_handle, data.encode('utf-8'))

# Функция за изчисляване на нивото на батерията
def calculate_battery_level(voltage):
    min_voltage = 3.3
    max_voltage = 4.2
    level = ((voltage - min_voltage) / (max_voltage - min_voltage)) * 100
    level = max(0, min(100, level))
    return int(level)

def reset_gpio_pins():
    led.value(0)
    pins = [2, 3, 4, 26, 27]
    for pin in pins:
        gpio = machine.Pin(pin, machine.Pin.IN)
        gpio.value(0)

# Инициализация на Bluetooth сървъра
ble_server = BLEServer()

# Инициализация на таймер за мигане на LED на платката докато очаква връзка
tmr_led.init(freq=2.5, mode=Timer.PERIODIC, callback=led_blink)



# Основен цикъл със обработка на изключения
try:
    while True:
        # измерване на аналогови величини
        voltage = read_voltage()      
        current = read_current() 
        temperature = read_temperature()
    
        # Управление на схема за зареждане или разреждане на батерията в
        # зависимост от превключвателя на платката
        if mode_switch.value() == 0:  # Ключът е включен и батерията се разрежда през товарните резистори
            stop_pwm(pwm_charge_pin)                          # Изключване на ключът за зареждане
            if voltage > 3.3:                                 # Когато напрежението е над минималното 3.3V се изпълнява разреждане
               set_pwm_duty_inverted(pwm_discharge_pin, 100)  # Включване на ключът за разреждане, 100 е максимален ток в проценти
            else:
               set_pwm_duty_inverted(pwm_discharge_pin, 0)    # Изключване на ключът за разреждане, 0 е изключен
        else:
            stop_pwm_inverted(pwm_discharge_pin)              # Изключване на ключът за разреждане
            if voltage > 4.2:                                 # Когато напрежението е достигнало максималното се изключва зареждането
               set_pwm_duty(pwm_charge_pin, 0)              # Изключване на ключът за зареждане 
            else:
               set_pwm_duty(pwm_charge_pin, 100)              # Включване на ключът за зареждане 100 е максимален ток в проценти
                
        # Подготовка на данни за клиент
        battery_level = calculate_battery_level(voltage)  # Изчисляване на нивото на батерията
        print(f"V = {voltage:.2f} V, I = {current:.2f}A, Level = {battery_level:.2d} %, T = {temperature:.1f} ॰C")
    
        # Запис на стойността в характеристиката
        ble_server.ble.gatts_write(ble_server.battery_handle, bytes([battery_level]))
    
        # Изпращане на данни през Bluetooth
        data = f"V: {voltage:.2f}V, I: {current:.2f}A, L: {battery_level:.2f}%, T: {temperature:.2f}C"
        ble_server.send_data(data)
    

        # прериод за нотификации на свързания клиент
        sleep(0.5)    
# Установяване на изходите към ключовете на платка при спиране на програмата
# с REPL интерфейс или Ctrl+C комбинация
except KeyboardInterrupt:
    reset_gpio_pins()
    # Връщане в REPL
