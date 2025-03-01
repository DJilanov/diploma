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

import machine, onewire, ds18x20

# the device is on GPIO12
ds_pin = machine.Pin(17)

# create the onewire object
ds_sensor = ds18x20.DS18X20(onewire.OneWire(ds_pin))

# scan for devices on the bus
roms = ds_sensor.scan()
print('found devices:', roms)
ds_sensor.convert_temp()
#for rom in roms:
#tempC = ds_sensor.read_temp(1)
#print('temperature (ºC):', "{:.2f}".format(tempC))


# Конфигурация на ADC пинове
voltage_pin = ADC(27)  # GPIO 26 за напрежение на батерията
voltage_drop_pin = ADC(26)  # GPIO 27 за спад на напрежение върху шунтовия резистор
amper_pin = ADC(28) # измерване на напрежението от сензора на Lem
sensor_temp = machine.ADC(4)  # вътрешен сензор за температура

# Коефиценти за ADC
min_voltage = 3.3
max_voltage = 4.2
min_temperature = 0
max_temperature = 40
current_cycle = 0
max_cycles = 0
save_on_cycle = 0
active_logging = 0
conversion_factor = min_voltage / (65535)   # коефицент за изчисляване на температура от вътрешния сензор
reference_voltage = 3.3 # референтно напрежение на ADC

# LED върху платката
led = Pin("LED", Pin.OUT)
led_charge = Pin(5, Pin.OUT)
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
    # raw = raw - 416   # корекция на измерванията за точност
    voltage = (raw / 65535.0) * reference_voltage * 4  # Предполагаем делител на напрежение 1:4
    return voltage

# Функция за преобразуване на ADC стойности в ток (коригирай шунтовия резистор)
def read_current():
    raw = amper_pin.read_u16()
    raw = raw - 416   # корекция на измерванията за точност
    current = (raw / 65535.0) * reference_voltage * 10  # Предполагаем делител на напрежение 1:2
    
    
#     raw_drop = voltage_drop_pin.read_u16()
#     raw_drop = raw_drop - 416            # калибровъчен коефицент за АЦП
#     raw_battery = voltage_pin.read_u16()
#     raw_battery = raw_battery - 416      # калибровъчен коефицент за АЦП
#     raw = raw_drop - raw_battery
#     current = ((raw / 65535.0) * min_voltage * 2)/ 0.94  # Шунтов резистор 2 x 0.47 Ом
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
    global roms
    ds_sensor.convert_temp()
    for rom in roms:
        tempC = ds_sensor.read_temp(rom)
    #print('temperature (ºC):', "{:.2f}".format(tempC))
    #global conversion_factor
    #reading = sensor_temp.read_u16() * conversion_factor
    #temperature = 27 - (reading - 0.706)/0.001721
    return tempC

def parse_command_to_array(data: bytes) -> list:
    """
    Parse the binary command data into an array where each element is a single byte.
    Each element will be an integer (0-255).
    """
    return list(data)

def merge_two_bytes(high_byte, low_byte):
    return (high_byte << 8) | low_byte  # Shift high byte 8 bits left and OR with low byte

def create_data_file():
    """
    Create a file named 'data.txt' in the Pico's filesystem so data can be written.
    """
    try:
        with open("log.txt", "w") as f:
            f.write("")
        print("File 'data.txt' created successfully.")
    except Exception as e:
        print("Error creating file:", e)

def write_to_data_file(new_line):
    """
    Appends a new line to 'log.txt'. 
    If the file exceeds 500 lines, the oldest line is removed.
    """
    try:
        file_path = "log.txt"

        # Read existing lines
        try:
            with open(file_path, "r") as f:
                lines = f.readlines()
        except OSError:
            lines = []  # If file doesn't exist or can't be read, start fresh
            print("Еррор")

        # Append the new line
        lines.append(new_line + "\n")

        # Keep only the last 500 lines
        if len(lines) > 500:
            lines = lines[-500:]

        # Write back to file
        with open(file_path, "а") as f:
            f.write(new_line + "\n")

        # print("New line added to 'log.txt'.")

    except Exception as e:
        print("Error writing to file:", e)

# Bluetooth клас
class BLEServer:
    def __init__(self):
        self.ble = bluetooth.BLE()
        self.ble.active(True)
        self.ble.config(gap_name="PicoBatt")
        self.ble.irq(self.ble_irq)
        self.connections = set()

        # Standard Battery Service UUID and Characteristic UUID
        self.battery_service_uuid = bluetooth.UUID(0x180F)  # Battery Service
        self.battery_char_uuid = bluetooth.UUID(0x2A19)       # Battery Level Characteristic
        self.char_1000_uuid = bluetooth.UUID(1000)  # Custom Characteristic (1000)

        # Define services and characteristics:
        # We add the control characteristic with FLAG_WRITE.
        self.services = (
            (self.battery_service_uuid, (
                (self.battery_char_uuid, bluetooth.FLAG_READ | bluetooth.FLAG_NOTIFY),
                (self.char_1000_uuid, bluetooth.FLAG_WRITE | bluetooth.FLAG_READ),
            )),
        )
        
        self.handles = self.ble.gatts_register_services(self.services)
        print("Handles:", self.handles)
        
        # Handles for the characteristics:
        # Assuming the first service tuple contains two handles:
        self.battery_handle = self.handles[0][0]  # Battery Level
        self.char_1000_handle = self.handles[0][1]  # Handle за характеристиката

        # Start advertising
        self.start_advertising()

    def ble_irq(self, event, data):
        global is_connected
        print("Event:", event)
        print("data:", data)
        if event == 1:  # Connection event
            conn_handle, _, _ = data
            self.connections.add(conn_handle)
            is_connected = True
        elif event == 2:  # Disconnection event
            conn_handle, _, _ = data
            self.connections.remove(conn_handle)
            self.start_advertising()
            is_connected = False
        elif event == 3:  # Write event
            conn_handle, attr_handle = data
            # Check if the write was to our control characteristic
            if attr_handle == self.char_1000_handle:
                value = self.ble.gatts_read(attr_handle)
                byte_array = parse_command_to_array(value)
                print("Control characteristic written:", byte_array)
                # Check if the array equals [0x01, 0x42, 0x01]
                if len(byte_array) > 1 and byte_array[0] == 0x01:
                    print("Got byte array")
                    if byte_array[1] == 0x41:  # Конфигуриране на минимални и максимални стойности
                        global min_voltage, max_voltage, min_temperature, max_temperature
                        min_voltage = int(merge_two_bytes(byte_array[2], byte_array[3]), 16) / 10
                        max_voltage = int(merge_two_bytes(byte_array[4], byte_array[5]), 16) / 10
                        min_temperature = int(merge_two_bytes(byte_array[10], byte_array[11]), 16) / 10
                        max_temperature = int(merge_two_bytes(byte_array[12], byte_array[13]), 16) / 10
                        print("Trigger command 41!")
                    elif byte_array[1] == 0x42:  # Кодът се използва за да се изпрати нужната информация за начало на цикъл от зареждане и разреждане.
                        global current_cycle, max_cycles   
                        current_cycle = 0
                        max_cycles = int(merge_two_bytes(byte_array[2], byte_array[3]), 16)
                        print("Trigger command 42!")
                    elif byte_array[1] == 0x43:  # Кодът се използва за да се прекрати текущия цикъл  
                        global current_cycle, max_cycles   
                        current_cycle = 0
                        max_cycles = 0
                        print("Trigger command 43!")
                    elif byte_array[1] == 0x44:  # Заявка за превключване на режим
                        if byte_array[2] == 0x01:
                            stop_pwm(pwm_charge_pin)
                            set_pwm_duty_inverted(pwm_discharge_pin, 100)
                            led_charge.value(1)
                        else:
                            stop_pwm(pwm_discharge_pin)
                            set_pwm_duty_inverted(pwm_charge_pin, 100)
                            led_charge.value(0)
                        
                        print("Trigger command 44!")
                    elif byte_array[1] == 0x45:  # Заявка за пазене на история на процесите
                        global save_on_cycle, active_logging   
                        active_logging = 1
                        save_on_cycle = int(merge_two_bytes(byte_array[2], byte_array[3]), 16)
                        print("Trigger command 45!")
                    elif byte_array[1] == 0x46:  # Заявка за прочитане и визуализация на записаните данни от процесите
                        try:
                            with open("log.txt", "r") as f:
                                file_content = f.read()
                            self.send_data(file_content)
                            print("File content sent:", file_content)
                        except Exception as e:
                            print("Error reading file:", e)

                        print("Trigger command 46!")
                    elif byte_array[1] == 0x47:  # Заявка за получаване на текущите стойности на платката
                        # Изпращане на данни през Bluetooth
                        data = f"min_voltage: {min_voltage:.2f}, max_voltage: {max_voltage:.2f}, min_temperature: {min_temperature:.2f}, max_temperature: {max_temperature:.2f}, current_cycle: {current_cycle:.2f}, max_cycles: {max_cycles:.2f}, save_on_cycle: {save_on_cycle:.2f}, active_logging: {active_logging:.2f}, max_cycles: {max_cycles:.2f}"
                        ble_server.send_data(data)
                        print("Trigger command 47!")
                    else:
                        print("Received unknown command:", byte_array)
                else:
                    print("Received unknown command:", byte_array)

    def start_advertising(self):
        payload = advertising_payload(name="PicoBatt", services=[self.battery_service_uuid])
        self.ble.gap_advertise(100, payload)

    def send_data(self, data):
        # Send notification to all connected clients.
        for conn_handle in self.connections:
            self.ble.gatts_notify(conn_handle, self.battery_handle, data.encode('utf-8'))

# Функция за изчисляване на нивото на батерията
def calculate_battery_level(voltage):
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
            if voltage > min_voltage:                                 # Когато напрежението е над минималното 3.3V се изпълнява разреждане
               set_pwm_duty_inverted(pwm_discharge_pin, 100)  # Включване на ключът за разреждане, 100 е максимален ток в проценти
            else:
               set_pwm_duty_inverted(pwm_discharge_pin, 0)    # Изключване на ключът за разреждане, 0 е изключен
        else:
            stop_pwm_inverted(pwm_discharge_pin)              # Изключване на ключът за разреждане
            if voltage > max_voltage:                                 # Когато напрежението е достигнало максималното се изключва зареждането
               set_pwm_duty(pwm_charge_pin, 0)              # Изключване на ключът за зареждане 
            else:
               set_pwm_duty(pwm_charge_pin, 100)              # Включване на ключът за зареждане 100 е максимален ток в проценти
                
        # Подготовка на данни за клиент
        battery_level = calculate_battery_level(voltage)  # Изчисляване на нивото на батерията
        if current_cycle < max_cycles:
            if battery_level == 100:
                stop_pwm(pwm_charge_pin)
                set_pwm_duty_inverted(pwm_discharge_pin, 100)
                led_charge.value(1)
            elif battery_level == 0:
                stop_pwm(pwm_discharge_pin)
                set_pwm_duty_inverted(pwm_charge_pin, 100)
                led_charge.value(0)
        # Лог да знаем как вървят данните
        # print(f"V = {voltage:.2f} V, I = {current:.2f} A, Level = {battery_level:.2d} %, T = {temperature:.1f} ॰C")
        # TODO: Add the limit of the logs on specific cycle
        write_to_data_file(f"V: {voltage:.2f}, I: {current:.2f}, Level: {battery_level:.2d}, T: {temperature:.1f}, max_cycles: {max_cycles:.2f}")
        # Запис на стойността в характеристиката
        ble_server.ble.gatts_write(ble_server.battery_handle, bytes([battery_level]))
    
        # Изпращане на данни през Bluetooth
        data = f"V: {voltage:.2f}, I: {current:.2f}, L: {battery_level:.2f}, T: {temperature:.2f}"
        ble_server.send_data(data)
    

        # прериод за нотификации на свързания клиент
        sleep(0.5)    
# Установяване на изходите към ключовете на платка при спиране на програмата
# с REPL интерфейс или Ctrl+C комбинация
except KeyboardInterrupt:
    reset_gpio_pins()
    # Връщане в REPL

