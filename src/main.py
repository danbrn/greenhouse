import pycom
from machine import PWM, Pin, RTC
from utime import sleep, time, localtime
from dth import DTH
from _thread import start_new_thread

#from greenhouse import Limits, Buzzer, Button, LocalAlarm, Greenhouse


class AlarmState:
    OK = 0
    WARN_LOW = 1
    WARN_HIGH = 2
    ALARM_LOW = 3
    ALARM_HIGH = 4
    WARN = 5
    ALARM = 6


class Limits:
    alarm_low = None
    warn_low = None
    alarm_high = None
    warn_high = None

    def __init__(self, warn_low=None, alarm_low=None, warn_high=None, alarm_high=None):
        if warn_low:
            if alarm_low and warn_low <= alarm_low:
                raise Exception('warn_low >= alarm_low')
            if warn_high and warn_low >= warn_high:
                raise Exception('warn_low >= warn_high')
            if alarm_high and warn_low >= alarm_high:
                raise Exception('alarm_low >= alarm_high')
        if alarm_low:
            if warn_high and alarm_low >= warn_high:
                raise Exception('alarm_low >= warn_high')
            if alarm_high and alarm_low >= alarm_high:
                raise Exception('alarm_low >= alarm_high')
        if warn_high:
            if alarm_high and warn_high >= alarm_high:
                raise Exception('warn_high >= alarm_high')

        self.warn_low = warn_low
        self.alarm_low = alarm_low
        self.warn_high = warn_high
        self.alarm_high = alarm_high

    def check(self, value):
        if self.alarm_low and value <= self.alarm_low:
            return AlarmState.ALARM_LOW
        if self.warn_low and value <= self.warn_low:
            return AlarmState.WARN_LOW
        if self.alarm_high and value >= self.alarm_high:
            return AlarmState.ALARM_HIGH
        if self.warn_high and value >= self.warn_high:
            return AlarmState.WARN_HIGH
        return AlarmState.OK


class DummyLimits:
    def check(self, value):
        return AlarmState.OK


class Buzzer:
    def __init__(self, buzzer_pin, channel=0, beep_length=0.2, beep_delay=4.5):
        self.beep = False
        self.beep_length = beep_length
        self.beep_delay = beep_delay
        self.running = False
        self.pwm = PWM(0, 1500)
        self.pwm_channel = self.pwm.channel(
            channel, pin=buzzer_pin, duty_cycle=0)

    def enable(self):
        self.beep = True

    def disable(self):
        self.beep = False

    def run(self):
        if self.running:
            return
        start_new_thread(lambda: self.run_loop(), ())

    def stop(self):
        self.running = False

    def run_loop(self):
        self.running = True
        while self.running:
            while self.running and self.beep:
                self.pwm_channel.duty_cycle(0.5)
                sleep(self.beep_length)
                self.pwm_channel.duty_cycle(0)
                sleep(self.beep_length / 2)
                self.pwm_channel.duty_cycle(0.5)
                sleep(self.beep_length)
                self.pwm_channel.duty_cycle(0)
                sleep(self.beep_delay)

    def is_running(self):
        return self.running

    def is_beeping(self):
        return self.beep


class Button:
    def __init__(self, button_pin):
        self.button = Pin(button_pin, mode=Pin.IN)
        self.running = False

    def run(self, callback):
        if self.running:
            self.stop()
            sleep(0.1)
        self.callback = callback
        start_new_thread(lambda: self.run_loop(), ())

    def stop(self):
        self.running = False

    def run_loop(self):
        self.running = True
        while self.running:
            if self.button() < 1:
                self.callback()
                self.running = False

    def is_running(self):
        return self.running


class LocalAlarm:
    def __init__(self, ok_color=0x000700, warn_color=0x1f1000, alarm_color=0x2f0000, grace_period=120, buzzer=None, button=None):
        self.ok_color = ok_color
        self.warn_color = warn_color
        self.alarm_color = alarm_color
        self.grace_period = grace_period
        self.buzzer = buzzer
        if self.buzzer is not None:
            self.buzzer.run()
        self.button = button
        self.read_alarm_state()
        if self.state is None:
            self.update_state(AlarmState.OK, time(), False)
        else:
            self.read_alarm_start_time()
            if self.start_time is None:
                self.update_state(start_time=time())
            self.read_alarm_acked()
        self.apply_state()

    def update_state(self, state=None, start_time=None, acked=None):
        if state is not None:
            self.state = state
            pycom.nvs_set('alarm_state', state)
        if start_time is not None:
            self.start_time = start_time
            pycom.nvs_set('alarm_start', start_time)
        if acked is not None:
            self.acked = acked
            if self.acked:
                pycom.nvs_set('alarm_acked', 1)
            else:
                pycom.nvs_set('alarm_acked', 0)

    def apply_state(self):
        if self.state == AlarmState.ALARM:
            pycom.rgbled(self.alarm_color)
            if self.button is not None and not self.button.is_running():
                if not self.acked and time() - self.start_time > self.grace_period:
                    self.enable_beep()
                    self.button.run(lambda: self.ack())
        elif self.state == AlarmState.WARN:
            pycom.rgbled(self.warn_color)
        else:
            pycom.rgbled(self.ok_color)

    def read_alarm_state(self):
        try:
            self.state = pycom.nvs_get('alarm_state')
        except ValueError:
            self.state = None

    def write_alarm_state(self):
        pycom.nvs_set('alarm_start', self.state)

    def read_alarm_start_time(self):
        try:
            self.start_time = pycom.nvs_get('alarm_start')
        except ValueError:
            self.start_time = None

    def write_alarm_start_time(self):
        pycom.nvs_set('alarm_start', self.start_time)

    def read_alarm_acked(self):
        try:
            acked = pycom.nvs_get('alarm_acked')
            self.acked = acked != 0
        except ValueError:
            self.acked = False

    def write_alarm_acked(self):
        pycom.nvs_set('alarm_acked', self.acked)

    def ok(self, tag=None):
        if self.state != AlarmState.OK:
            self.update_state(AlarmState.OK, time(), False)

    def warn(self, tag=None):
        if self.state != AlarmState.WARN:
            self.update_state(AlarmState.WARN, time())

    def alarm(self, tag=None):
        if self.state != AlarmState.ALARM:
            self.update_state(AlarmState.ALARM, time(), False)

    def ack(self):
        self.update_state(acked=True)
        self.disable_beep()

    def enable_beep(self):
        if self.buzzer is None or self.buzzer.is_beeping():
            return
        self.buzzer.enable()

    def disable_beep(self):
        if self.buzzer is None or not self.buzzer.is_beeping():
            return
        self.buzzer.disable()

    def enable_button(self):
        if self.button is None:
            return
        self.button.run(lambda: self.ack())

    def disable_button(self):
        if self.button is None:
            return
        self.button.stop()


class DummyAlarm:
    def update_state(self):
        pass

    def update_state(self, state=None, start_time=None, acked=None):
        pass

    def apply_state(self):
        pass

    def read_alarm_state(self):
        pass

    def write_alarm_state(self):
        pass

    def read_alarm_start_time(self):
        pass

    def write_alarm_start_time(self):
        pass

    def read_alarm_acked(self):
        pass

    def write_alarm_acked(self):
        pass

    def ok(self, tag=None):
        pass

    def warn(self, tag=None):
        pass

    def alarm(self, tag=None):
        pass

    def ack(self):
        pass

    def enable_beep(self):
        pass

    def disable_beep(self):
        pass

    def enable_button(self):
        pass

    def disable_button(self):
        pass


class Greenhouse:
    def __init__(self, sensor_pin, interval=30, reports_per_hour=6, dht_version=0, alarm_handler=None, temperature_limits=None, humidity_limits=None):
        pycom.heartbeat(False)
        pycom.rgbled(0)
        if reports_per_hour < 1 or reports_per_hour > 30:
            raise Exception(
                "reports_per_hour must be between 1 and 30")
        self.interval = interval
        self.reporting_interval = 60 * 60 / reports_per_hour
        self.dht = DTH(sensor_pin, dht_version)
        self.temps = []
        self.hums = []
        self.last_sync = -1

        if alarm_handler is None:
            self.alarm_handler = DummyAlarm()
        else:
            self.alarm_handler = alarm_handler

        if temperature_limits is None:
            self.temperature_limits = DummyLimits()
        else:
            self.temperature_limits = temperature_limits

        if humidity_limits is None:
            self.humidity_limits = DummyLimits()
        else:
            self.humidity_limits = humidity_limits

    def report(self):
        if (not self.temps) or (not self.hums):
            return
        pybytes.send_signal(11, sum(self.temps) / len(self.temps))
        self.temps = []
        pybytes.send_signal(21, sum(self.hums) / len(self.hums))
        self.hums = []
        self.last_sync = time()

    def measure(self):
        res = self.dht.read()
        if not res.is_valid():
            return None
        self.temps.append(res.temperature)
        self.hums.append(res.humidity)
        return res

    def run(self):
        while True:
            value = self.measure()
            if value is not None:
                if self.temperature_limits is not None:
                    temp_status = self.temperature_limits.check(
                        value.temperature)
                else:
                    temp_status = AlarmState.OK
                if self.humidity_limits is not None:
                    hum_status = self.humidity_limits.check(value.humidity)
                else:
                    hum_status = AlarmState.OK
                print("temperature: %dC, humidity: %d%%" %
                      (value.temperature, value.humidity))
                if temp_status == AlarmState.OK and hum_status == AlarmState.OK:
                    self.alarm_handler.ok()
                elif temp_status in [AlarmState.ALARM_LOW, AlarmState.ALARM_HIGH] or hum_status in [AlarmState.ALARM_LOW, AlarmState.ALARM_HIGH]:
                    self.alarm_handler.alarm()
                else:
                    self.alarm_handler.warn()
                self.alarm_handler.apply_state()

            if time() >= self.last_sync + self.reporting_interval:
                self.report()
                self.last_sync = time()
            sleep(self.interval)


print("RTC sync")
rtc = RTC()
rtc.ntp_sync('europe.pool.ntp.org')

pycom.heartbeat(False)

# Yellow light at or below 22 and at or above 27 C, red light and beeps at or below 18 and at or above 29.
temp_limit = Limits(22, 18, 27, 29)

# Relative humidity (%), warn -55 or 65+, alarm at -45 or 75+.
hum_limit = Limits(55, 45, 65, 75)

buzzer = Buzzer('P11', beep_length=0.2, beep_delay=2.5)

button = Button('P10')

alarm = LocalAlarm(buzzer=buzzer, button=button)

gh = Greenhouse('P23', 15, 20, alarm_handler=alarm,
                temperature_limits=temp_limit, humidity_limits=hum_limit)

print("Starting...")
gh.run()
