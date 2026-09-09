# Thermal plant simulation for simulavr, so emulated heaters actually heat.
#
# Copyright (C) 2026  SoftFever <softfeverever@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# simulavr floats its analog pins at 0.55*Vcc, so an unattended thermistor
# reads a constant ~103.3 C and no temperature-dependent behaviour can be
# exercised. This module closes the loop: it watches the heater output pins,
# integrates a thermal model against them, and writes the resulting
# temperature back onto the sensor pins as a real analog voltage.
#
# No simulavr patch is needed. Pin.SetAnalogValue feeds straight into the ADC
# mux (libsim/atmega1284abase.cpp wires ADC0-7 to the PORTA pin objects), and
# an injected value survives unrelated writes to the same port because
# PortPin::CalcPinOverride only recomputes outState, leaving analogVal alone.
import math

try:
    import pysimulavr
    _Pin = pysimulavr.Pin
    _SimulationMember = pysimulavr.PySimulationMember
except ImportError:
    # The thermistor maths carries no emulator dependency, so it stays
    # importable - and self-testable - on a machine with no pysimulavr built.
    pysimulavr = None
    _Pin = _SimulationMember = object

# simulavr's AvrDevice::v_supply defaults to 5.0 V (libsim/avrdevice.cpp), and
# Klipper's AVR port selects AVcc as the ADC reference (ADMUX_DEFAULT in
# src/avr/adc.c), so the reading klippy sees is simply V_pin / 5.0.
VCC = 5.0
SIMULAVR_FREQ = 10**9

# Defaults matching config/generic-simulavr.cfg's "EPCOS 100K B57560G104F".
EPCOS_100K = ((25., 100000.), (150., 1641.9), (250., 226.15))
DEFAULT_PULLUP = 4700.

KELVIN_TO_CELSIUS = -273.15


class Thermistor:
    # Steinhart-Hart, kept deliberately identical to klippy/extras/thermistor.py.
    # It is duplicated rather than imported because that module is part of the
    # klippy package (it does "from . import adc_temperature") and avrsim runs
    # on the system interpreter, outside klippy's virtualenv.
    def __init__(self, pullup=DEFAULT_PULLUP, points=EPCOS_100K):
        self.pullup = pullup
        (t1, r1), (t2, r2), (t3, r3) = points
        inv_t1, inv_t2, inv_t3 = [1. / (t - KELVIN_TO_CELSIUS)
                                  for t in (t1, t2, t3)]
        ln_r1, ln_r2, ln_r3 = [math.log(r) for r in (r1, r2, r3)]
        ln3_r1, ln3_r2, ln3_r3 = ln_r1**3, ln_r2**3, ln_r3**3
        inv_t12, inv_t13 = inv_t1 - inv_t2, inv_t1 - inv_t3
        ln_r12, ln_r13 = ln_r1 - ln_r2, ln_r1 - ln_r3
        ln3_r12, ln3_r13 = ln3_r1 - ln3_r2, ln3_r1 - ln3_r3
        self.c3 = ((inv_t12 - inv_t13 * ln_r12 / ln_r13)
                   / (ln3_r12 - ln3_r13 * ln_r12 / ln_r13))
        self.c2 = (inv_t12 - self.c3 * ln3_r12) / ln_r12
        self.c1 = inv_t1 - self.c2 * ln_r1 - self.c3 * ln3_r1

    def calc_adc(self, temp):
        # Inverse of calc_temp: the fraction of full scale klippy should read.
        inv_t = 1. / (temp - KELVIN_TO_CELSIUS)
        y = (self.c1 - inv_t) / (2. * self.c3)
        x = math.sqrt((self.c2 / (3. * self.c3))**3 + y**2)
        ln_r = math.pow(x - y, 1. / 3.) - math.pow(x + y, 1. / 3.)
        r = math.exp(ln_r)
        return r / (self.pullup + r)

    def calc_temp(self, adc):
        adc = max(.00001, min(.99999, adc))
        r = self.pullup * adc / (1.0 - adc)
        ln_r = math.log(r)
        return 1.0 / (self.c1 + self.c2 * ln_r + self.c3 * ln_r**3) \
            + KELVIN_TO_CELSIUS


class HeaterPin(_Pin):
    # Joined to a heater output pin through a Net, exactly as avrsim.py wires
    # the serial pins. Klipper drives heaters with soft PWM on a 0.100 s cycle,
    # so the instantaneous pin level says almost nothing - what the model needs
    # is on-time integrated across the whole sampling window.
    def __init__(self, clock):
        _Pin.__init__(self)
        self.clock = clock
        self.is_on = False
        self.on_time = 0
        self.last_edge = self.window_start = clock.GetCurrentTime()

    def SetInState(self, pin):
        _Pin.SetInState(self, pin)
        is_on = pin.outState == pin.HIGH
        if is_on == self.is_on:
            return
        now = self.clock.GetCurrentTime()
        if self.is_on:
            self.on_time += now - self.last_edge
        self.last_edge = now
        self.is_on = is_on

    def take_duty(self):
        # Mean duty cycle since the previous call, then reset the window.
        now = self.clock.GetCurrentTime()
        if self.is_on:
            self.on_time += now - self.last_edge
            self.last_edge = now
        span = now - self.window_start
        on_time, self.on_time = self.on_time, 0
        self.window_start = now
        if span <= 0:
            return 1.0 if self.is_on else 0.0
        return min(1.0, float(on_time) / span)


class ThermalZone:
    # A single first-order thermal system:
    #
    #     dT/dt = rate * duty - (T - ambient) / tau,  tau = (ceiling - ambient) / rate
    #
    # "rate" is the initial heating rate at ambient under full power and
    # "ceiling" the steady-state temperature it would eventually reach, so both
    # knobs mean something physical. With duty 0 this is plain exponential
    # decay toward ambient; the same pole governs heating and cooling, which is
    # what a real heater does.
    #
    # The ceiling must sit above the configured max_temp. A target the plant
    # cannot reach stalls the approach, and verify_heater faults on that long
    # before the temperature becomes interesting.
    def __init__(self, device, clock, name, heater_pin, sensor_pin,
                 rate, ceiling, ambient, thermistor):
        if ceiling <= ambient:
            raise ValueError("%s: ceiling %.1f must exceed ambient %.1f"
                             % (name, ceiling, ambient))
        self.name = name
        self.rate = rate
        self.ambient = ambient
        self.tau = (ceiling - ambient) / rate
        self.temp = ambient
        self.thermistor = thermistor
        self.sensor = device.GetPin(sensor_pin)
        self.heater = HeaterPin(clock)
        # Held on the instance: dropping the Net would silently unwire the pin.
        self.net = pysimulavr.Net()
        self.net.Add(self.heater)
        self.net.Add(device.GetPin(heater_pin))
        # Publish ambient before klippy connects, so its very first ADC read is
        # already inside min_temp/max_temp rather than the 103 C floating value.
        self.publish()

    def step(self, dt):
        duty = self.heater.take_duty()
        self.temp += (self.rate * duty
                      - (self.temp - self.ambient) / self.tau) * dt
        if self.temp < self.ambient:
            self.temp = self.ambient
        self.publish()

    def publish(self):
        self.sensor.SetAnalogValue(self.thermistor.calc_adc(self.temp) * VCC)


class ThermalSim(_SimulationMember):
    # Integrates every zone on a fixed simulated-time tick. Simulated time is
    # the right base: the emulated MCU's own clock is what klippy's control
    # loop is timed against, so a heat-up spans the same number of control
    # updates no matter what pacing rate the simulation is running at.
    def __init__(self, zones, interval=0.05):
        _SimulationMember.__init__(self)
        self.clock = pysimulavr.SystemClock.Instance()
        self.zones = zones
        self.interval = interval
        self.delay = int(SIMULAVR_FREQ * interval)
        self.clock.Add(self)

    def DoStep(self, trueHwStep):
        for zone in self.zones:
            zone.step(self.interval)
        return self.delay


def parse_zone(spec):
    # "PB4:PA7:50:420[:pullup]" - heater pin, sensor pin, rate, ceiling.
    fields = spec.split(':')
    if len(fields) not in (4, 5):
        raise ValueError(
            "expected heater:sensor:rate:ceiling[:pullup], got %r" % (spec,))
    heater_pin, sensor_pin = [f.strip().upper().lstrip('P')
                              for f in fields[:2]]
    rate, ceiling = float(fields[2]), float(fields[3])
    pullup = float(fields[4]) if len(fields) == 5 else DEFAULT_PULLUP
    if rate <= 0.:
        raise ValueError("%s: rate must be positive" % (spec,))
    return heater_pin, sensor_pin, rate, ceiling, pullup


def attach(device, specs, ambient=25., interval=0.05):
    # Returns the ThermalSim, which the caller must keep alive for the run.
    clock = pysimulavr.SystemClock.Instance()
    zones = []
    for spec in specs:
        heater_pin, sensor_pin, rate, ceiling, pullup = parse_zone(spec)
        zones.append(ThermalZone(
            device, clock, spec, heater_pin, sensor_pin, rate, ceiling,
            ambient, Thermistor(pullup)))
    return ThermalSim(zones, interval)


def self_test():
    # A floating simulavr pin sits at 0.55*Vcc, and the harness has repeatedly
    # observed klippy report ~103.3 C from one. Reproducing that number from
    # these coefficients proves the whole voltage/temperature chain.
    therm = Thermistor()
    floating = therm.calc_temp(0.55)
    assert abs(floating - 103.30) < 0.01, floating
    for temp in (0., 25., 60., 200., 240., 300., 340.):
        assert abs(therm.calc_temp(therm.calc_adc(temp)) - temp) < 1e-6, temp
    print("avrsim_thermal self-test OK (floating pin = %.2f C)" % (floating,))


if __name__ == '__main__':
    self_test()
