class FakeGPIO:
    """Class to simulate rpi.gpio for testing purposes."""
    BCM = None
    OUT = None
    IN = None
    HIGH = None
    LOW = None

    def __init__(self):
        self.pins = {}

    def setwarnings(self, warnings):
        print(f"Setting warnings to {warnings}")

    def setmode(self, mode):
        print(f"Setting mode to {mode}")

    def setup(self, pin, mode):
        self.pins[pin] = {'mode': mode, 'state': FakeGPIO.LOW}
        print(f"Setting up pin {pin} as {'OUTPUT' if mode == FakeGPIO.OUT else 'INPUT'}")

    def output(self, pin, state):
        if pin in self.pins and self.pins[pin]['mode'] == FakeGPIO.OUT:
            self.pins[pin]['state'] = state
            print(f"Setting pin {pin} to {'HIGH' if state == FakeGPIO.HIGH else 'LOW'}")

    def input(self, pin):
        if pin in self.pins and self.pins[pin]['mode'] == FakeGPIO.IN:
            print(f"Reading pin {pin}: {'HIGH' if self.pins[pin]['state'] == FakeGPIO.HIGH else 'LOW'}")
            return self.pins[pin]['state']
        return None

    def cleanup(self):
        self.pins.clear()

    class PWM:
        def __init__(self, pin, frequency=100):
            self.pin = pin
            self.frequency = frequency
            self.duty_cycle = 0
        
        def start(self, duty_cycle):
            print(f"Starting PWM on pin {self.pin} with frequency "
                    "{self.frequency} duty cycle {duty_cycle}")
        
        def stop(self):
            print(f"Stopping PWM on pin {self.pin}")

class RotaryEncoder:
    """Class to simulate a rotary encoder from gpiozero."""
    def __init__(self, a, b):
        self.a = a
        self.b = b
        self.position = 0
