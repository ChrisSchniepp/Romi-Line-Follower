from pyb import Timer, Pin

# SLP = Motor Enable = EN_pin
# DIR = Motor Direction = PH_pin
# PWM = PWM
# https://www.pololu.com/product/3543


class Motor:
    """
    @brief Class to enable and use L6206 motor

    @detail With this class you can set the duty of the motor from -100 to 100

    """

    def __init__ (self, PWM_tim, PWM_pin, DIR_pin, SLP_pin):
        """
        @brief Init for Romi motor

        @detail Takes a PWM configured Timer and a EN and PH pins which are pins 5 and 6 on for Romi Motor Driver

        @param PWM_tim = Timer configured for PWM, ex tim_a = Timer(n, freq = 20_000)

        @param PWM_pin = controlls the PWM for motor speed
        
        @param SLP_pin = pin for EN of motor

        @param DIR_pin = pin for PH of motor

        """   
        # Pin declaration
        self.DIR = Pin(DIR_pin, mode=Pin.OUT_PP)
        self.SLP = Pin(SLP_pin, mode=Pin.OUT_PP)
        self.SLP.high()

        # Timer Channnel
        self.CH1 = PWM_tim.channel(1, Timer.PWM, pin=PWM_pin)
        self.CH1.pulse_width_percent(0)

    def set_duty (self, duty):
        """
        @brief Main function for class, turns on motor to set duty cycle

        @detail Takes inputted duty cycle and turns on motor accordingly, can take negative values for reverse

        @param duty = duty cycle for motor

        """    
        if duty > 0 and duty <= 100:
            self.DIR.low()
            self.CH1.pulse_width_percent(duty)
        elif duty == 0:
            self.CH1.pulse_width_percent(0)
        elif duty >= -100 and duty < 0:
            self.DIR.high()
            self.CH1.pulse_width_percent(-duty)
        else:
            print("Error, -100 < duty cycle < 100")
    
    def enable (self):
        """
        @brief Enables motor, must do to run motor

        """
        self.SLP.high()

    def disable (self):
        """
        @brief disables motor

        """    
        self.SLP.low()
