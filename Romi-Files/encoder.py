from pyb import Timer
import time
import math

class Encoder:
    """
    @brief Class to update and record encoder position and change
    """

    def __init__(self, ENC_tim, CH_A_PIN, CH_B_PIN):
        """
        @brief Initialize the Encoder with timer and encoder pins.
        """
        # Initial declarations
        self.AR = 65535
        self.tim = ENC_tim
        self.delta = 0
    
        self.tim.channel(1, pin=CH_A_PIN, mode=Timer.ENC_AB)
        self.tim.channel(2, pin=CH_B_PIN, mode=Timer.ENC_AB)
        
        # Get initial position
        self.position = 0
        self.prev_position = self.tim.counter()

        # Timing variables
        self.current_time = time.ticks_us()
        self.last_time = self.current_time

        # Speed and filtering parameters
        self.speed = 0
        self.alpha = 0.8  # Smoothing factor for low-pass filter; adjust as needed (0 < alpha < 1)

    def update(self): 
        """
        @brief The main method in Encoder to get position and speed.
        """
        # Calculate Delta
        self.current_position = self.tim.counter()
        self.delta = self.current_position - self.prev_position
        self.last_time = self.current_time
        self.current_time = time.ticks_us()

        # Set previous value for next update iteration
        self.prev_position = self.current_position

        # Check for over- or underflow
        if self.delta > (self.AR + 1) / 2:
            self.delta -= self.AR + 1
        elif self.delta < -(self.AR + 1) / 2:
            self.delta += self.AR + 1
        
        # Update position
        self.position += self.delta
        
        # Calculate raw speed
        radians_per_tick = (2 * math.pi) / (12 * 120) 
        time_interval = time.ticks_diff(self.current_time, self.last_time) * 1e-6  # Convert microseconds to seconds
        raw_speed = (self.delta * radians_per_tick) / time_interval
        
        # Apply low-pass filter to smooth the speed
        self.speed = self.alpha * self.speed + (1 - self.alpha) * raw_speed

    def get_position(self):
        """
        @brief Returns position of motor in ticks.
        """        
        return self.position * -1

    def get_delta(self):
        """
        @brief Returns position change of motor in ticks.
        """        
        return self.delta

    def zero(self):
        """
        @brief Zero the encoder position.
        """        
        self.position = 0
        self.prev_position = self.tim.counter()
    
    def get_count(self):
        """
        @brief Returns ticks of encoder, used for testing.
        """        
        while True:
            print(self.tim.counter())
            time.sleep(1)

    def get_speed(self):
        """
        @brief Returns the current filtered speed of the motor in rad/s.
        """
        self.update()
        return -self.speed
