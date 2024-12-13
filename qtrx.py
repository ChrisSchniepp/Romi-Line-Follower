import pyb

class QTRX:
    def __init__(self, sensor_pins, ir_led_pin=None):
        """
        Initializes the QTRX sensor array.

        :param sensor_pins: List of GPIO pins connected to the sensors.
        :param ir_led_pin: (Optional) GPIO pin controlling the IR LEDs.
        """
        self.sensor_pins = [pyb.Pin(pin, pyb.Pin.OUT_PP) for pin in sensor_pins]
        self.ir_led_pin = pyb.Pin(ir_led_pin, pyb.Pin.OUT_PP) if ir_led_pin else None
    
    def turn_on_ir_leds(self):
        """Turns on the IR LEDs if applicable."""
        if self.ir_led_pin:
            self.ir_led_pin.high()
    
    def turn_off_ir_leds(self):
        """Turns off the IR LEDs if applicable."""
        if self.ir_led_pin:
            self.ir_led_pin.low()
    
    def read_sensor(self, sensor_pin):
        """
        Reads a single QTRX sensor.

        :param sensor_pin: Pin object corresponding to the sensor.
        :return: Decay time in microseconds.
        """
        # Turn on IR LEDs (optional)
        self.turn_on_ir_leds()
        
        # Set sensor pin to output and drive it high
        sensor_pin.init(pyb.Pin.OUT_PP)
        sensor_pin.high()
        
        # Wait for at least 10 µs for the capacitor to charge
        pyb.udelay(10)
        
        # Set sensor pin to input (high impedance)
        sensor_pin.init(pyb.Pin.IN)
        
        # Measure the time it takes for the pin to go low
        start_time = pyb.micros()
        while sensor_pin.value() == 1:
            if pyb.elapsed_micros(start_time) > 1000:  # Timeout
                break
        decay_time = pyb.elapsed_micros(start_time)
        
        # Turn off IR LEDs (optional)
        self.turn_off_ir_leds()
        
        return decay_time
    
    def read_all_sensors(self):
        """
        Reads all sensors in the array.

        :return: List of decay times for each sensor.
        """
        results = []
        for pin in self.sensor_pins:
            results.append(self.read_sensor(pin))
        return results
    
    def normalize_reading(self, reading, min_value=0, max_value=1000):
        """
        Normalizes a single reading to the range 0.0 to 1.0.

        :param reading: The raw sensor reading (decay time).
        :param min_value: The minimum expected reading (e.g., for white surface).
        :param max_value: The maximum expected reading (e.g., for black surface).
        :return: Normalized value between 0.0 and 1.0.
        """
        return max(0.0, min(1.0, (reading - min_value) / (max_value - min_value)))
    
    def threshold_reading(self, reading, threshold=0.9):
        """
        Applies thresholding to a single reading to classify it.

        :param reading: Normalized reading (e.g., 0.0 to 1.0).
        :param thresholds: List of threshold values to categorize the reading.
                           For example, [0.3, 0.7] for three categories: [0-0.3], [0.3-0.7], [0.7-1.0].
        :return: 1 for black, 0 for white
        """
        return 1 if reading >= threshold else 0
    
    def calculate_centroid(self, readings):
        """
        Calculates the centroid of the sensor data.

        :param readings: List of normalized sensor readings.
        :return: Centroid of the sensor values, or None if all readings are zero.
        """
        total_weight = 0
        weighted_sum = 0
        for i, reading in enumerate(readings):
            total_weight += reading
            weighted_sum += i * reading  # Index as weight position
        return weighted_sum / total_weight if total_weight > 0 else 0


    def is_line(self):
        raw_values = self.read_all_sensors()
        normalized_values = [self.normalize_reading(r) for r in raw_values]
        thresholded_values = [self.threshold_reading(n) for n in normalized_values]
        centroid = self.calculate_centroid(thresholded_values)
        return centroid > 1


# Example Usage
if __name__ == "__main__":
    # Define GPIO pins connected to sensors, Emitters not needed
    sensor_pins = ['B0', 'C1', 'H0', 'H1', 'B1', 'B2', 'B12', 'A10']
    # Create a QTRX sensor array instance
    qtrx = QTRX(sensor_pins)
    
    # Main loop to continuously read and process sensor values
    while True:
        raw_values = qtrx.read_all_sensors()
        normalized_values = [qtrx.normalize_reading(r, min_value=100, max_value=900) for r in raw_values]
        thresholded_values = [qtrx.threshold_reading(n) for n in normalized_values]
        centroid = qtrx.calculate_centroid(thresholded_values)

        #print("Raw readings:", raw_values)
        print("Normalized readings:", normalized_values)
        print("Thresholded readings:", thresholded_values)
        print("Centroid:", centroid)
        pyb.delay(100)  # Adjust the reading frequency (100 ms)
