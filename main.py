import pyb  # Import pyb fully to access its functions directly with pyb.function_name()
from pyb import Timer, Pin, I2C
import task_share
import cotask
from motor import Motor
from encoder import Encoder
from imu import IMU
import math
import time
from qtrx import QTRX

BT_ser = pyb.UART(3, 115200)
pyb.repl_uart(BT_ser)

# Constants for robot geometry
WHEEL_RADIUS = 1.375/12  # Radius of wheels in feet
TRACK_WIDTH = 5.5/12    # Distance between wheels in feet

# PI controller gains (for initial tuning; adjust as needed)
Kp_inner = .3         # Proportional gain for inner loop
Ki_inner = .9        # Integral gain for inner loop
Kd_inner = .02       # Derivative gain for inner loop

Kp_outer_v = 1.5      # Proportional gain for outer longitudinal loop
Ki_outer_v = .2     # Integral gain for outer longitudinal loop
Kd_outer_v = 0.05   # Derivative gain for outer longitudinal loop

Kp_outer_yaw = 3    # Proportional gain for outer yaw loop
Ki_outer_yaw = 0.15    # Integral gain for outer yaw loop
Kd_outer_yaw = 0.1      # Derivative gain for outer yaw loop

# PD gains for the line following controller
Kp_line = 3  # Proportional gain for line following
Ki_line = 0.15 # Integral gain for line following
Kd_line = 0.1


# Shared variables
left_motor_speed = task_share.Share('f')              # Desired speed for left motor
right_motor_speed = task_share.Share('f')             # Desired speed for right motor

# Initialize motor speed variables with zero to prevent undefined reads in inner loop
left_motor_speed.put(0.0)  # Initialize with zero speed
right_motor_speed.put(0.0)  # Initialize with zero speed

# Define the pins for the left and right bumpers with internal pull-ups enabled
BMP0 = pyb.Pin(pyb.Pin.cpu.C6, mode=pyb.Pin.IN, pull=pyb.Pin.PULL_UP) # Right bumper pin 0 (input with pull-up)
BMP1 = pyb.Pin(pyb.Pin.cpu.C8, mode=pyb.Pin.IN, pull=pyb.Pin.PULL_UP)
BMP2 = pyb.Pin(pyb.Pin.cpu.C13, mode=pyb.Pin.IN, pull=pyb.Pin.PULL_UP)

BMP3 = pyb.Pin(pyb.Pin.cpu.C11, mode=pyb.Pin.IN, pull=pyb.Pin.PULL_UP) # Left bumper pin 3 (input with pull-up)
BMP4 = pyb.Pin(pyb.Pin.cpu.C10, mode=pyb.Pin.IN, pull=pyb.Pin.PULL_UP)
BMP5 = pyb.Pin(pyb.Pin.cpu.C12, mode=pyb.Pin.IN, pull=pyb.Pin.PULL_UP)

# Initialize line sensors
sensor_pins = ['B0', 'C1', 'H0', 'H1', 'B1', 'B2', 'B12', 'A10']
qtrx = QTRX(sensor_pins)
#qtrx.calibrate()

# Global variables
call_round_block = False
round_block_state = 0  # 0: not active, 1: reverse, 2: turn1, 3: straight1, 4: turn2, 5: straight2, 6: turn3, 7: end
last_trigger_time = 0
round_block_start_time = 0
crossed_line = False
    
def bumper_isr(line):
    global last_trigger_time, call_round_block
    current_time = time.ticks_ms()
    if not call_round_block: # Only call the round block if you aren't already doing it
       if time.ticks_diff(current_time, last_trigger_time) > 200:  # 200 ms debounce
            last_trigger_time = current_time
            call_round_block = True

# Decoupling Matrix Function
def calculate_motor_speeds(v, yaw_rate):
    omega_l = (1 / WHEEL_RADIUS) * v - (TRACK_WIDTH / (2 * WHEEL_RADIUS)) * yaw_rate
    omega_r = (1 / WHEEL_RADIUS) * v + (TRACK_WIDTH / (2 * WHEEL_RADIUS)) * yaw_rate
    return omega_l, omega_r

def calibrate_imu_before_running():
    """
    Initializes and calibrates the IMU before starting the main task loops.
    """
    i2c = I2C(1, I2C.CONTROLLER)
    imu = IMU(i2c)
    imu.calibrate_2()  # Wait for calibration to complete

# PID Controller Class
class PIDController:
    def __init__(self, Kp, Ki, Kd, out_max, integral_limit=100):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.out_max = out_max
        self.integral_limit = integral_limit  # Maximum value for the integral term
        self.prev_error = 0
        self.prev_time = None
    
    def set_gains(self,Kp,Ki,Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def update(self, setpoint, measurement, dt):
        error = setpoint - measurement
        
        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        
        derivative = 0
        if dt > 0:
            derivative = (error - self.prev_error)/dt
            
        self.prev_error = error

        output = self.Kp*error + self.Ki*self.integral + self.Kd*derivative
        
        if output > self.out_max:
            output = self.out_max
        if output <= -self.out_max:
            output = -self.out_max
        return output
    


# Instantiate outer loop controllers
outer_controller_v = PIDController(Kp_outer_v, Ki_outer_v, Kd_outer_v, 10, integral_limit=10)
outer_controller_yaw = PIDController(Kp_outer_yaw, Ki_outer_yaw, Kd_outer_v, 50, integral_limit=50)
encoder_left = Encoder(Timer(2, prescaler=0, period=65535), Pin.cpu.A0, Pin.cpu.A1)
encoder_right = Encoder(Timer(3, prescaler=0, period=65535), Pin.cpu.A6, Pin.cpu.A7)

line_controller = PIDController(Kp_line,Ki_line,Kd_line,10,10)

motor_left = Motor(Timer(1, freq=20000), Pin.cpu.A8, Pin.cpu.B10, Pin.cpu.B4)  
motor_right = Motor(Timer(4, freq=20000), Pin.cpu.B6, Pin.cpu.C7, Pin.cpu.A9)

left_motor_controller = PIDController(Kp_inner, Ki_inner, Kd_inner, 100, integral_limit=100)
right_motor_controller = PIDController(Kp_inner, Ki_inner, Kd_inner, 100, integral_limit=100)

imu = IMU(I2C(1, I2C.CONTROLLER))

# Read Initial Heading
initial_heading = imu.read_heading()/16 

longitudinal_setpoint = .4  # Setpoint for longitudinal velocity (ft/s)

# Inner Loop Task for Motor Speed Control
def inner_loop_task():
    global last_trigger_time, call_round_block, round_block_state, round_block_start_time, final_step
    # Initialize the motor and encoder objects using the user's specified pins and timers
      
    dt = 0.01 
    duty = 20
    turn_time = .65
    left_correction = 3

    final_step = False
    final_step_state = 0
    
    while True:
        while call_round_block:
            if round_block_state == 0:
                print("Start Reverse")
                round_block_state = 1  #Start the round block by setting the state to 1
                round_block_start_time = time.ticks_ms()
            
            elif round_block_state == 1: # Reverse
                 motor_left.set_duty(-duty-left_correction)
                 motor_right.set_duty(-duty)
                 if time.ticks_diff(time.ticks_ms(),round_block_start_time) > 750:
                     print("Start Turn 1")
                     round_block_state = 2
                     round_block_start_time = time.ticks_ms()
                     
            elif round_block_state == 2: # turn 1
                 motor_left.set_duty(duty+left_correction)
                 motor_right.set_duty(-duty)
                 if time.ticks_diff(time.ticks_ms(),round_block_start_time) > int(turn_time*900):
                     print("Start Straight 1")
                     round_block_state = 3
                     round_block_start_time = time.ticks_ms()                     
                     
            elif round_block_state == 3: # arc
            
                longitudinal_setpoint = 1  # Setpoint for longitudinal velocity (ft/s)
                yaw_rate_setpoint = .95     # Setpoint for yaw rate (rad/s)
                       
                # Get the current motor speeds from encoders
                omega_left = encoder_left.get_speed()
                omega_right = encoder_right.get_speed()
               
                linear_velocity = ((omega_left*WHEEL_RADIUS) + (omega_right*WHEEL_RADIUS))/2
                yaw_rate_measured = imu.read_yaw_rate()*math.pi/180
                
                v_output = outer_controller_v.update(longitudinal_setpoint, linear_velocity, dt)
                yaw_output = outer_controller_yaw.update(yaw_rate_setpoint, yaw_rate_measured, dt)
                
                # Calculate motor speeds using the decoupling matrix
                omega_l, omega_r = calculate_motor_speeds(v_output, yaw_output)
                
                # Update PI controllers for each motor
                left_pwm = left_motor_controller.update(omega_l, omega_left, dt)
                right_pwm = right_motor_controller.update(omega_r, omega_right, dt)
                
                if left_pwm > 100:
                    left_pwm = 100
                elif left_pwm < -100:
                    left_pwm = -100
                if right_pwm > 100:
                    right_pwm = 100
                elif right_pwm < -100:
                    right_pwm = -100
                    
                motor_left.set_duty(left_pwm)
                motor_right.set_duty(right_pwm)

                if time.ticks_diff(time.ticks_ms(),round_block_start_time) > 3500:
                     print("Start Straight 1")
                     round_block_state = 4
                     round_block_start_time = time.ticks_ms()                               
                
            elif round_block_state == 4: # find line
                 if qtrx.is_line():
                     motor_left.set_duty(duty+left_correction)
                     motor_right.set_duty(-duty)
                     print("Small Turn")
                     if time.ticks_diff(time.ticks_ms(),round_block_start_time) > 400:
                         print("Leaving Box Loop")
                         round_block_state = 0
                         call_round_block = False
                         final_step = True
                   
            yield 0
        
        if final_step and crossed_line:
            print("Final")
            while True:
                if final_step_state == 0:
                    final_step_state = 1  #Start the round block by setting the state to 1
                    round_block_start_time = time.ticks_ms()
                
                elif final_step_state == 1: # Forward into finish
                    motor_left.set_duty(duty)
                    motor_right.set_duty(duty)
                    if time.ticks_diff(time.ticks_ms(),round_block_start_time) > 1200:
                        print("Enter Finish")
                        final_step_state = 2
                        round_block_start_time = time.ticks_ms()
                        
                elif final_step_state == 2: # Stop
                    motor_left.set_duty(0)
                    motor_right.set_duty(0)
                    if time.ticks_diff(time.ticks_ms(),round_block_start_time) > 1000:
                        print("Stop")
                        final_step_state = 3
                        round_block_start_time = time.ticks_ms()

                elif final_step_state == 3: # turn around
                    motor_left.set_duty(10+left_correction)
                    motor_right.set_duty(-10)
                    if abs(initial_heading - imu.read_heading()/16) <= 0.5:
                        print("Turn around")
                        final_step_state = 4
                        motor_left.set_duty(0)
                        motor_right.set_duty(0)
                        round_block_start_time = time.ticks_ms()

                elif final_step_state == 4: # Go straight
                        motor_left.set_duty(duty-.5)
                        motor_right.set_duty(duty)  
                        if time.ticks_diff(time.ticks_ms(),round_block_start_time) > 6600:
                            print("Done")
                            motor_left.set_duty(0)
                            motor_right.set_duty(0)
                            raise(KeyboardInterrupt)
                
                yield 0

        # Get the current motor speeds from encoders
        omega_left = encoder_left.get_speed()
        omega_right = encoder_right.get_speed()

        # Retrieve target speeds from shared variables
        omega_left_set = left_motor_speed.get()
        omega_right_set = right_motor_speed.get()
        
        # Update PI controllers for each motor
        left_pwm = left_motor_controller.update(omega_left_set, omega_left, dt)
        right_pwm = right_motor_controller.update(omega_right_set, omega_right, dt)
        
        if left_pwm > 100:
            left_pwm = 100
        elif left_pwm < -100:
            left_pwm = -100
        if right_pwm > 100:
            right_pwm = 100
        elif right_pwm < -100:
            right_pwm = -100

        # Set motor PWM duty cycles based on PI control output
        motor_left.set_duty(left_pwm)
        motor_right.set_duty(right_pwm)

        yield 0  # Yield for multitasking

# Outer Loop Task for Velocity and Yaw Rate Control
def outer_loop_task():
    alpha = 0
    previous_filtered_yaw = 0
    global crossed_line

    #initial_heading = imu.read_heading()
    
    dt = .03
    while True:
        # Read actual velocity and yaw rate from IMU
        # Get the current motor speeds from encoders
        omega_left = encoder_left.get_speed()
        omega_right = encoder_right.get_speed()
        
        linear_velocity = ((omega_left*WHEEL_RADIUS) + (omega_right*WHEEL_RADIUS))/2
    
        # Retrieve feedback values for the outer loop
        v_measured = linear_velocity
        yaw_rate_measured = imu.read_yaw_rate()*math.pi/180
        filtered_yaw = alpha * previous_filtered_yaw + (1 - alpha) * yaw_rate_measured
        previous_filtered_yaw = filtered_yaw

        
        # Line following logic

        raw_values = qtrx.read_all_sensors()
        normalized_values = [qtrx.normalize_reading(r,min_value=0, max_value=1000) for r in raw_values]
        thresholded_values = [qtrx.threshold_reading(n) for n in normalized_values]
        centroid = qtrx.calculate_centroid(thresholded_values)
        #print(thresholded_values)

        
        if final_step and sum(thresholded_values) >= 7:
            crossed_line = True

        if centroid > 0:
            line_following_control_output = line_controller.update(3.5,centroid,dt)
        else:
            line_following_control_output = 0

        # Update PI controllers for longitudinal and yaw control
        v_output = outer_controller_v.update(longitudinal_setpoint, v_measured, dt)
        yaw_output = outer_controller_yaw.update(line_following_control_output, filtered_yaw, dt)
        
        # Calculate motor speeds using the decoupling matrix
        omega_l, omega_r = calculate_motor_speeds(v_output, yaw_output)
        
        while call_round_block:
            yield 0 

        # Update shared variables for inner loop control
        left_motor_speed.put(omega_l)
        right_motor_speed.put(omega_r)

        yield 0  # Yield for multitasking

if __name__ == "__main__":
    calibrate_imu_before_running()
    time.sleep(5)                 # Break to romi in the right position to run
    
    # Task Scheduler Setup
    task_inner = cotask.Task(inner_loop_task, name='InnerLoop', priority=2, period=10, profile=True, trace=False)
    task_outer = cotask.Task(outer_loop_task, name='OuterLoop', priority=1, period=30, profile=True, trace=False)

    # Add tasks to the scheduler task list
    cotask.task_list.append(task_inner)
    cotask.task_list.append(task_outer)

    # Setup external interrupts
    pyb.ExtInt(BMP0, pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP, bumper_isr)
    pyb.ExtInt(BMP1, pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP, bumper_isr)
    pyb.ExtInt(BMP2, pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP, bumper_isr)

    pyb.ExtInt(BMP3, pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP, bumper_isr)
    pyb.ExtInt(BMP4, pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP, bumper_isr)
    pyb.ExtInt(BMP5, pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP, bumper_isr)

    # Run the task scheduler
    while True:
        try:
            cotask.task_list.pri_sched()
        except KeyboardInterrupt:
            mot_A = Motor(Timer(1, freq=20000), Pin.cpu.A8, Pin.cpu.B10, Pin.cpu.B4)  # Replace with actual pins
            mot_B = Motor(Timer(4, freq=20000), Pin.cpu.B6, Pin.cpu.C7, Pin.cpu.A9)
            mot_A.set_duty(0)
            mot_B.set_duty(0)
            break
        
    print('\n' + str (cotask.task_list))