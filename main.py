import novapi
import time
import math
from mbuild.encoder_motor import encoder_motor_class
from mbuild import power_expand_board
from mbuild import gamepad
from mbuild.led_matrix import led_matrix_class
from mbuild.smart_camera import smart_camera_class
from mbuild.ranging_sensor import ranging_sensor_class
from mbuild.smartservo import smartservo_class
from mbuild import power_manage_module

left_forward_wheel = encoder_motor_class("M3", "INDEX1")
right_forward_wheel = encoder_motor_class("M5", "INDEX1")
left_back_wheel = encoder_motor_class("M1", "INDEX1")
right_back_wheel = encoder_motor_class("M6", "INDEX1")

MAX_SPEED = 255

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.setpoint = setpoint  # Desired value (target)
        self.integral = 0  # Sum of errors over time
        self.previous_error = 0  # Previous error (used for derivative)

    def update(self, current_value):
        # Calculate the error (setpoint - current value)
        error = self.setpoint - current_value
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.integral += error
        I = self.Ki * self.integral

        # Derivative term
        derivative = error - self.previous_error
        D = self.Kd * derivative

        # Calculate the output
        output = P + I + D

        # Save the current error for the next update
        self.previous_error = error

        return output

    def set_setpoint(self, setpoint):
        """ Update the target setpoint for the PID controller """
        self.setpoint = setpoint
        self.integral = 0  # Reset the integral to avoid wind-up
        self.previous_error = 0  # Reset previous error to avoid a large derivative spike
        
class motors:
    
    def drive(lf: int, lb: int, rf: int, rb: int):
        left_forward_wheel.set_speed(lf)             # LEFT FORWARD
        left_back_wheel.set_speed(lb) # left back :DDDDD
        right_forward_wheel.set_speed(-(rf))      # RIGHT FORWARD
        right_back_wheel.set_speed(-rb)  # RIGHT BACK  
   
    def stop():
        motors.drive(0, 0, 0, 0)
        
class util:
    def restrict(val, minimum, maximum):
        return max(min(val, maximum), minimum)
    
class holonomic:    
    
    pid = {
        "vx": PID(0.5, 0.1, 0.1),
        "vy": PID(0.5, 0.1, 0.1)
    }
    
    """
    Holonomic driving system for mecanum.
    vx, the desired x velocity
    vy, the desired y velocity
    wL, the desired angular velocity
    deadzone, the deadzone where the value lower than this value will be set to 0
    """
    def drive(self, vx, vy, wL, deadzone=5):
        # Create a deadzone so that if the joystick isn't moved perfectly,
        # the controller can still make the robot move perfectly.
        if math.fabs(vx) < math.fabs(deadzone):
            vx = 0
        if math.fabs(vy) < math.fabs(deadzone):
            vy = 0
        if math.fabs(wL) < math.fabs(deadzone):
            wL = 0
            
        multiplier = 2 # SPEED MULTIPLIER
        
        self.pid["vx"].set_setpoint(vx)
        vx = vx * self.pid["vx"].update(novapi.get_acceleration("x"))
        self.pid["vx"].set_setpoint(vy)
        vy = vy * self.pid["vy"].update(novapi.get_acceleration("y"))

        # Calculation for the wheel speed
        # Visit https://github.com/neumann-lab/holonomic-mecanum/blob/main/th.md for the formula
        vFL = (vx + vy + wL) * multiplier
        vFR = (-vx + vy - wL) * multiplier
        vBL = (-vx + vy + wL) * multiplier
        vBR = (vx + vy - wL) * multiplier

        # Velocity
        vFL = util.restrict(vFL, -MAX_SPEED, MAX_SPEED)
        vFR = util.restrict(vFR, -MAX_SPEED, MAX_SPEED)
        vBL = util.restrict(vBL, -MAX_SPEED, MAX_SPEED)
        vBR = util.restrict(vBR, -MAX_SPEED, MAX_SPEED)
        # Drive motor
        motors.drive(vFL, vBL, vFR, vBR)
        
robot_holonomic = holonomic()

while True:
    if math.fabs(gamepad.get_joystick("Lx")) > 20 or math.fabs(gamepad.get_joystick("Ly")) > 20 or math.fabs(gamepad.get_joystick("Rx")) > 20:
        robot_holonomic.drive(-gamepad.get_joystick("Lx"), gamepad.get_joystick("Ly"), -(1.2 * gamepad.get_joystick("Rx")))
    else:
        motors.drive(0,0,0,0)
