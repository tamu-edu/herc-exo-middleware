""" An object that inherits from ActPackMan and wraps Dephy ExoBoot """

from ActPackMan import ActPackMan
from ActPackMan import FlexSEA
from ActPackMan import DEFAULT_VARIABLES
import numpy as np
import time

class EB51Man(ActPackMan):
    def __init__(self, devttyACMport, baudRate=230400, csv_file_name=None,
        hdf5_file_name=None, vars_to_log=DEFAULT_VARIABLES, gear_ratio=1.0,
        printingRate = 10, updateFreq = 100, shouldLog = False, logLevel=6, slack = 1.0, 
        whiplashProtect = True):

        super(EB51Man, self).__init__(devttyACMport, baudRate=230400, csv_file_name=None,
        hdf5_file_name=None, vars_to_log=DEFAULT_VARIABLES, gear_ratio=1.0,
        printingRate = 10, updateFreq = 100, shouldLog = False, logLevel=6)

        self.slack = slack
        self.whiplashProtect = whiplashProtect

    
    def update(self):
        # fetches new data from the device
        if not self.entered:
            raise RuntimeError("ActPackMan updated before __enter__ (which begins the streaming)")
        currentTime = time.time()
        if abs(currentTime-self.prevReadTime)<0.25/self.updateFreq:
            print("warning: re-updating twice in less than a quarter of a time-step")
        self.act_pack = FlexSEA().read_device(self.dev_id) # a c-types struct
        self.gear_ratio = self._calculate_gear_ratio()  # Update gear ratio for ankle angle output
        self.prevReadTime = currentTime

        # Automatically save all the data as a csv file
        if self.csv_file_name is not None:
            self.csv_writer.writerow([time.time()]+[getattr(self.act_pack,x) for x in self.vars_to_log])

        if self.hdf5_file_name is not None:
            raise NotImplemented()

    # Private functions

    def _calculate_gear_ratio(self):
        " Gear ratio only accurate if no slack in belt "
        ank_ang_deg = self.get_ank_ang_deg()
        if (ank_ang_deg >= 33 and ank_ang_deg < 118):
            return 14.7536
        if (ank_ang_deg >= 118 and ank_ang_deg < 152):
            return -0.724*(ank_ang_deg - 118) + 14.7536
        if (ank_ang_deg >= 152 and ank_ang_deg < 180):
            return -9.853
        print("Warning: gear ratio not updated")
        return self.gear_ratio

    def _check_slack_condition(self):   ## Fill in function
        pass
        
    # Tension condition

    def get_ank_ang_deg(self):
        " Convert ankle angle to degrees "
        if (self.act_pack is None):
            raise RuntimeError("ActPackMan not updated before state is queried.")
        return self.act_pack.ank_ang * 2*np.pi / pow(2,14) * 180/np.pi

    def get_output_angle_radians(self):
        return self.get_ank_ang_deg() * np.pi/180

    def get_output_velocity_radians_per_second(self):
        return self.get_motor_velocity_radians_per_second()/self.gear_ratio

    def get_output_acceleration_radians_per_second_squared(self):
        return self.get_motor_acceleration_radians_per_second_squared()/self.gear_ratio

    def get_output_torque_newton_meters(self): 
        return self.get_motor_torque_newton_meters()*self.gear_ratio

    #### 

    def set_output_angle_radians(self, angle, slacked = False):  ### Need to check 
        " Input angle is the ankle angle (in degrees) " 
        " Function sends command to set motor angle corresponding to ankle angle "
        if slacked == True & self.gear_ratio >= 0:
            target_angle = angle + self.slack
        elif slacked == True & self.gear_ratio < 0:
            target_angle = angle - self.slack
        elif slacked == False:
            target_angle = angle 
        motor_angle = self.get_desired_motor_angle_radians(target_angle) 
        self.set_motor_angle_radians(motor_angle) 

    def set_output_velocity_radians_per_second(self, vel):
        raise NotImplemented()

    def set_output_acceleration_radians_per_second_squared(self, acc):
        raise NotImplemented()
    
    def set_output_torque_newton_meters(self, torque):   ## Needs to be updated
        " Criteria for which direction torque can be applied depending on sign of gear ratio " 
        " If commanded for non-executable torque, use position control to keep motor in slightly slacked position " 
        if (self.gear_ratio == 0) or (np.sign(self.gear_ratio) != np.sign(torque)):
            torque = 0
            print("Warning: torque command not executable")
            self.set_position_gains()
            self.set_output_angle_radians(angle = self.get_ank_ang_deg(), slacked = True)
            self.set_current_gains() 
        self.set_motor_torque_newton_meters(torque/self.gear_ratio)
        

    # Slack condition

    def get_desired_motor_angle_radians(self, output_angle):   ## Need to check
        " Calculate motor angle based on ankle angle "
        " output_angle in degrees "
        if (output_angle >= 33) & (output_angle < 118):
            return (14.754*(output_angle - 33) - 1001.518)*np.pi/180
        if (output_angle >= 118) & (output_angle < 152):
            return (-0.362*pow(output_angle - 118, 2) + 14.754*(output_angle - 118) + 252.537)*np.pi/180
        if (output_angle >= 152) & (output_angle <= 180):
            return (-9.853*(output_angle - 152) + 335.842)*np.pi/180
        raise RuntimeError("Not valid output angle") 

    def set_slack(self, slack):
        " Define the amount of slack (in output/ankle degrees) to allow "
        self.slack = slack

    # Output-side variables (ankle)

    θ = property(get_output_angle_radians, set_output_angle_radians)
    θd = property(get_output_velocity_radians_per_second, 
        set_output_velocity_radians_per_second, doc="output_velocity_radians_per_second")
    θdd = property(get_output_acceleration_radians_per_second_squared, 
        set_output_acceleration_radians_per_second_squared,
        doc="output_acceleration_radians_per_second_squared")

    τ = property(get_output_torque_newton_meters, set_output_torque_newton_meters,
        doc="output_torque_newton_meters")


