from tkinter import *
from tkinter import ttk
from tkinter import messagebox
import time
from labjack import ljm
import numpy as np
import serial
import struct
import crcmod
import threading
import datetime
import sys
#import matplotlib
#import pandas as pd

class LabJack_Motor_PWM:

    def __init__(self, mainframe, **kwargs):        
        
        # Position
        self.mainframe = mainframe
        self.ROW_START = kwargs.get("ROW", 0)
        self.COL_START = kwargs.get("COL", 0)
        self.DIO_NUM_PWM = str(kwargs.get("DIO_PWM", 6))
        self.DIO_NUM_Enable = str(kwargs.get("DIO_EN", 5))
        self.DIO_NUM_Dir  = str(kwargs.get("DIO_DIR", 4))

        self.Duty_Cycle = 50
        self.Frequency = 10

        self.Motor_Enable = 0
        self.Motor_Dir = 0
        self.RPM = 10
        self.Switch_Position = "11001111"
        self.DIO_Value = 0

        self.CLOCK_DVISORS = [1, 2, 4, 8, 16, 32, 64, 256] #Valid Clock Divider settings
        self.CLOCK_SPEED = 100*10**6 # 100MHz Clock
        self.MAX_RPM = 1000
        self.PPR_LUT = {
            0b1111: 200,
            0b0111: 400,
            0b1011: 800,
            0b0011: 1600,
            0b1101: 3200,
            0b0101: 6400,
            0b1001: 12800,
            0b0001: 25600,
            0b1110: 1000,
            0b0110: 2000,
            0b1010: 4000,
            0b0010: 5000,
            0b1100: 8000,
            0b0100: 10000,
            0b1000: 20000,
            0b0000: 25000,
        }
        self.CURRENT_MAX = 0b000
        
        # Initializing Variable Types
        self.Duty_Cycle_Disp = StringVar()
        Duty_Cycle_entry = ttk.Entry(mainframe, width=7, textvariable=self.Duty_Cycle_Disp)
        Duty_Cycle_entry.grid(column=2+self.COL_START, row=7+self.ROW_START, sticky=(W, E))
        self.Duty_Cycle_Disp.set(self.Duty_Cycle)

        self.Frequency_Disp = StringVar()
        Frequency_entry = ttk.Entry(mainframe, width=7, textvariable=self.Frequency_Disp)
        Frequency_entry.grid(column=2+self.COL_START, row=5+self.ROW_START, sticky=(W, E))
        self.Frequency_Disp.set(self.Frequency)

        self.RPM_Disp = StringVar()
        RPM_entry = ttk.Entry(mainframe, width=7, textvariable=self.RPM_Disp)
        RPM_entry.grid(column=2+self.COL_START, row=6+self.ROW_START, sticky=(W, E))
        self.RPM_Disp.set(self.RPM)

        self.Switch_Position_Disp = StringVar()
        Switch_Position_entry = ttk.Entry(mainframe, width=7, textvariable=self.Switch_Position_Disp)
        Switch_Position_entry.grid(column=2+self.COL_START, row=2+self.ROW_START, sticky=(W, E))
        self.Switch_Position_Disp.set(self.Switch_Position)

        self.Pulse_Per_Rev = self.PPR_Get()
        
        self.PWM_Update()
        
        self.DIO_Value_Disp = StringVar()
        self.Motor_Enable_Disp = StringVar()
        self.Motor_Dir_Disp = StringVar()


        # Initializing Buttons, Textvariables (Display Variables, which change as thier values do) and Static Text
        ttk.Button(mainframe, textvariable=self.Motor_Enable_Disp, command=self.Enable_Motor).grid(column=2+self.COL_START, row=3, sticky=W)
        ttk.Button(mainframe, textvariable=self.Motor_Dir_Disp, command=self.Dir_Motor).grid(column=2+self.COL_START, row=4, sticky=W)
        
        ttk.Button(mainframe, text="Update", command=self.PPR_Update).grid(column=3+self.COL_START, row=2, sticky=W)
        ttk.Button(mainframe, text="Update", command=self.Freq_Update).grid(column=3+self.COL_START, row=5, sticky=W)
        ttk.Button(mainframe, text="Update", command=self.RPM_Update).grid(column=3+self.COL_START, row=6, sticky=W)
        ttk.Button(mainframe, text="Update", command=self.DC_Update).grid(column=3+self.COL_START, row=7, sticky=W)
        
        # Organization
        ttk.Label(mainframe, text="Inputs:").grid(column=1+self.COL_START, row=1+self.ROW_START, sticky=E)
        ttk.Label(mainframe, text="Output:").grid(column=1+self.COL_START, row=8+self.ROW_START, sticky=E)

        # Current IO
        ttk.Label(mainframe, textvariable=self.DIO_Value_Disp).grid(column=2+self.COL_START, row=9+self.ROW_START, sticky=(W, E))

        # Motor Enable
        ttk.Label(mainframe, text="ON/OFF").grid(column=3+self.COL_START, row=3+self.ROW_START, sticky=(E))
        ttk.Label(mainframe, text="Enable").grid(column=1+self.COL_START, row=3+self.ROW_START, sticky=(E))

        # Motor Dir
        ttk.Label(mainframe, text="Back/Fore").grid(column=3+self.COL_START, row=4+self.ROW_START, sticky=(E))
        ttk.Label(mainframe, text="Direction").grid(column=1+self.COL_START, row=4+self.ROW_START, sticky=(E))

        # Input Entry
        ttk.Label(mainframe, text="Stepper Switch State").grid(column=1+self.COL_START, row=2+self.ROW_START, sticky=E)
        ttk.Label(mainframe, text="Frequency (Hz):").grid(column=1+self.COL_START, row=5+self.ROW_START, sticky=E)
        ttk.Label(mainframe, text="RPM:").grid(column=1+self.COL_START, row=6+self.ROW_START, sticky=E)
        ttk.Label(mainframe, text="Duty Cycle (%):").grid(column=1+self.COL_START, row=7+self.ROW_START, sticky=E)
        
        # Output
        ttk.Label(mainframe, text="DIO State:").grid(column=1+self.COL_START, row=9+self.ROW_START, sticky=E)

        RPM_entry.focus() # Automatically have frequency block focused (Cursor already clicked)

        # Initialize Defaults

        self.Current_DIO_State()
        self.Enable_Motor()
        self.Dir_Motor()
        
    def Pullup_Conversion(self, value):
        if value == 1:
            return "LOW"
        elif value == 0:
            return "HIGH"
        else:
            print("Invalid Input")
            return "INVALID RESULT"
    
    def Dir_Conversion(self, value):
        if value == 1:
            return "Backwards"
        elif value == 0:
            return "Forwards"
        else:
            print("Invalid Input")
            return "INVALID RESULT"
    
    def Enable_Conversion(self, value):
        if value == 1:
            return "OFF"
        elif value == 0:
            return "ON"
        else:
            print("Invalid Input")
            return "INVALID RESULT"

    def Freq_Update(self):
        self.Frequency = float(self.Frequency_Disp.get())
        self.PWM_Update(self, "Freq")

    def RPM_Update(self):
        self.RPM = float(self.RPM_Disp.get())
        self.PWM_Update(self, "RPM")

    def DC_Update(self):
        self.Duty_Cycle = float(self.Duty_Cycle_Disp.get())
        self.PWM_Update(self, "DC")

    def PPR_Update(self):
        self.Switch_Position = self.Switch_Position_Disp.get()
        self.Current_Check()
        self.PWM_Update(self, "PPR")
        
    
    def Current_Check(self):
        #print(int(self.Switch_Position.get()[0:2], 2))
        if self.CURRENT_MAX >= int(self.Switch_Position[0:2], 2):
            PPR = self.Switch_Position[-5:]
            self.Switch_Position = "001" + PPR
            print("Current Draw set too High Motor Maximum is 3A")
            

    def PPR_Get(self):
        #print(self.Switch_Position.get()[-4:])
        return self.PPR_LUT[int(self.Switch_Position[-4:], 2)]

    def PWM_Update(self, *args):
        try:
            try:
                Type = args[1]
            except:
                Type = "ALL"

            
            """
            PWM Works by setting the freq and duty
            DIO#_EF_ENABLE: 0 = Disable, 1 = Enable
            DIO#_EF_INDEX: 0 <- Specifies PWM mode
            DIO#_EF_CONFIG_A: When the specified clocks source's count matches this value, the line will transition from high to low.
            DIO#_EF_OPTIONS: Bits 0-2 specify which clock source to use ... b000 for Clock0, b001 for
            Clock1, and b010 for Clock2. All other bits are reserved and should be set to 0.
                CLOCK0: 32-bit
                CLOCK1: 16-bit
                CLOCK2: 16-bit
            Clock setup (Example here is 10Hz):
            DIO_EF_CLOCK0_ENABLE = 0
            DIO_EF_CLOCK0_DIVISOR = 16 # 100 MHz / 16 = 6.25MHz
            DIO_EF_CLOCK0_ROLL_VALUE = 625000
            DIO_EF_CLOCK0_ENABLE = 1
            """

            self.Pulse_Per_Rev = self.PPR_Get()
            
            self.Duty_Cycle = self.DC_Check(self.Duty_Cycle)

            if Type == "Freq":
                self.RPM = self.RPM_Check(self.Freq_to_RPM(self.Frequency))
                self.Frequency = self.RPM_to_Freq(self.RPM)

            else:
                self.RPM = self.RPM_Check(self.RPM)
                self.Frequency = self.RPM_to_Freq(self.RPM)
        
            Desired_Clock_Divisor = self.CLOCK_SPEED/self.Frequency # Find Clock Dividor for desired freq

            Clock_Divider_Setting = self.find_largest(self.CLOCK_DVISORS, Desired_Clock_Divisor) # Find the biggeest one

            Clock_Roll = int((self.CLOCK_SPEED/Clock_Divider_Setting)/self.Frequency) # Find clock roll for the desired clock divider chosen

            PWM_ROLLOVER = int((self.Duty_Cycle/100)*Clock_Roll) # Find the Duty Cycle Roll value

            ljm.eWriteName(handle, "DIO_EF_CLOCK1_ENABLE", 0) #Disables clock to change clock properties
            ljm.eWriteName(handle, "DIO_EF_CLOCK1_DIVISOR", Clock_Divider_Setting) # 100 MHz Core Clock / 16 = 6.25MHz
            ljm.eWriteName(handle, "DIO_EF_CLOCK1_ROLL_VALUE", Clock_Roll) # Set roll to 0.625M, so repeats every 10Hz
            ljm.eWriteName(handle, "DIO_EF_CLOCK1_ENABLE", 1) # Clock Re-enable
            
            ljm.eWriteName(handle, "DIO"+str(self.DIO_NUM_PWM)+"_EF_ENABLE", 0) # Disable before changing, 0 = Disable, 1 = Enable PWM
            ljm.eWriteName(handle, "DIO"+str(self.DIO_NUM_PWM)+"_EF_INDEX", 0) # 0 <- Specifies PWM mode
            ljm.eWriteName(handle, "DIO"+str(self.DIO_NUM_PWM)+"_EF_CONFIG_A", PWM_ROLLOVER) #When the specified clocks source's count matches this value
            ljm.eWriteName(handle, "DIO"+str(self.DIO_NUM_PWM)+"_EF_OPTIONS", 0b001) # Bits 0-2 specify which clock source to use ... b000 for Clock0, b001 for Clock1 etc...
            ljm.eWriteName(handle, "DIO"+str(self.DIO_NUM_PWM)+"_EF_ENABLE", 1) #0 = Disable, 1 = Enable PWM
            
            global Update_PD
            Update_PD = True

            #print(PWM_ROLLOVER)

        except ValueError:
            print("Invalid Input")

    def RPM_to_Freq(self, RPM):
        return self.Pulse_Per_Rev*(RPM/60)
        
    def Freq_to_RPM(self, freq):
        return (freq/self.Pulse_Per_Rev)*60

    def DC_Check(self, dc):
        if dc > 100:
            dc = 100
            print("Invalid Duty Cycle, choose between 0-100%")
        elif dc < 0:
            dc = 0
            print("Invalid Duty Cycle, choose between 0-100%")

        return dc

    def RPM_Check(self, RPM):
        if RPM > self.MAX_RPM:
            RPM = 100
            print("Invalid RPM, Max is 1000")
        elif RPM <= 0:
            print("Minimum RPM is >0, to reverse change the Direction, to Stop spinning Press Enable")
            self.Enable_Motor(1) # 1 diasables motor since pull-down inversion
            RPM = 1
        return RPM
    
    def Current_DIO_State(self):
        #print("{:08b}".format(int(ljm.eReadName(handle, "FIO_STATE"))))
        self.DIO_Value = (int(ljm.eReadName(handle, "FIO_STATE")) & 1<<int(self.DIO_NUM_PWM))>>int(self.DIO_NUM_PWM) # Reads Current Output state of FIO+self.DIO_NUM
        #self.mainframe.after(10, self.Current_DIO_State)

    
    def Enable_Motor(self, enable=None):
        global Update_PD
        Update_PD = True

        if isinstance(enable, type(None)):
            if self.Motor_Enable == 0:
                ljm.eWriteName(handle, "FIO"+str(self.DIO_NUM_Enable), 1)
                self.Motor_Enable = 1
            else:
                ljm.eWriteName(handle, "FIO"+str(self.DIO_NUM_Enable), 0)
                self.Motor_Enable = 0
        else:
            ljm.eWriteName(handle, "FIO"+str(self.DIO_NUM_Enable), enable)    

    def Dir_Motor(self, dir=None):
        global Update_PD
        Update_PD = True

        if isinstance(dir, type(None)):
            if self.Motor_Dir == 0:
                ljm.eWriteName(handle, "FIO"+str(self.DIO_NUM_Dir), 1)
                self.Motor_Dir = 1
            else:
                ljm.eWriteName(handle, "FIO"+str(self.DIO_NUM_Dir), 0)
                self.Motor_Dir = 0
        else:
            ljm.eWriteName(handle, "FIO"+str(self.DIO_NUM_Dir), dir) 


    def find_largest(self, array, value):
        array = np.asarray(array)
        idx = array[(array - value) < 0].argmax()
        return array[idx]

    def Disp_Update(self):
        self.DIO_Value_Disp.set(self.Pullup_Conversion(self.DIO_Value)) # Reads Current Output state of FIO+self.DIO_NUM
    
    def Disp_Update_Inputs(self):
        self.Motor_Enable_Disp.set(self.Enable_Conversion(self.Motor_Enable))
        self.Motor_Dir_Disp.set(self.Dir_Conversion(self.Motor_Dir))
        self.Switch_Position_Disp.set(self.Switch_Position)
        self.Frequency_Disp.set(round(self.Frequency, 2))
        self.Duty_Cycle_Disp.set(round(self.Duty_Cycle, 2))
        self.RPM_Disp.set(round(self.RPM, 2))

class LabJack_DIO:
    def __init__(self, mainframe, **kwargs):
        # Position
        self.mainframe = mainframe
        self.ROW_START = kwargs.get("ROW", 0)
        self.COL_START = kwargs.get("COL", 0)
        
        self.Name = kwargs.get("DIO_NAME", "DIO")
        self.DIO_NUM = str(kwargs.get("DIO_NUM", 0))
        self.DIO_TYPE = str(kwargs.get("DIO_TYPE", "FIO"))
        self.DIO_STATES = str(kwargs.get("DIO_STATES", "On/Off"))
        self.DIO_DEFAULT = kwargs.get("DIO_DEFAULT", 0)

        self.DIO = self.DIO_TYPE + self.DIO_NUM

        self.DIO_Value = 0
        self.DIO_Value_Disp = StringVar()

        # Organization / Labels
        ttk.Label(mainframe, text="{}: ".format(self.Name)).grid(column=1+self.COL_START, row=0+self.ROW_START, sticky=E)

        # AIN Readouts
        ttk.Button(mainframe, textvariable=self.DIO_Value_Disp, command=self.Change_DIO).grid(column=2+self.COL_START, row=0+self.ROW_START, sticky=W)
        ttk.Label(mainframe, text=self.DIO_STATES).grid(column=3+self.COL_START, row=0+self.ROW_START, sticky=(W, E))
        
        # Initialize Defaults
        self.Change_DIO(self.DIO_DEFAULT)
        self.Set_Current_DIO()

    def Change_DIO(self, value=None):
        if isinstance(value, type(None)):
            if self.DIO_Value == 1:
                self.DIO_Value = 0
                ljm.eWriteName(handle, self.DIO, self.DIO_Value) 
            elif self.DIO_Value == 0:
                self.DIO_Value = 1
                ljm.eWriteName(handle, self.DIO, self.DIO_Value)
            else:
                print("Invalid Input")
        else:
            ljm.eWriteName(handle, self.DIO, value)      
        self.Set_Current_DIO()

    def Set_Current_DIO(self):
        self.DIO_Value_Disp.set(self.Text_Conversion(self.DIO_Value))

    def Text_Conversion(self, value):
        return value

class Relay(LabJack_DIO):
    def Text_Conversion(self, value):
        if value == 1:
            return "ON"
        elif value == 0:
            return "OFF"
        else:
            print("Invalid Input")
            return "INVALID RESULT"

class LabJack_AIN:

    def __init__(self, mainframe, **kwargs):        
        
        # Position
        self.mainframe = mainframe
        self.ROW_START = kwargs.get("ROW", 0)
        self.COL_START = kwargs.get("COL", 0)
        
        self.Name = kwargs.get("AIN_NAME", "AIN")
        self.AIN_NUM = str(kwargs.get("AIN_NUM", 0))
        self.Unit = kwargs.get("AIN_UNIT", "N_A")

        self.AIN = "AIN" + self.AIN_NUM

        self.AIN_Value = 0
        self.Converted_AIN = 0

        self.AIN_Value_Disp = StringVar()
        self.Converted_AIN_Disp = StringVar()

        # Organization / Labels
        ttk.Label(mainframe, text="Voltage Output (V):").grid(column=1+self.COL_START, row=2+self.ROW_START, sticky=E)
        ttk.Label(mainframe, text="{} ({}): ".format(self.Name, self.Unit)).grid(column=1+self.COL_START, row=3+self.ROW_START, sticky=E)

        # AIN Readouts
        ttk.Label(mainframe, textvariable=self.AIN_Value_Disp).grid(column=2+self.COL_START, row=2+self.ROW_START, sticky=(W, E))
        ttk.Label(mainframe, textvariable=self.Converted_AIN_Disp).grid(column=2+self.COL_START, row=3+self.ROW_START, sticky=(W, E))

        # Initialize Defaults
        self.Get_Current_AIN()
        self.Set_Current_AIN()

    def Get_Current_AIN(self):
        self.AIN_Value = ljm.eReadName(handle, self.AIN) # Reads Current Output of AIN
        self.Calibrate_AIN()         

    def Calibrate_AIN(self):
        self.Converted_AIN = self.Calibration_Formula(float(self.AIN_Value))

    def Set_Current_AIN(self):
        self.AIN_Value_Disp.set(round(self.AIN_Value, 3))
        self.Converted_AIN_Disp.set(round(self.Converted_AIN, 3))

    def Calibration_Formula(self, AIN):
        return AIN


class MAF(LabJack_AIN):

    def Calibration_Formula(self, AIN):
        y = 25.003*(AIN**2) - 58.484*(AIN) + 38.339 # g/s
        #y = (5.9541*AIN**(2.9394)) / 60  # kg/h -> kg/s
        if isinstance(y, complex):
            return 0
        else:
            return y

class Gas_Analyzer(LabJack_AIN):

    def Calibration_Formula(self, AIN):
        return AIN*1000  # Voltage to ppm or %O2

class Gas_Analyzer_NOX(LabJack_AIN):

    def Calibration_Formula(self, AIN):
        return AIN*10000  # Voltage to ppm for NOX only

class TPSA(LabJack_AIN):

    def Calibration_Formula(self, AIN):
        return ((100 - 0)/(3.958 - 0.922)) * (AIN - 0.922)  + 0 # Voltage to Percent Open #y = (y_2-y_1)/(x_2-x_1) * (x-x_1) + y_1

    def Inverse_Calibration_Formula(self, Open_Percent):
        return ((3.958 - 0.922)/(100 - 0)) * (Open_Percent - 0)  + 0.922 # Percent Open to voltage #y = (y_2-y_1)/(x_2-x_1) * (x-x_1) + y_1

"""
class TPSB(LabJack_AIN):

    def Calibration_Formula(self, AIN):
        return ((AIN- 1.6)/(4 - 1.6))*100  # Voltage to Percent Open
"""

class Thermocouple(LabJack_AIN):

    def __init__(self, mainframe, **kwargs):        
        
        # Position
        self.mainframe = mainframe
        self.ROW_START = kwargs.get("ROW", 0)
        self.COL_START = kwargs.get("COL", 0)
        
        self.Name = kwargs.get("AIN_NAME", "AIN")
        self.AIN_NUM = str(kwargs.get("AIN_NUM", 0))
        self.Unit = kwargs.get("AIN_UNIT", "°C")

        self.AIN = "AIN" + self.AIN_NUM

        self.AIN_Value = 0
        self.Converted_AIN = 0

        self.AIN_Value_Disp = StringVar()
        self.Converted_AIN_Disp = StringVar()

        # Organization / Labels
        ttk.Label(mainframe, text="Voltage Output (V):").grid(column=1+self.COL_START, row=2+self.ROW_START, sticky=E)
        ttk.Label(mainframe, text="{} ({}): ".format(self.Name, self.Unit)).grid(column=1+self.COL_START, row=3+self.ROW_START, sticky=E)

        # AIN Readouts
        ttk.Label(mainframe, textvariable=self.AIN_Value_Disp).grid(column=2+self.COL_START, row=2+self.ROW_START, sticky=(W, E))
        ttk.Label(mainframe, textvariable=self.Converted_AIN_Disp).grid(column=2+self.COL_START, row=3+self.ROW_START, sticky=(W, E))

        # Setup LabJack
        
        ljm.eWriteName(handle, self.AIN + "_RANGE", 0.075) # Sets the resolution of the AIN for hte thermocouple for better accuracy
        ljm.eWriteName(handle, self.AIN + "_EF_INDEX", 22) # Specifys Type K thermocouple
        ljm.eWriteName(handle, self.AIN + "_EF_CONFIG_A", 1) # Specifies Reading in C
        """
        ljm.eWriteName(handle, self.AIN + "_EF_CONFIG_B", 60052) # CJC reading address
        #ljm.eWriteName(handle, self.AIN + "_EF_CONFIG_B", 60052) # CJC Default
        ljm.eWriteName(handle, self.AIN + "_EF_CONFIG_D", 1.00) # CJC slope
        ljm.eWriteName(handle, self.AIN + "_EF_CONFIG_E", 0.0) # CJC Offset
        """
        
        # Initialize Defaults
        self.Get_Current_AIN()
        
    def Get_Current_AIN(self):
        self.AIN_Value = ljm.eReadName(handle, self.AIN + "_EF_READ_B") # Reads Current Output of AIN
        self.Read_AIN_Temp()         
        
    def Read_AIN_Temp(self):
        self.Converted_AIN_Value = ljm.eReadName(handle, self.AIN + "_EF_READ_A") # Reads Current Output of AIN

    def Set_Current_AIN(self):
        self.AIN_Value_Disp.set(round(self.AIN_Value, 3))
        self.Converted_AIN_Disp.set(round(self.Converted_AIN_Value, 3))

class AIN_4_20mA(LabJack_AIN): # Need to Update with REOTEMP Response
        
    def __init__(self, mainframe, **kwargs):        
        
        # Position
        self.mainframe = mainframe
        self.ROW_START = kwargs.get("ROW", 0)
        self.COL_START = kwargs.get("COL", 0)
        
        self.Resistance = kwargs.get("Resistance", 500)
        self.Name = kwargs.get("AIN_NAME", "AIN")
        self.AIN_NUM = str(kwargs.get("AIN_NUM", 0))
        self.Unit = kwargs.get("AIN_UNIT", "N_A")

        self.AIN = "AIN" + self.AIN_NUM

        self.AIN_Value = 0
        self.Converted_AIN = 0
        self.Amp_AIN = 0

        self.AIN_Value_Disp = StringVar()
        self.Converted_AIN_Disp = StringVar()
        self.Amp_AIN_Disp = StringVar()

        # Organization / Labels
        ttk.Label(mainframe, text="Voltage Output (V):").grid(column=1+self.COL_START, row=2+self.ROW_START, sticky=E)
        ttk.Label(mainframe, text="Current Output (mA):").grid(column=1+self.COL_START, row=3+self.ROW_START, sticky=E)
        ttk.Label(mainframe, text="{} ({}): ".format(self.Name, self.Unit)).grid(column=1+self.COL_START, row=4+self.ROW_START, sticky=E)

        # AIN Readouts
        ttk.Label(mainframe, textvariable=self.AIN_Value_Disp).grid(column=2+self.COL_START, row=2+self.ROW_START, sticky=(W, E))
        ttk.Label(mainframe, textvariable=self.Amp_AIN_Disp).grid(column=2+self.COL_START, row=3+self.ROW_START, sticky=(W, E))
        ttk.Label(mainframe, textvariable=self.Converted_AIN_Disp).grid(column=2+self.COL_START, row=4+self.ROW_START, sticky=(W, E))

        # Initialize Defaults
        self.Get_Current_AIN()
        self.Calibrate_AIN()

    def Get_Current_AIN(self):
        self.AIN_Value = ljm.eReadName(handle, self.AIN) # Reads Current Output of AIN
        self.Amp_AIN = self.AIN_Value/self.Resistance * 1000 # mA Conversion
        self.Calibrate_AIN()         
    
    def Calibrate_AIN(self):
        self.Converted_AIN = self.Calibration_Formula(float(self.Amp_AIN))
    
    def Calibration_Formula(self, AIN):
        return (1177-(-18))/(20-4) * (AIN-4) + (-18)  # Current to Temperature, 4mA=0F, 20mA=2150F and Linear Interpolation

    def Set_Current_AIN(self):
        self.AIN_Value_Disp.set(round(self.AIN_Value, 3))
        self.Converted_AIN_Disp.set(round(self.Converted_AIN, 3))
        self.Amp_AIN_Disp.set(round(self.Amp_AIN, 3))

class Throttle_PID_Controller:
    def __init__(self, DC, integral, previous_error, setpoint, kp, ki, kd, dt):
        self.DC = DC # Duty Cycle (What we are controlling)
        self.integral = integral
        self.previous_error = previous_error
        self.setpoint = setpoint  # Desired setpoint (50% Open)
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.dt = dt  # Time step
        self.current_open = TPSA_CLASS.AIN_Value
        self.control = 0

    def DC_Check(self):
        """
        Cannot set Duty Cycle higher than 100% or lower than 0%, changes the control to ensure that it is limited
        """
        if self.DC > 100:
            self.control = (self.DC - 100)/self.dt # Control to ensure that DC never gets set above 100
            #print("Check math, this should be 100: ", self.DC + self.control * self.dt)
            self.DC = 100
            
        elif self.DC < 0:
            self.control = (self.DC - 0)/self.dt # Control to ensure that DC never gets set below 0
            #print("Check math, this should be 0: ", self.DC + self.control * self.dt)
            self.DC = 0


    def pid_controller(self):
        error = self.setpoint - self.current_open
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        self.control = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error

    def control_mainloop(self):
        #time_now = time.time()

        global read_index#, time_last
        #print("Actual Samling Rate: ", 1/(time_now - time_last))
        #time_last = time_now

        # Get new value of TPSA
        TPSA_CLASS.Get_Current_AIN()

        match read_index:
            case 0:
                Thermocouple_Reactor_CLASS.Get_Current_AIN()
            case 1:
                Thermocouple_Pre_HX_CLASS.Get_Current_AIN()
            case 2:
                Thermocouple_Post_HX_CLASS.Get_Current_AIN()
            case 3:
                Gas_Analyzer_NOX_CLASS.Get_Current_AIN()
            case 4:
                Gas_Analyzer_O2_CLASS.Get_Current_AIN()
            case 5:
                Gas_Analyzer_CO_CLASS.Get_Current_AIN()
            case 6:
                MAF_CLASS.Get_Current_AIN()
            case 7:
                PD_CLASS.Current_DIO_State()
            case 8:
                Throttle_Motor_CLASS.Current_DIO_State()
                

        read_index = (read_index + 1) % 9
        self.current_open = TPSA_CLASS.AIN_Value

        # Apply PID Control
        self.pid_controller()
        self.DC += self.control * self.dt  #control * dt Update DC based on controller

        # Check what PID wants to do is valid
        self.DC_Check()
            
        # Update PWM for new value
        Throttle_Motor_CLASS.Duty_Cycle = self.DC
        Throttle_Motor_CLASS.PWM_Update()
        #print("Exe: ", time.time()-time_now)
        

    def PID_Update(self, setpoint):
        self.setpoint = setpoint

class Throttle_PWM(LabJack_Motor_PWM):

    def __init__(self, mainframe, **kwargs):

        self.mainframe = mainframe
        self.ROW_START = kwargs.get("ROW", 0)
        self.COL_START = kwargs.get("COL", 0)
        self.DIO_NUM_PWM = kwargs.get("DIO_NUM", 7)
        self.CLOCK_DVISORS = [1, 2, 4, 8, 16, 32, 64, 256] #Valid Clock Divider settings
        self.CLOCK_SPEED = 100*10**6 # 100MHz Clock
        self.Duty_Cycle = 0
        self.Frequency = 20000
        self.Percent_Open = 0
        
        # Initializing Variable Types
        self.Percent_Open_Disp = StringVar()
        self.Percent_Open_Disp.set(round(self.Percent_Open, 2))

        Percent_Open_entry = ttk.Entry(mainframe, width=7, textvariable=self.Percent_Open_Disp)
        Percent_Open_entry.grid(column=2+self.COL_START, row=3+self.ROW_START, sticky=(W, E))
        
        Throttle_PID_CLASS.PID_Update(self.Percent_Open)

        self.Frequency_Disp = StringVar()
        Frequency_entry = ttk.Entry(mainframe, width=7, textvariable=self.Frequency_Disp)
        Frequency_entry.grid(column=2+self.COL_START, row=2+self.ROW_START, sticky=(W, E))
        self.Frequency_Disp.set(round(self.Frequency, 2))
        
        self.PWM_Update()
        
        self.DIO_Value_Disp = StringVar()
        self.Duty_Cycle_Disp = StringVar()

        # Initializing Buttons, Textvariables (Display Variables, which change as thier values do) and Static Text
        ttk.Button(mainframe, text="Update", command=self.Freq_Update).grid(column=3+self.COL_START, row=2+self.ROW_START, sticky=W)
        ttk.Button(mainframe, text="Update", command=self.Percent_Open_Update).grid(column=3+self.COL_START, row=3+self.ROW_START, sticky=W)
        
        # Organization
        ttk.Label(mainframe, text="Inputs:").grid(column=1+self.COL_START, row=1+self.ROW_START, sticky=E)
        ttk.Label(mainframe, text="Output:").grid(column=1+self.COL_START, row=4+self.ROW_START, sticky=E)

        # Input Entry
        ttk.Label(mainframe, text="Frequency (Hz):").grid(column=1+self.COL_START, row=2+self.ROW_START, sticky=E)
        ttk.Label(mainframe, text="Percent Open (%):").grid(column=1+self.COL_START, row=3+self.ROW_START, sticky=E)
        
        # Current DC
        ttk.Label(mainframe, textvariable=self.Duty_Cycle_Disp).grid(column=2+self.COL_START, row=5+self.ROW_START, sticky=(W, E))
        ttk.Label(mainframe, text="Duty Cycle (%):").grid(column=1+self.COL_START, row=5+self.ROW_START, sticky=E)

        # Current DIO
        ttk.Label(mainframe, textvariable=self.DIO_Value_Disp).grid(column=2+self.COL_START, row=6+self.ROW_START, sticky=(W, E))
        ttk.Label(mainframe, text="DIO State:").grid(column=1+self.COL_START, row=6+self.ROW_START, sticky=E)

        # Initialize Defaults
        self.Current_DIO_State()
        
    
    def Pullup_Conversion(self, value):
        if value == 1:
            return "HIGH"
        elif value == 0:
            return "LOW"
        else:
            print("Invalid Input")
            return "INVALID RESULT"
    
    def Percent_Open_Update(self):
        self.Percent_Open = float(self.Percent_Open_Disp.get())
        Throttle_PID_CLASS.PID_Update(TPSA_CLASS.Inverse_Calibration_Formula(float(self.Percent_Open)))
        
        global Update_Throttle_PID
        Update_Throttle_PID = True

    def Freq_Update(self):
        self.Frequency = float(self.Frequency_Disp.get())
        self.PWM_Update(self, "Freq")
    
        global Update_Throttle_PID
        Update_Throttle_PID = True
    
    def PWM_Update(self):
        try:            
            
            """
            PWM Works by setting the freq and duty
            DIO#_EF_ENABLE: 0 = Disable, 1 = Enable
            DIO#_EF_INDEX: 0 <- Specifies PWM mode
            DIO#_EF_CONFIG_A: When the specified clocks source's count matches this value, the line will transition from high to low.
            DIO#_EF_OPTIONS: Bits 0-2 specify which clock source to use ... b000 for Clock0, b001 for
            Clock1, and b010 for Clock2. All other bits are reserved and should be set to 0.
                CLOCK0: 32-bit
                CLOCK1: 16-bit
                CLOCK2: 16-bit
            Clock setup (Example here is 10Hz):
            DIO_EF_CLOCK0_ENABLE = 0
            DIO_EF_CLOCK0_DIVISOR = 16 # 100 MHz / 16 = 6.25MHz
            DIO_EF_CLOCK0_ROLL_VALUE = 625000
            DIO_EF_CLOCK0_ENABLE = 1
            """
            
            self.Duty_Cycle = self.DC_Check(self.Duty_Cycle)

            Desired_Clock_Divisor = self.CLOCK_SPEED/self.Frequency # Find Clock Dividor for desired freq

            Clock_Divider_Setting = self.find_largest(self.CLOCK_DVISORS, Desired_Clock_Divisor) # Find the biggeest one

            Clock_Roll = int((self.CLOCK_SPEED/Clock_Divider_Setting)/self.Frequency) # Find clock roll for the desired clock divider chosen

            PWM_ROLLOVER = int((self.Duty_Cycle/100)*Clock_Roll) # Find the Duty Cycle Roll value

            #print(Clock_Divider_Setting)
            #print(Clock_Roll)
            #print(DIO_EF_CONFIG_A_PWM_ROLLOVER)


            ljm.eWriteName(handle, "DIO_EF_CLOCK2_ENABLE", 0) #Disables clock to change clock properties
            ljm.eWriteName(handle, "DIO_EF_CLOCK2_DIVISOR", Clock_Divider_Setting) # 100 MHz Core Clock / 16 = 6.25MHz
            ljm.eWriteName(handle, "DIO_EF_CLOCK2_ROLL_VALUE", Clock_Roll) # Set roll to 0.625M, so repeats every 10Hz
            ljm.eWriteName(handle, "DIO_EF_CLOCK2_ENABLE", 1) # Clock Re-enable
            
            ljm.eWriteName(handle, "DIO"+str(self.DIO_NUM_PWM)+"_EF_ENABLE", 0) # Disable before changing, 0 = Disable, 1 = Enable PWM
            ljm.eWriteName(handle, "DIO"+str(self.DIO_NUM_PWM)+"_EF_INDEX", 0) # 0 <- Specifies PWM mode
            ljm.eWriteName(handle, "DIO"+str(self.DIO_NUM_PWM)+"_EF_CONFIG_A", PWM_ROLLOVER) #When the specified clocks source's count matches this value
            ljm.eWriteName(handle, "DIO"+str(self.DIO_NUM_PWM)+"_EF_OPTIONS", 0b010) # Bits 0-2 specify which clock source to use ... b000 for Clock0, b001 for Clock1 etc...
            ljm.eWriteName(handle, "DIO"+str(self.DIO_NUM_PWM)+"_EF_ENABLE", 1) #0 = Disable, 1 = Enable PWM

        except ValueError:
            print("Invalid Input")

    def Disp_Update_Inputs(self):
        self.Frequency_Disp.set(round(self.Frequency, 2))
        self.Percent_Open_Disp.set(round(self.Percent_Open, 2))
        
    def Disp_Update(self):
        self.Duty_Cycle_Disp.set(round(self.Duty_Cycle, 2))
        self.DIO_Value_Disp.set(self.Pullup_Conversion(self.DIO_Value))

class Pressure_Sensor:

    def __init__(self, mainframe, Serial_Init, Sensor_Num, **kwargs):
        
        # Position
        self.mainframe = mainframe
        self.ROW_START = kwargs.get("ROW", 0)
        self.COL_START = kwargs.get("COL", 0)
        
        self.Unit = kwargs.get("Unit", "kPa")

        self.Label = kwargs.get("Label", "Pressure")

        self.Sensor_Num = Sensor_Num # This is the number written in the bottom left of the diaplay

        if not isinstance(Serial_Init, type(None)):
            self.Connected = True
        else:
            self.Connected = False

        self.Serial_Init = Serial_Init

        self.Pressure_Reading = 0

        self.Pressure_Reading_Disp = StringVar()

        # Organization / Labels
        ttk.Label(mainframe, text=self.Label+": ("+self.Unit+")").grid(column=1+self.COL_START, row=0+self.ROW_START, sticky=E)

        # Readouts
        ttk.Label(mainframe, textvariable=self.Pressure_Reading_Disp).grid(column=2+self.COL_START, row=0+self.ROW_START, sticky=(W, E))

        # Initialize Defaults
        if self.Connected:
            self.Get_Pressure()
    
    def CRC_Calculation (self, input):
        crc16_func = crcmod.Crc(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000) #0x18005 0x1A001

        crc16_func.update(bytearray((input)))

        res = crc16_func.crcValue

        byte1, byte2 = res.to_bytes(2, 'little')

        #print(hex(byte1), hex(byte2)) #0x25CF

        return [byte1, byte2]
        
    def Command_CRC_Calculation (self, input):
        
        byte1, byte2 = self.CRC_Calculation(input)
        
        command = input + [byte1, byte2]

        #print(command)

        return command


    def CRC_Verification (self, recieved_data):

        byte1, byte2 = self.CRC_Calculation(recieved_data[:-2])

        if byte1 == recieved_data[-2] and byte2 == recieved_data[-1]:
            return True
        else:
            return False

    def Read_Write_Command (self, Sensor, Command, Return_Bytes): 
        """
        Sensor is the Serial Class of the sensor, Command is the command to send to it without the CRC, Return_Bytes is the number of bytes to read


        # Write Format:
                # Address,  Function Code, Starting Address (2 byte), Number of Words to Read (2 bytes), CRC16 (2 Bytes)
        #wrt = s.write([0x01,        0x03,         0x00, 0x16,                 0x00, 0x02,                     0x25, 0xCF])

        # Response Format
                # Address,  Function Code, Data Length,          Data,      CRC16 (2 Bytes)
        #wrt = s.write([0x01,        0x03,         0x02,          0x00, 0x02,     0x25, 0xCF])

        # Current Device settings:
            # Pairity = None
            # Baud = 9600
            # Slave Address = 1
        """

        command_CRC = self.Command_CRC_Calculation(Command)
        
        CRC_Verified = False

        attempt = 0

        while (not CRC_Verified) or (attempt > 3):
            wrt = Sensor.write(command_CRC)

            res = Sensor.read(Return_Bytes)
            #print("Pressure Sensor: ", res)

            if len(res) == Return_Bytes:
            
                try:
                    CRC_Verified = self.CRC_Verification(res)
                except:
                    print("Unknown Error - Data may be incorrect")
                    #CRC_Verified = True
            
            else:
                CRC_Verified = False

            if not CRC_Verified:
                print("Pressure Data not recived correctly, trying again, this can be from no power to the sensor, or from the incorrect COM port or Sensor Address Specified\nNote: Sensor Address is specified on the bottom of the display on the sensor")
                #CRC_Verified = True

            attempt += 1

        return res
    
    def Get_Pressure (self):
        """
        Uses Read_Write_Command, with the read pressure command on the specified Sensor to read the pressure and returns it as a float
        """

        command = [self.Sensor_Num, 0x03, 0x00, 0x16, 0x00, 0x02]
        
        result = self.Read_Write_Command (self.Serial_Init, command, 9)

        self.Pressure_Reading = struct.unpack('>f', result[3:-2])[0]
    
    def Set_Pressure(self):
        self.Pressure_Reading_Disp.set(round(self.Pressure_Reading, 3))
    
    def Calibrate_Pressure(self, Calibration_Value):
        """ Changes the 0 offset value, IE what pressure the pressure sensor reads as 0
        """
        print('Functionality not Working Right now')

        """

        if (Calibration_Value < -19999) or (Calibration_Value > 99999):
            print("Valid Range of calibration is from -19999-99999 only")
        
        else:
            #bytelist = list(int(Calibration_Value).to_bytes(length=2, byteorder="big")) #, Byte_3, Byte_4

            bytelist = list(struct.pack('f', Calibration_Value))

            command = [self.Sensor_Num, 0x06, 0x00, 0x18, 0x00, 0x04] + bytelist

            result = self.Read_Write_Command (self.Serial_Init, command, 8)
        """
            
    def Change_Slave_Address(self, slave_address):
        """ Changes the Address the Command are sent to
        In RS485, each device has its own slave address so the slave address in the message being sent has to match the slave address of the device
        """
        self.Sensor_Num = slave_address
        print(slave_address)
        print(self.Sensor_Num)
        

class CO2_Meter:

    def __init__(self, mainframe, Serial_Init, **kwargs):
        
        # Position
        self.mainframe = mainframe
        self.ROW_START = kwargs.get("ROW", 0)
        self.COL_START = kwargs.get("COL", 0)

        self.Label = kwargs.get("Label", "Sensor:")
        self.Label_CO2 = kwargs.get("Label_CO2", "CO2")
        self.Label_VOC = kwargs.get("Label_VOC", "VOC")
        self.Label_PM25 = kwargs.get("Label_PM25", "PM2.5")
        self.Label_RH = kwargs.get("Label_RH", "RH")
        self.Label_Temp = kwargs.get("Label_Temp", "Temp")
        
        if not isinstance(Serial_Init, type(None)):
            self.Connected = True
        else:
            self.Connected = False

        self.Serial_Init = Serial_Init

        self.CO2_Reading = 0
        self.VOC_Reading = 0
        self.PM25_Reading = 0
        self.RH_Reading = 0
        self.Temp_Reading = 0

        self.CO2_Reading_Disp = StringVar()
        self.VOC_Reading_Disp = StringVar()
        self.PM25_Reading_Disp = StringVar()
        self.RH_Reading_Disp = StringVar()
        self.Temp_Reading_Disp = StringVar()

        # Organization / Labels
        ttk.Label(mainframe, text=self.Label).grid(column=1+self.COL_START, row=0+self.ROW_START, sticky=W)
        ttk.Label(mainframe, text=self.Label_CO2+": (ppm)").grid(column=1+self.COL_START, row=1+self.ROW_START, sticky=E)
        ttk.Label(mainframe, text=self.Label_VOC+": (Level)").grid(column=1+self.COL_START, row=2+self.ROW_START, sticky=E)
        ttk.Label(mainframe, text=self.Label_PM25+": (ug/m^3)").grid(column=1+self.COL_START, row=3+self.ROW_START, sticky=E)
        ttk.Label(mainframe, text=self.Label_RH+": (%)").grid(column=1+self.COL_START, row=4+self.ROW_START, sticky=E)
        ttk.Label(mainframe, text=self.Label_Temp+": (°C)").grid(column=1+self.COL_START, row=5+self.ROW_START, sticky=E)

        # Readouts
        ttk.Label(mainframe, textvariable=self.CO2_Reading_Disp).grid(column=2+self.COL_START, row=1+self.ROW_START, sticky=(W, E))
        ttk.Label(mainframe, textvariable=self.VOC_Reading_Disp).grid(column=2+self.COL_START, row=2+self.ROW_START, sticky=(W, E))
        ttk.Label(mainframe, textvariable=self.PM25_Reading_Disp).grid(column=2+self.COL_START, row=3+self.ROW_START, sticky=(W, E))
        ttk.Label(mainframe, textvariable=self.RH_Reading_Disp).grid(column=2+self.COL_START, row=4+self.ROW_START, sticky=(W, E))
        ttk.Label(mainframe, textvariable=self.Temp_Reading_Disp).grid(column=2+self.COL_START, row=5+self.ROW_START, sticky=(W, E))

        # Initialize Defaults
        if self.Connected:
            self.Get_Data()
    
    def CS_Calculation (self, input):
        """ Checksum calculation: Cumulative sum of data = 256 - (HEAD+LEN+CMD+DATA)
        """
        CS = 10*256
        
        for data in input:
            CS = CS - data

        #print(CS & 255)

        return CS & 255
        
    def Command_CS_Calculation (self, input):
        
        CheckSum = self.CS_Calculation(input)
        
        command = input + [CheckSum]

        #print(command)

        return command


    def CS_Verification (self, recieved_data):

        CheckSum = self.CS_Calculation(recieved_data[:-1])

        #print(hex(CheckSum))

        if CheckSum == recieved_data[-1]:
            return True
        else:
            return False

    def Read_Write_Command (self, Sensor, Command, Return_Bytes): 
        command_CS = self.Command_CS_Calculation (Command)

        CS_Verified = False

        attempt = 0

        while (not CS_Verified) or (attempt > 3):
            wrt = Sensor.write(command_CS)

            res = Sensor.read(Return_Bytes)
            
            #print(res)

            if len(res) == Return_Bytes:
                try:
                    CS_Verified = self.CS_Verification(res)
                except:
                    CS_Verified = True
                    print("Unknown Error - Data may be incorrect")

            else:
                CS_Verified = False
            
            if not CS_Verified:
                print("CO2 Sensor Data not recived correctly, trying again")
                
            attempt += 1

        return res


    def Calibrate_CO2(self, Calibration_Value):
        """ Calibrates the CO2 Sensor to the set value
        """
        
        print(Calibration_Value)


        if (Calibration_Value < 400) or (Calibration_Value > 1500):
            print("Valid Range of calibration is from 400-1500 only")
        
        else:
            Byte_1 = int(Calibration_Value/256)
            Byte_2 = Calibration_Value % 256

            command = [0x11, 0x03, 0x03, Byte_1, Byte_2]

            res = self.Read_Write_Command (self.Serial_Init, command, 4)

            print(res)

            if list(res) == [0x16, 0x01, 0x03, 0xE6]:
                print('CO2 Calibration Sucessful')

            else:
                print('CO2 Calibration Failed')

    def Get_Data (self):
        """
        Uses Read_Write_Command, with the read all things, CO2, VOC, RH, Temp, PM25 on the specified Sensor
        """
        command = [0x11, 0x02, 0x01, 0x01]

        res = self.Read_Write_Command (self.Serial_Init, command, 25)

        self.CO2_Reading = struct.unpack('>H', res[3:5])[0] # 0-5000ppm

        self.VOC_Reading  = struct.unpack('>H', res[5:7])[0] # 0->3 3 being lots of VOC, 0 being no VOC

        RH_Raw = struct.unpack('>H', res[7:9])[0] # Percentage humidity (5->99%)

        self.RH_Reading = RH_Raw/10

        Temp_Raw = struct.unpack('>H', res[9:11])[0] #300 ~ 1,200 -> -20 ~ 70℃ -> Use linear interpolation

        self.Temp_Reading = (Temp_Raw - 500)/10 #(70-(-20))/(1200-300) * (Temp_Raw-300) + (-20)

        self.PM25_Reading = struct.unpack('>H', res[19:21])[0] #0 ~ 1,000ug/m^3

    def Set_Data(self):
        self.CO2_Reading_Disp.set(self.CO2_Reading)
        self.VOC_Reading_Disp.set(self.VOC_Reading)
        self.RH_Reading_Disp.set(round(self.RH_Reading, 1))
        self.Temp_Reading_Disp.set(round(self.Temp_Reading, 2))
        self.PM25_Reading_Disp.set(self.PM25_Reading)

class Recorder:
    def __init__(self, mainframe, Recording_Rate):
        
        self.Recording = StringVar()

        self.Recording_Rate = Recording_Rate

        self.Recording.set('Record')

        self.Thread_TextWrite_Running = False
        
        # Initializing Buttons, Textvariables (Display Variables, which change as thier values do) and Static Text
        ttk.Button(mainframe, textvariable=self.Recording, command=self.start_stop_recording).place(relx=0.5, rely=0.5, anchor=CENTER)
        ttk.Label(mainframe, text='Press Button to Record Data: ').grid(column=0, row=0, sticky=W)

    def write_text(self):
        """ Writes the current data to a file when called
        """

        while self.Thread_TextWrite_Running:
            
            data_to_write = [
                (time.time() - self.t_start),
                PD_CLASS.Motor_Dir, PD_CLASS.Motor_Enable, PD_CLASS.RPM, PD_CLASS.Frequency, PD_CLASS.Switch_Position, PD_CLASS.Duty_Cycle, PD_CLASS.DIO_Value, 
                MAF_CLASS.AIN_Value, MAF_CLASS.Converted_AIN,
                TPSA_CLASS.AIN_Value, Throttle_Motor_CLASS.Percent_Open, Throttle_Motor_CLASS.Frequency, Throttle_Motor_CLASS.DIO_Value, Throttle_Motor_CLASS.Duty_Cycle,
                Thermocouple_Reactor_CLASS.AIN_Value, Thermocouple_Reactor_CLASS.Converted_AIN, Thermocouple_Post_HX_CLASS.AIN_Value, Thermocouple_Post_HX_CLASS.Converted_AIN, Thermocouple_Pre_HX_CLASS.AIN_Value, Thermocouple_Pre_HX_CLASS.Converted_AIN,
                Gas_Analyzer_NOX_CLASS.AIN_Value, Gas_Analyzer_NOX_CLASS.Converted_AIN, Gas_Analyzer_CO_CLASS.AIN_Value, Gas_Analyzer_CO_CLASS.Converted_AIN, Gas_Analyzer_O2_CLASS.AIN_Value, Gas_Analyzer_O2_CLASS.Converted_AIN,
                Pressure_Pre_Filter_CLASS.Pressure_Reading, Pressure_Post_Filter_CLASS.Pressure_Reading,
                CO2_Atmosphere_CLASS.CO2_Reading, CO2_Atmosphere_CLASS.VOC_Reading, CO2_Atmosphere_CLASS.PM25_Reading, CO2_Atmosphere_CLASS.RH_Reading, CO2_Atmosphere_CLASS.Temp_Reading,
                CO2_Reactor_CLASS.CO2_Reading, CO2_Reactor_CLASS.VOC_Reading, CO2_Reactor_CLASS.PM25_Reading, CO2_Reactor_CLASS.RH_Reading, CO2_Reactor_CLASS.Temp_Reading,
                Relay_LB_CLASS.DIO_Value, Relay_PD_CLASS.DIO_Value
            ]
            
            f = open(self.file_name, "a")
            f.write('\t'.join(str(x) for x in data_to_write) + '\n')
            f.close()

            time.sleep(1/self.Recording_Rate - (time.time() % (1/self.Recording_Rate)))

    def start_stop_recording(self):
        if self.Thread_TextWrite_Running: #If the thread is running
            self.Thread_TextWrite_Running = False
            self.Recording.set('Record')

        else: # If the thread is not running start it and therefore the recording
            date_file_name = str(datetime.datetime.now()).split(".")[0].replace(":", "_") # Name stuff
            self.file_name = "Recording " + date_file_name + ".txt"
            header = [
                        'Time_Elapsed',
                        'PD_Dir', 'PD_Enable', 'PD_RPM', 'PD_Frequency', 'PD_Switch_Position', 'PD_Duty_Cycle', 'PD_DIO', 
                        'MAF_Voltage', 'MAF',
                        'T_TPSA_Voltage', 'T_Percent_Open', 'T_Frequency', 'T_DIO', 'T_Duty_Cycle',
                        'Reactor_Thermocouple_Voltage', 'Reactor_Thermocouple', 'Post_HX_Thermocouple_Voltage', 'Post_HX_Thermocouple', 'Pre_HX_Thermocouple_Voltage', 'Pre_HX_Thermocouple',
                        'NOX_Voltage', 'NOX', 'CO_Voltage', 'CO', 'O2_Voltage', 'O2',
                        'Pre_Filter_Pressure', 'Post_Filter_Pressure',
                        'Atmospheric_CO2', 'Atmospheric_VOC', 'Atmospheric_PM2.5', 'Atmospheric_RH', 'Atmospheric_Temp',
                        'Reactor_CO2', 'Reactor_VOC', 'Reactor_PM2.5', 'Reactor_RH', 'Reactor_Temp',
                        'LB_Relay', 'PD_Relay'
                    ]
            
            self.Recording.set('Recording')

            self.Thread_TextWrite_Running = True

            thread_Textwrite = threading.Thread(target=self.write_text, daemon=True)
            self.t_start = time.time()
            thread_Textwrite.start()
            

            # Create file to start recording data into
            f = open(self.file_name, "w")
            f.write('\t'.join(str(x) for x in header) + '\n')
            f.close()

class Settings:
    def __init__(self, mainframe):
        
        # Initializing Buttons, Textvariables (Display Variables, which change as thier values do) and Static Text
        ttk.Label(mainframe, text='Pre Filter Pressure Sensor').grid(column=0, row=0, sticky=W)
        ttk.Button(mainframe, text='Settings', command=self.Settings_Pre_Filter_Pressure).grid(column=1, row=0, sticky=EW)

        ttk.Label(mainframe, text='Post Filter Pressure Sensor').grid(column=0, row=1, sticky=W)
        ttk.Button(mainframe, text='Settings', command=self.Settings_Post_Filter_Pressure).grid(column=1, row=1, sticky=EW)

        ttk.Label(mainframe, text='Atmospheric CO2 Sensor').grid(column=0, row=2, sticky=W)
        ttk.Button(mainframe, text='Settings', command=self.Settings_Atmospheric_CO2).grid(column=1, row=2, sticky=EW)

        ttk.Label(mainframe, text='Reactor CO2 Sensor').grid(column=0, row=3, sticky=W)
        ttk.Button(mainframe, text='Settings', command=self.Settings_Reactor_CO2).grid(column=1, row=3, sticky=EW)

        ttk.Label(mainframe, text='Sampling Rates').grid(column=0, row=4, sticky=W)
        ttk.Button(mainframe, text='Settings', command=self.Settings_Sampling_Rates).grid(column=1, row=4, sticky=EW)

        ttk.Label(mainframe, text='PID Controller').grid(column=0, row=5, sticky=W)
        ttk.Button(mainframe, text='Settings', command=self.Settings_PID).grid(column=1, row=5, sticky=EW)
        
    def Settings_PID(self):
        popup = Toplevel(root)
        popup.geometry("500x250")
        popup.title("PID Settings")
        
        kp_Disp = StringVar()
        ki_Disp = StringVar()
        kd_Disp = StringVar()

        kp_Disp.set(Throttle_PID_CLASS.kp)
        ki_Disp.set(Throttle_PID_CLASS.ki)
        kd_Disp.set(Throttle_PID_CLASS.kd)

        # kp Update
        ttk.Label(popup, text='Proportional Constant (kp): ').grid(column=0, row=0, sticky=W)
        ttk.Entry(popup, textvariable=kp_Disp).grid(column=1, row=0, sticky=W)
        ttk.Button(popup, text='Update', command= lambda: self.Update_kp(kp_Disp)).grid(column=2, row=0, sticky=EW)        

        # ki Update
        ttk.Label(popup, text='Integral Constant (ki): ').grid(column=0, row=1, sticky=W)
        ttk.Entry(popup, textvariable=ki_Disp).grid(column=1, row=1, sticky=W)
        ttk.Button(popup, text='Update', command= lambda: self.Update_ki(ki_Disp)).grid(column=2, row=1, sticky=EW)        

        # kd Update
        ttk.Label(popup, text='Derivative Constant (kd): ').grid(column=0, row=2, sticky=W)
        ttk.Entry(popup, textvariable=kd_Disp).grid(column=1, row=2, sticky=W)
        ttk.Button(popup, text='Update', command= lambda: self.Update_kd(kd_Disp)).grid(column=2, row=2, sticky=EW)        
    
    def Update_kp(self, kp_Disp):
        Throttle_PID_CLASS.kp = float(kp_Disp.get())

    def Update_ki(self, ki_Disp):
        Throttle_PID_CLASS.ki = float(ki_Disp.get())

    def Update_kd(self, kd_Disp):
        Throttle_PID_CLASS.kd = float(kd_Disp.get())

    def Settings_Sampling_Rates(self):
        global LabJack_Sampling_Rate, Sampling_Rate_PID, Disp_Update_Rate, Text_Recording_Rate

        popup = Toplevel(root)
        popup.geometry("500x250")
        popup.title("Sampling Rate Settings")
        
        LabJack_Sampling_Rate_Disp = StringVar()
        Sampling_Rate_PID_Disp = StringVar()
        Disp_Update_Rate_Disp = StringVar()
        Text_Recording_Rate_Disp = StringVar()

        LabJack_Sampling_Rate_Disp.set(LabJack_Sampling_Rate)
        Sampling_Rate_PID_Disp.set(Sampling_Rate_PID)
        Disp_Update_Rate_Disp.set(Disp_Update_Rate)
        Text_Recording_Rate_Disp.set(Text_Recording_Rate)

        # Labjack Sampling Rate
        ttk.Label(popup, text='LabJack Sampling Rate: ').grid(column=0, row=0, sticky=W)
        ttk.Entry(popup, textvariable=LabJack_Sampling_Rate_Disp).grid(column=1, row=0, sticky=W)
        ttk.Button(popup, text='Update', command= lambda: self.Update_LabJack_Sampling_Rate(LabJack_Sampling_Rate_Disp)).grid(column=2, row=0, sticky=EW)        

        # PID Sampling Rate
        ttk.Label(popup, text='PID Sampling Rate: ').grid(column=0, row=1, sticky=W)
        ttk.Entry(popup, textvariable=Sampling_Rate_PID_Disp).grid(column=1, row=1, sticky=W)
        ttk.Button(popup, text='Update', command= lambda: self.Update_PID_Sampling_Rate(Sampling_Rate_PID_Disp)).grid(column=2, row=1, sticky=EW)        

        # GUI Display Update
        ttk.Label(popup, text='GUI Display Refresh Rate: ').grid(column=0, row=2, sticky=W)
        ttk.Entry(popup, textvariable=Disp_Update_Rate_Disp).grid(column=1, row=2, sticky=W)
        ttk.Button(popup, text='Update', command= lambda: self.Update_Disp_Refresh_Rate(Disp_Update_Rate_Disp)).grid(column=2, row=2, sticky=EW)        

        # Recording Rate
        ttk.Label(popup, text='Data Recording Rate: ').grid(column=0, row=3, sticky=W)
        ttk.Entry(popup, textvariable=Text_Recording_Rate_Disp).grid(column=1, row=3, sticky=W)
        ttk.Button(popup, text='Update', command= lambda: self.Update_Recording_Rate(Text_Recording_Rate_Disp)).grid(column=2, row=3, sticky=EW)        

    def Update_Recording_Rate(self, Text_Recording_Rate_Disp):
        Recorder_CLASS.Recording_Rate = int(Text_Recording_Rate_Disp.get())
    
    def Update_Disp_Refresh_Rate(self, Disp_Update_Rate_Disp):
        global Disp_Update_Rate
        Disp_Update_Rate = int(Disp_Update_Rate_Disp.get())

    def Update_PID_Sampling_Rate(self, Sampling_Rate_PID_Disp):
        Throttle_PID_CLASS.dt = 1/float(Sampling_Rate_PID_Disp.get())

    def Update_LabJack_Sampling_Rate(self, LabJack_Sampling_Rate_Disp):
        ljm.eWriteName(handle, "AIN_SAMPLING_RATE_HZ", int(LabJack_Sampling_Rate_Disp.get()))
        Actual_Sampling_Rate = ljm.eReadName(handle, "AIN_SAMPLING_RATE_ACTUAL_HZ")
        LabJack_Sampling_Rate_Disp.set(round(Actual_Sampling_Rate, 2))

    def Settings_Pre_Filter_Pressure(self):
        Pressure_Class = Pressure_Pre_Filter_CLASS
        self.Settings_Pressure(Pressure_Class)

    def Settings_Post_Filter_Pressure(self):
        Pressure_Class = Pressure_Post_Filter_CLASS
        self.Settings_Pressure(Pressure_Class)

    def Settings_Atmospheric_CO2(self):
        CO2_Class = CO2_Atmosphere_CLASS
        self.Settings_CO2(CO2_Class)

    def Settings_Reactor_CO2(self):
        CO2_Class = CO2_Reactor_CLASS
        self.Settings_CO2(CO2_Class)
    
    def Settings_Pressure(self, Pressure_Class):
        
        popup = Toplevel(root)
        popup.geometry("500x250")
        popup.title("Pressure Sensor Settings")
        
        COM_Disp = StringVar()
        Pres_Cal_Disp = StringVar()
        Slave_Address_Disp = StringVar()

        if not isinstance(Pressure_Class.Serial_Init, type(None)):
            COM_Disp.set(Pressure_Class.Serial_Init.port)
        else:
            COM_Disp.set('Disconnected')
        
        Pres_Cal_Disp.set(Pressure_Class.Pressure_Reading)
        Slave_Address_Disp.set(Pressure_Class.Sensor_Num)

        # Com Port Update
        ttk.Label(popup, text='COM Port: ').grid(column=0, row=0, sticky=W)
        ttk.Entry(popup, textvariable=COM_Disp).grid(column=1, row=0, sticky=W)
        ttk.Button(popup, text='Update', command= lambda: self.Update_COM(COM_Disp, Pressure_Class)).grid(column=2, row=0, sticky=EW)

        # Calibrate Pressure
        ttk.Label(popup, text='Calibrate Pressure: ').grid(column=0, row=1, sticky=W)
        ttk.Entry(popup, textvariable=Pres_Cal_Disp).grid(column=1, row=1, sticky=W)
        ttk.Button(popup, text='Update', command= lambda: self.Update_Calibration_Pressure(Pres_Cal_Disp, Pressure_Class)).grid(column=2, row=1, sticky=EW)

        # Change Slave Address
        ttk.Label(popup, text='RS485 Slave Address: ').grid(column=0, row=2, sticky=W)
        ttk.Entry(popup, textvariable=Slave_Address_Disp).grid(column=1, row=2, sticky=W)
        ttk.Button(popup, text='Update', command= lambda: self.Update_Slave_Address(Slave_Address_Disp, Pressure_Class)).grid(column=2, row=2, sticky=EW)


    def Settings_CO2(self, CO2_Class):
        popup = Toplevel(root)
        popup.geometry("500x250")
        popup.title("CO2 Meter Settings")
        
        COM_Disp = StringVar()
        CO2_Cal_Disp = StringVar()

        if not isinstance(CO2_Class.Serial_Init, type(None)):
            COM_Disp.set(CO2_Class.Serial_Init.port)
        else:
            COM_Disp.set('Disconnected')

        CO2_Cal_Disp.set(CO2_Class.CO2_Reading)

        # Com Port Update
        ttk.Label(popup, text='COM Port: ').grid(column=0, row=0, sticky=W)
        ttk.Entry(popup, textvariable=COM_Disp).grid(column=1, row=0, sticky=W)
        ttk.Button(popup, text='Update', command= lambda: self.Update_COM(COM_Disp, CO2_Class)).grid(column=2, row=0, sticky=EW)

        # Calibrate CO2
        ttk.Label(popup, text='Calibrate CO2: ').grid(column=0, row=1, sticky=W)
        ttk.Entry(popup, textvariable=CO2_Cal_Disp).grid(column=1, row=1, sticky=W)
        ttk.Button(popup, text='Update', command= lambda: self.Update_Calibration_CO2(CO2_Cal_Disp, CO2_Class)).grid(column=2, row=1, sticky=EW)

    def Update_COM(self, COM_Disp, Class):
        Class.Serial_Init.close()
        Class.Serial_Init, Class.Connected = Connect_Serial(COM_Disp.get(), Class.Label)
        

    def Update_Calibration_CO2(self, CO2_Cal_Disp, CO2_Class):
        print("Currently Not Working")
        #CO2_Class.Calibrate_CO2(int(CO2_Cal_Disp.get()))
        #CO2_Atmosphere_CLASS.Calibrate_CO2(550)

    def Update_Calibration_Pressure(self, Pres_Cal_Disp, Pressure_Class):
        Pressure_Class.Calibrate_Pressure(float(Pres_Cal_Disp.get()))
    
    def Update_Slave_Address(self, Slave_Address_Disp, Pressure_Class):
        Pressure_Class.Change_Slave_Address(int(Slave_Address_Disp.get()))

def Connect_Serial(COM='COM3', Name='CO2 Sensor / Pressure Sensor'):
    try:
        Connected_Serial = serial.Serial(COM, baudrate=9600, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=1)
        print(Name + " Connected")
        Connected = True
    except:
        print(Name + " not found, please connect and check COM port number and modify to match")
        Connected = False
        Connected_Serial = None

    return Connected_Serial, Connected

def Get_Serial():
    if Pressure_Pre_Filter_CLASS.Connected:
        Pressure_Pre_Filter_CLASS.Get_Pressure()
        
    if Pressure_Post_Filter_CLASS.Connected:
        Pressure_Post_Filter_CLASS.Get_Pressure()
        
    if CO2_Atmosphere_CLASS.Connected:
        CO2_Atmosphere_CLASS.Get_Data()
        
    if CO2_Reactor_CLASS.Connected:
        CO2_Reactor_CLASS.Get_Data()

def Update_Disp():
    """
    Updates all the values in the display
    This is run on a seperate thread to increase speed of operation
    """
    #TPSB_CLASS.Get_Current_AIN()

    #TPSA_CLASS.Get_Current_AIN()
    """
    Thermocouple_Reactor_CLASS.Get_Current_AIN()
    Thermocouple_Pre_HX_CLASS.Get_Current_AIN()
    Thermocouple_Post_HX_CLASS.Get_Current_AIN()
    Gas_Analyzer_NOX_CLASS.Get_Current_AIN()
    Gas_Analyzer_O2_CLASS.Get_Current_AIN()
    Gas_Analyzer_CO_CLASS.Get_Current_AIN()
    MAF_CLASS.Get_Current_AIN()
    """

    global Update_PD, Update_Throttle_PID, Serial_Get

    TPSA_CLASS.Set_Current_AIN()
    Thermocouple_Reactor_CLASS.Set_Current_AIN()
    Thermocouple_Pre_HX_CLASS.Set_Current_AIN()
    Thermocouple_Post_HX_CLASS.Set_Current_AIN()
    Gas_Analyzer_NOX_CLASS.Set_Current_AIN()
    Gas_Analyzer_O2_CLASS.Set_Current_AIN()
    Gas_Analyzer_CO_CLASS.Set_Current_AIN()
    MAF_CLASS.Set_Current_AIN()
    Throttle_Motor_CLASS.Disp_Update()
    PD_CLASS.Disp_Update()

    if Update_PD:
        PD_CLASS.Disp_Update_Inputs()
        Update_PD = False
    if Update_Throttle_PID:
        Throttle_Motor_CLASS.Disp_Update_Inputs()
        Update_Throttle_PID = False
    
    if Pressure_Pre_Filter_CLASS.Connected:
        Pressure_Pre_Filter_CLASS.Set_Pressure()
    if Pressure_Post_Filter_CLASS.Connected:
        Pressure_Post_Filter_CLASS.Set_Pressure()
    if CO2_Atmosphere_CLASS.Connected:
        CO2_Atmosphere_CLASS.Set_Data()
    if CO2_Reactor_CLASS.Connected:
        CO2_Reactor_CLASS.Set_Data()
    
    root.after(int((1/Disp_Update_Rate)*1000), Update_Disp)

    thread_Serial = threading.Thread(target=Get_Serial, daemon=True)
    thread_Serial.start()

def scheduled_update_PID():
    while True:
        Throttle_PID_CLASS.control_mainloop()

        #time.sleep(max(1/Sampling_Rate_PID - (time.time()-t_now), 0))
        time.sleep(1/Sampling_Rate_PID - (time.time() % (1/Sampling_Rate_PID)))

def on_closing():
    close = messagebox.askyesno("Exit?", "Are you sure you want to exit?")
    
    if close:
        root.destroy()
        ljm.eWriteName(handle, "IO_CONFIG_SET_CURRENT_TO_DEFAULT", 1)
        ljm.close(handle)
        print("Closed LabJack Connection")

#LabJack Setup
handle = ljm.openS("T8", "ANY", "ANY")  # T8 device, Any connection, Any identifier

ljm.eWriteName(handle, "DIO_EF_CLOCK0_ENABLE", 0) #Disables clock to change clock properties

#Pressure Sensor Setup
"""
To find what COM port to use go into device manager on the computer and look at the Ports(COM & LPT) drop down for the COM port 
numbers of the devices, they show up as USB-SERIAL CH340 (COMX), where the X is the COM port
"""

# Triggers for when to update the data in PD and Throttle Respectively
Update_PD = False
Update_Throttle_PID = False
Serial_Get = False

Expected_COM = {
    'CO2 Atmosphere':'COM5', 
    'CO2 Reactor':'COM6', 
    'Pressure Pre Filter':'COM3',#CH340? 
    'Pressure Post Filter':'COM4'#CH340? 
}

CO2_Reactor, _ = Connect_Serial(Expected_COM.get('CO2 Reactor'), 'CO2 Reactor')
CO2_Atmosphere, _ = Connect_Serial(Expected_COM.get('CO2 Atmosphere'), 'CO2 Atmosphere')
Pressure_Pre_Filter, _ = Connect_Serial(Expected_COM.get('Pressure Pre Filter'), 'Pressure Pre Filter')
Pressure_Post_Filter, _ = Connect_Serial(Expected_COM.get('Pressure Post Filter'), 'Pressure Post Filter')

info = ljm.getHandleInfo(handle)
print("Opened a LabJack with Device type: %i, Connection type: %i,\n"
      "Serial number: %i, IP address: %s, Port: %i,\nMax bytes per MB: %i" %
      (info[0], info[1], info[2], ljm.numberToIP(info[3]), info[4], info[5]))

LabJack_Sampling_Rate = 20
Disp_Update_Rate = 10
Text_Recording_Rate = 1

ljm.eWriteName(handle, "AIN_SAMPLING_RATE_HZ", LabJack_Sampling_Rate)
Actual_Sampling_Rate = ljm.eReadName(handle, "AIN_SAMPLING_RATE_ACTUAL_HZ")
print("AIN Sampling Rate: ", Actual_Sampling_Rate)

time_last = time.time()
read_index = 0

Sampling_Rate_PID = 20

root = Tk()
root.geometry("700x400")
root.title("MP3 - Control System")
root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)

# Initizlizing the Window itself

tabs = ttk.Notebook(root) 
  
Powder_Delivery_Tab = ttk.Frame(tabs, padding="3 3 12 12") #
MAF_Tab = ttk.Frame(tabs, padding="3 3 12 12") 
Throttle_Tab = ttk.Frame(tabs, padding="3 3 12 12")
Thermocouple_Tab = ttk.Frame(tabs, padding="3 3 12 12")
Gas_Analyzer_Tab = ttk.Frame(tabs, padding="3 3 12 12")
Pressure_Tab = ttk.Frame(tabs, padding="3 3 12 12") 
CO2_Tab = ttk.Frame(tabs, padding="3 3 12 12")
Relay_Tab = ttk.Frame(tabs, padding="3 3 12 12")
Recording_Tab = ttk.Frame(tabs, padding="3 3 12 12")
Settings_Tab = ttk.Frame(tabs, padding="3 3 12 12") 

tabs.add(Powder_Delivery_Tab, text ='Powder Delivery') 
tabs.add(MAF_Tab, text ='MAF')
tabs.add(Throttle_Tab, text ='Throttle')
tabs.add(Thermocouple_Tab, text ='Thermocouple') 
tabs.add(Gas_Analyzer_Tab, text ='Gas Analyzer')
tabs.add(Pressure_Tab, text ='Pressure') 
tabs.add(CO2_Tab, text ='CO2 Meter')
tabs.add(Relay_Tab, text ='Relay')
tabs.add(Recording_Tab, text ='Data Recording')
tabs.add(Settings_Tab, text ='Settings') 
tabs.pack(expand = 1, fill ="both")

MAF_CLASS = MAF(MAF_Tab, AIN_NAME = 'MAF Sensor', AIN_NUM = "0", AIN_UNIT='g/s')
PD_CLASS = LabJack_Motor_PWM(Powder_Delivery_Tab)
TPSA_CLASS = TPSA(Throttle_Tab, AIN_NAME = 'TPS Sensor A', AIN_NUM = "1", AIN_UNIT='% Open')
#TPSB_CLASS = TPSB(Throttle_Tab, AIN_NAME = 'TPS Sensor B', AIN_NUM = "2", AIN_UNIT='% Open', ROW=3)
#Average_TPS = StringVar()

#PID
"""
20Hz Best for Limited
kp = 600  # Proportional gain
ki = 5  # Integral gain
kd = 50  # Derivative gain
"""

kp = 100  # Proportional gain
ki = 0  # Integral gain
kd = 20  # Derivative gain
previous_error = 0
integral = 0
dt = 1/Sampling_Rate_PID  # Time step
current_open = TPSA_CLASS.AIN_Value
setpoint = TPSA_CLASS.Inverse_Calibration_Formula(0)  # Desired setpoint to start (0% Open)
DC = 0

Throttle_PID_CLASS = Throttle_PID_Controller(DC, integral, previous_error, setpoint, kp, ki, kd, dt)

Thermocouple_Reactor_CLASS = Thermocouple(Thermocouple_Tab, AIN_NAME = 'Reactor Thermocouple', AIN_NUM = "2", AIN_UNIT='°C', ROW=0)
Thermocouple_Pre_HX_CLASS = Thermocouple(Thermocouple_Tab, AIN_NAME = 'Post HX Thermocouple', AIN_NUM = "3", AIN_UNIT='°C', ROW=7)
Thermocouple_Post_HX_CLASS = AIN_4_20mA(Thermocouple_Tab, AIN_NAME = 'Pre HX Thermocouple', AIN_NUM = "4", AIN_UNIT='°C', ROW=3, Resistance=473)

Gas_Analyzer_NOX_CLASS = Gas_Analyzer_NOX(Gas_Analyzer_Tab, AIN_NAME = 'NOX', AIN_NUM = "5", AIN_UNIT='ppm', ROW=0)
Gas_Analyzer_CO_CLASS = Gas_Analyzer(Gas_Analyzer_Tab, AIN_NAME = 'CO', AIN_NUM = "7", AIN_UNIT='ppm', ROW=3)
Gas_Analyzer_O2_CLASS = Gas_Analyzer(Gas_Analyzer_Tab, AIN_NAME = 'O2', AIN_NUM = "6", AIN_UNIT='% O2', ROW=6)

Throttle_Motor_CLASS = Throttle_PWM(Throttle_Tab, ROW=8)

Pre_Filter_RS485_Address = 2
Post_Filter_RS485_Address = 1

Pressure_Pre_Filter_CLASS = Pressure_Sensor(Pressure_Tab, Pressure_Pre_Filter, Pre_Filter_RS485_Address, ROW=0, Label="Pre-Filter Pressure (kPa): ")
Pressure_Post_Filter_CLASS = Pressure_Sensor(Pressure_Tab, Pressure_Post_Filter, Post_Filter_RS485_Address, ROW=6, Label="Post-Filter Pressure (kPa): ")

CO2_Atmosphere_CLASS = CO2_Meter(CO2_Tab, CO2_Atmosphere, ROW=0, Label="Atmospheric Sensor: ", Label_CO2="Atmospheric CO2", Label_VOC="Atmospheric VOC", Label_PM25="Atmospheric PM2.5", Label_RH="Atmospheric RH", Label_Temp="Atmospheric Temp")
CO2_Reactor_CLASS = CO2_Meter(CO2_Tab, CO2_Reactor, ROW=6, Label="Reactor Sampling Sensor: ", Label_CO2="Reactor CO2", Label_VOC="Reactor VOC", Label_PM25="Reactor PM2.5", Label_RH="Reactor RH", Label_Temp="Reactor Temp")

Relay_LB_CLASS = Relay(Relay_Tab, DIO_NAME = 'Leaf Blower Relay', DIO_NUM = "2", DIO_TYPE='FIO', DIO_STATES="On/Off", DIO_DEFAULT=0, ROW=0)
Relay_PD_CLASS = Relay(Relay_Tab, DIO_NAME = 'PD Relay', DIO_NUM = "3", DIO_TYPE='FIO', DIO_STATES="On/Off", DIO_DEFAULT=0, ROW=1)

Recorder_CLASS = Recorder(Recording_Tab, Text_Recording_Rate)

Settings_CLASS = Settings(Settings_Tab)

#ttk.Label(Throttle_Tab, text='Average TPS (% Open): ').grid(column=1, row=7, sticky=E)
#ttk.Label(Throttle_Tab, textvariable=Average_TPS).grid(column=2, row=7, sticky=E)

for Tab in [Throttle_Tab, MAF_Tab, Powder_Delivery_Tab, Thermocouple_Tab, Gas_Analyzer_Tab, Pressure_Tab, CO2_Tab, Relay_Tab, Recording_Tab, Settings_Tab]:
    for child in Tab.winfo_children(): 
        child.grid_configure(padx=5, pady=5)

#TPSB(Throttle_Tab, AIN_NAME = 'TPS Sensor B', AIN_NUM = "2", AIN_UNIT='% Open', ROW=4)

root.protocol("WM_DELETE_WINDOW", on_closing)
Update_Disp()

thread_PID = threading.Thread(target=scheduled_update_PID, daemon=True)
thread_PID.start()

print(sys.version)

#CO2_Atmosphere_CLASS.Calibrate_CO2(450)
#CO2_Reactor_CLASS.Calibrate_CO2(450)

root.mainloop()



