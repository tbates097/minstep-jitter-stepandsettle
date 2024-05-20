# -*- coding: utf-8 -*-
"""
Created on Thu Jun 16 09:34:03 2022

@author: MECHASSY
Edited by TB on 01/08/2024
Deleted underscores from A1 commands
Modified the ai0 data collection to allow selecting which axis the external sensor feedback comes in on
"""

import automation1 as a1
import sys
sys.path.append('../')
import a1data
import time as timemodule
import numpy as np
import scipy.signal as signal
import scipy.integrate as integrate
from scipy.optimize import curve_fit

class jitter(a1data.a1data):
    '''
    Represents a B5.64 compliant test procedure using the Automation 1 controller. When constructing an instance
    of this class, you specify the test parameters you want, then use the 'test' function to actually perform the
    in-position jitter test
    

    Parameters
    ----------
    axis : str
        The axis that the test will be run on.
    sample_rate : int
        Sample rate collected by the Automation 1 controller in Hz, must match up with one of the existing
        Automation 1 sample rate values.
    test_time : float
        Time of the test in seconds.
    direction : a1data.mode
        Either a1data.mode.positive_direction or a1data.mode.negative_direction. specifies the direction, according
        to the internal position, that the stage must travel to be in range of the sensor.
    sensitivity : float
        The sensitivity of the external position measurement sensor in units of self.units/V

    Returns
    -------
    None.

    '''
    
    def __init__(self, axis : str, sample_rate : int, test_time, direction, sensitivity, probe_axis, **kwargs):
        '''
        

        Parameters
        ----------
        axis : str
            The axis that the test will be run on.
        sample_rate : int
            Sample rate collected by the Automation 1 controller in Hz, must match up with one of the existing
            Automation 1 sample rate values.
        test_time : float
            Time of the test.
        direction : a1data.mode
            Either a1data.mode.positive_direction or a1data.mode.negative_direction. specifies the direction, according
            to the internal position, that the stage must travel to be in range of the sensor.
        sensitivity : float
            The sensitivity of the external position measurement sensor in units of self.units/V
        Returns
        -------
        None.

        '''

        super().__init__(axis,sample_rate,probe_axis,**kwargs)
        self.test_time = test_time
        self.direction = direction
        self.sens = sensitivity
        self.probe_axis = probe_axis

        
        
    def test(self, controller : a1.Controller):
        '''
        Method to perform the corresponding Automation 1 test in compliance with ASME B5.64

        Parameters
        ----------
        controller : a1.Controller.connect()
            The active Automation1 controller that should be performing the specified test

        Returns
        -------
        None

        '''
        #Go through the physical part of the instructions step by step until the code is able to completely take over
        
        #Set units
        self.n = (int)(self.sample_rate*self.test_time)
        
        #update controller units based on kwargs
        controller.runtime.parameters.axes.__setattr__('UnitsName', self.units)
        
        #Make sample_rate the correct class type for automation 1
        if self.sample_rate == 1000:
            self.__freq = a1.DataCollectionFrequency.Frequency1kHz
        elif self.sample_rate == 10000:
           self.__freq = a1.DataCollectionFrequency.Frequency10kHz
        elif self.sample_rate == 20000:
            self.__freq = a1.DataCollectionFrequency.Frequency20kHz
        elif self.sample_rate == 100000:
            self.__freq = a1.DataCollectionFrequency.Frequency100kHz
        elif self.sample_rate == 200000:
            self.__freq = a1.DataCollectionFrequency.Frequency200kHz
        else:
            raise ValueError('Frequency is invalid. Choose from available Automation 1 Options')
        #Make sure there are enough points in each sample    
        if self.n < 100:
            raise ValueError('Minimum threshold of 100 data points not met')
            
        #Set up motion parameters 
        controller.runtime.commands.motion_setup.setuptasktargetmode(a1.TargetMode.Absolute)
        controller.runtime.commands.motion_setup.setupcoordinatedramptype(self.ramp_type, self.ramp_type_arg, 
                                                                             self.ramp_type, self.ramp_type_arg)
        
        controller.runtime.commands.motion_setup.setupcoordinatedrampvalue(a1.RampMode.Rate, self.ramp_value, 
                                                                              a1.RampMode.Rate, self.ramp_value)
        controller.runtime.commands.motion.enable([self.axis],1)
        #controller.runtime.commands.motion.home(['X','Y'], 1)
        
        

        
        #Data configurations. These are how to configure data collection parameters
        if self.probe_axis == 'None':
            data_config = a1.DataCollectionConfiguration(self.n, self.__freq)  #Freq should be 20x the max frequency required by end process
            data_config.system.add(a1.SystemDataSignal.DataCollectionSampleTime)
            data_config.axis.add(a1.AxisDataSignal.PositionCommand, self.axis)
            data_config.axis.add(a1.AxisDataSignal.PositionFeedback, self.axis)
            data_config.axis.add(a1.AxisDataSignal.PositionError, self.axis)
            data_config.axis.add(a1.AxisDataSignal.VelocityCommand, self.axis)
            data_config.axis.add(a1.AxisDataSignal.VelocityFeedback, self.axis)
            data_config.axis.add(a1.AxisDataSignal.VelocityError, self.axis)
            data_config.axis.add(a1.AxisDataSignal.AnalogInput0, self.axis)
        ###########TB
        else:
            data_config = a1.DataCollectionConfiguration(self.n, self.__freq)  #Freq should be 20x the max frequency required by end process
            data_config.system.add(a1.SystemDataSignal.DataCollectionSampleTime)
            data_config.axis.add(a1.AxisDataSignal.PositionCommand, self.axis)
            data_config.axis.add(a1.AxisDataSignal.PositionFeedback, self.axis)
            data_config.axis.add(a1.AxisDataSignal.PositionError, self.axis)
            data_config.axis.add(a1.AxisDataSignal.VelocityCommand, self.axis)
            data_config.axis.add(a1.AxisDataSignal.VelocityFeedback, self.axis)
            data_config.axis.add(a1.AxisDataSignal.VelocityError, self.axis)
            data_config.axis.add(a1.AxisDataSignal.AnalogInput0, self.probe_axis)
        ###########TB
        
        
        # #Homing instructions if Cap Probe is used as to not break the cap probe
        # #Move stage until it is in the 0 position of the cap probe
        # if (self.direction == a1data.mode.negative_direction):
        #     controller.runtime.commands.motion.move_freerun(self.axis, [-50])
        # elif (self.direction == a1data.mode.positive_direction):
        #     controller.runtime.commands.motion.move_freerun(self.axis, [50])
        # #Poll to see if at origin of cap probe data
        # while True:
        #     if ((controller.runtime.status.get_status_items(status_item_configuration).axis.get(a1.AxisStatusItem.AnalogInput0 , self.axis).value) >= 0): #If AIN0 passes 0 point, break out of the loop
        #         controller.runtime.commands.motion.move_freerun_stop(self.axis, 1)
        #         break
    
    
        #Homing instructions for the purposes of the demo. Use PosFbk with no cap probe attached
        #controller.runtime.commands.motion.home(self.axis)

        
        timemodule.sleep(5) #Wait 5 seconds for the device to settle
        
        #Runs the test with the given parameters
        results = controller.runtime.data_collection.collect_snapshot(data_config)
        
        #Results as n length arrays with all of the data points collected
        self.populate(results)
        
    def data_analysis(self, mode : a1data.mode, window : tuple):
        t, pos = [], []
        d = self.choose_array(mode)
        
        #If analog data is chosen, must be multiplied by the sensitivity
        if mode.value == a1data.mode.ai0:
            d = [e * self.sens for e in d]
            
    
        
        
        #Window the time array and the chosen position array based on the window input
        for i, e in enumerate(self.time_array):
            if (e >= window[0]) & (e < window[1]):
                t.append(e)
                #Check if analog data. if so, adjust by sensor sensitivity
                if mode == a1data.mode.ai0:
                    pos.append(d[i]*self.sens)
                else:
                    pos.append(d[i])
                
        d = pos
        
        #Throw errors if data fails to meet standard requirements
        if len(t) <= 100:
            raise ValueError('Minimum of 100 data points required. Please extend the window')
        if (t[-1] - t[0]) < .250:
            raise ValueError('Minimum of 250 ms of data is required. Please extend the window')
        
        #Standard deviation calculation
        summation = 0
        d_ave = np.mean(d)
        for d_i in d:
            summation += (d_i - d_ave)**2
            
        s = np.sqrt(summation/(len(d)-1)) #Standard deviation
        d_pk_pk = np.max(d) - np.min(d) #Peak to peak value
        
        #Method for creating a CRMS Plot
        #https://blog.endaq.com/top-vibration-metrics-to-monitor-how-to-calculate-them#cum
        
        PSD = signal.periodogram(d, self.sample_rate) #Generate a power spectral density function as a startingpoint for cumulative rms
        f = PSD[0] #List of sample frequencies
        Pxx = PSD[1] #Power spectral density
        CRMS = np.sqrt(integrate.cumtrapz(Pxx)) #Cumulatively integrate
        CRMS = np.insert(CRMS,0,0) #Need to match up array lengths by inserting a 0 to the first element of the CRMS array
        
        data_dict = {'stdev' : s, #Standard deviation of the data inside the window
                     'peak' : d_pk_pk, #Peak to peak distance of the data inside the window
                     't_window' : t, #time array inside the window
                     'd_window' : d, #data array inside the window
                     'freq' : f, #List of frequencies corresponding to the CRMS values
                     'CRMS' : CRMS} #Cumulative RMS list
        
        return data_dict
       
    
    
    def remove_offset(self, data : a1data.mode):
        '''
        Remove a DC offset from the specified data signal

        Parameters
        ----------
        data : a1data.mode
            The specified data signal to be filtered

        Returns
        -------
        None.

        '''
        d = self.choose_array(data)
        
        d_copy = d - np.mean(d) #Remove DC Offset
        
        #Need to structure code this way to actually alter the intended array rather than a copy of the array
        for i,e in enumerate(d_copy):
            d[i] = e
            
    def endpoint_linear_norm(self, data : a1data.mode):
        '''
        Performs an endpoint linear normalization and prints an equation for the subtracted line

        Parameters
        ----------
        data : a1data.mode
            The specified data signal to be filtered

        Returns
        -------
        None.

        '''
        d = self.choose_array(data)
        #Endpoint Linear Normalization

        m = float((d[-1]-d[0])/(self.time_array[-1] - self.time_array[0])) #slope = rise/run
        
        y1 = d[0]
        x1 = self.time_array[0]

        for i, element in enumerate(d):
            d[i] -= (m*(self.time_array[i]-x1)+y1) #subtract line between endpoints

        print('y = {}*(t-{}) + {}'.format(m,x1,y1))
        
    def least_squares_linear_norm(self, data: a1data.mode):
        '''
        Performs a least-squares linear normalization and prints the subtracted line
        

        Parameters
        ----------
        data : a1data.mode
            The specified data signal to be filtered

        Returns
        -------
        None.

        '''
        d = self.choose_array(data)
        #Least-Squares Linear Normalization
        def func(x, a, b):
            y = a*x+b
            return y
        
        alpha = curve_fit(func, xdata = self.time_array, ydata=d)
        
        m = alpha[0][0]
        b = alpha[0][1]
        
        for i in range(len(d)):
            d[i] -= (m*self.time_array[i]+b) #subtract line
            
        print('y = {}x + {}'.format(m,b))
    
    # def import_params(self,filename):
    #     super(jitter,self).import_params(filename)
        
    #     #Enforce property types
    #     self.test_time = (float)(self.test_time)
    #     #self.direction = (a1data.mode)(self.direction)
    #     self.sens = (float)(self.sens)

    
    
    
    
        
    
    