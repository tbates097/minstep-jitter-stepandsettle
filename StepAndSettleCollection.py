# -*- coding: utf-8 -*-
"""
Created on Mon Jun 13 10:57:47 2022
Edited last on 6/21/2022 at 8:00 am

@author: tbates
"""

import automation1 as a1
import numpy as np
import time
from matplotlib import gridspec
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pandas as pd
import os
import sys
sys.path.append('../')

import a1data


class move_and_settle(a1data.a1data):
    '''
    Represents a B5.64 compliant test procedure using the Automation 1 controller. When constructing an instance
    of this class, you specify the test parameters you want, then use the 'test' function to actually perform the
    move-and-settle test

    Parameters
    ----------
    axis : str
        The axis that the test will be run on.
    sample_rate : int
        Sample rate collected by the Automation 1 controller in Hz, must match up with one of the existing
        Automation 1 sample rate values.
    step_size : float
        Size of each step, units are the same as the controller parameter.
    frame : tuple
        Beginning and end points of the test in terms of an absolute coordinate system,
        units are the same as the controller parameter.
    **kwargs : Optional parameters
        direction : a1data.mode
            Specify either a unidirectional test or a bidirectional test
        num_cycles : int
            Number of times to repeat the move and settle test across the frame
        step_time: float
            Time between each step and the time of each data collection period
        speed : float
            speed of the movement in units per second
        ramp_type : automation1.RampType
            Specify the type of ramp performed by the Automatiion 1 controller
        ramp_value : float
            Specify the acceleration of the move. Units are the same as the controller parameter.
        ramp_type_arg : float
            The ramping type additional argument for accelerations. 
            This is only used when $rampTypeAccel is RampType.SCurve and represents the s-curve percentage.
        folder : str
            This is specifies the folder used to store the .csv files in self.test() and access the files in self.plot()
            and self.to_dataframe()
        units: str
            units of the automation 1 controller

    Returns
    -------
    None.

    '''
    
    #Inputs for the Move and Settle Class
    #   axis
    #   step_size
    #   speed
    #   frame
    #   sample_rate (Hz)
    
    
    #**kwargs
    #   direction (1 or 2 for unidirectional or bidirectional)
    #   num_cycles
    #   step_time
    #   speed
    #   ramp_type - Linear, SCurve, Sine or another option : a1.RampType
    #   ramp_value - acceleration value
    #   ramp_type_arg - Percentage for an SCurve ramp type
    #   units - units used by automation 1
    #   folder - folder name in the current directory where the csv files will be stored.

    

    def __init__(self, axis:str, sample_rate, step_size:float, frame:tuple, probe_axis, **kwargs):

        #Input arguments that are necessary for the test to run
        super().__init__(axis, sample_rate, probe_axis, **kwargs)
        self.step_size = step_size
        self.frame = frame
        self.probe_axis = probe_axis
        
        #Input arguments that can be defined but may not be necessary
        default_kwargs = {'direction' : a1data.mode.Bidirectional,
                          'num_cycles': 1,
                          'step_time' : 1, #second
                          'folder' : 'moveandsettlecollection'
                         }
        
        #Update defaults if they exist
        kwargs = {**default_kwargs,**kwargs}
        
        self.direction = kwargs['direction']
        self.num_cycles = kwargs['num_cycles']
        self.step_time = kwargs['step_time']
        self.folder = kwargs['folder']
        
        
        #Non-user definable parameters
        self.data_len = -1 #Obviously fake number for error checking, defined in populate method
        self.pos_dev = [[]] #Position Deviation, 2D list
        
            
    def test(self, controller : a1.Controller):
        '''
        Method to perform the corresponding Automation 1 test in compliance with ASME B5.64

        Parameters
        ----------
        controller : a1.Controller.connect()
            The active Automation1 controller that should be performing the specified test

        Returns
        -------
        CSV files corresponding to each step performed in the move-and-settle test.
        These files are located in the folder, 'moveandsettlecollection', in the current directory

        '''
        #Set units
        self.n = (int)(self.sample_rate*self.step_time)
        data_config = super().test(controller)
        
        #Create folder for csv to be stored.
        
        
        self.current_directory = os.getcwd()
        self.new_folder_name = self.folder
        self.new_folder_path = os.path.join(self.current_directory, self.new_folder_name)
        
        if os.path.exists(self.new_folder_path) and os.path.isdir(self.new_folder_path):
            for filename in os.listdir(self.new_folder_path):
                self.file_path = os.path.join(self.new_folder_path, filename)
                if os.path.isfile(self.file_path):
                    os.remove(self.file_path)
        
        os.makedirs(self.new_folder_path, exist_ok=True)
        
        #Move to start position of travel
        controller.runtime.commands.motion.movelinear(self.axis, [self.frame[0]], self.speed)
        
        #wait for the stage to be in position
        time.sleep(self.step_time)


        #For the number of cycles that should be run
        for i in range(1, self.num_cycles + 1):
            #For the amount of steps that can fit in the given frame
            for j in range(1, (self.frame[1] - self.frame[0]) // self.step_size + 1):
                #Collect data and move
                controller.runtime.data_collection.start(a1.DataCollectionMode.Snapshot, data_config)
                
                # #Wait before move starts
                # time.sleep(1/100)
                
                controller.runtime.commands.motion.movelinear(self.axis, [self.frame[0] + j*self.step_size], self.speed)
                
                #Wait for the results to complete
                results = controller.runtime.data_collection.get_results(data_config, self.n)
                
                #Results as n length arrays with all of the data points collected
                self.populate(results)

                #Write cycle to a csv
                self.write_to_csv('{}\step{}_{}.csv'.format(self.folder, j,i))

                
            #If Bidirectional move and settle requested    
            if self.direction == a1data.mode.Bidirectional:
                for j in range(1, (self.frame[1] - self.frame[0]) // self.step_size + 1):
                    #Collect data and move backward
                    #Collect data and move
                    controller.runtime.data_collection.start(a1.DataCollectionMode.Snapshot, data_config)
                   
                    # #Wait before move starts
                    # time.sleep(1/100)
                    
                    #Move backward to the next step
                    controller.runtime.commands.motion.movelinear(self.axis, [self.frame[1] - j*self.step_size], self.speed)
                   
                    #Wait for the results to complete
                    results = controller.runtime.data_collection.get_results(data_config, self.n)
                    
                    #Results as n length arrays with all of the data points collected
                    self.populate(results)

                    
                    #Write cycle to a csv
                    self.write_to_csv('{}\stepback{}_{}.csv'.format(self.folder,j,i))
            
            #Move to start position of travel    
            else:
                controller.runtime.commands.motion.movelinear(self.axis, [self.frame[0]], self.speed)
                time.sleep(self.step_time)
                
        self.data_files = [file for file in os.listdir(self.folder) if 'csv' in file]
        self.data_len = len(self.data_files)
        
                       
                    
    def to_dataframe(self):
        '''
        Since a move and settle test requires the collection of multiple test windows at once, this method
        converts the csv files in self.folder to a list of dataframes for ease of accessing multiple data sets at 
        once

        Returns
        -------
        dataframe_list : pd.Dataframe[]
            List of every dataframe from a move-and-settle test

        '''
        dataframe_list = []
        #For number of cycles
        for i in range(1, self.num_cycles + 1):
            #For the amount of steps that can fit in the given frame
            for j in range(1, (self.frame[1] - self.frame[0]) // self.step_size + 1):
                dataframe_list.append(pd.read_csv(r'{}\step{}_{}.csv'.format(self.folder,j,i))) #Read in Move and Settle as a csv file
            if self.direction == a1data.mode.Bidirectional:
                for j in range(1, (self.frame[1] - self.frame[0]) // self.step_size + 1):
                    dataframe_list.append(pd.read_csv(r'{}\stepback{}_{}.csv'.format(self.folder,j,i))) #Read in Move and Settle as a csv file
        return dataframe_list
    
    
    def populate(self, results = None, file = None, dataframe = None, folder = None, df_list = None):
        super(move_and_settle,self).populate(results, file, dataframe)
        
        #Extending the populate function of a1data class to check if a folder was inputted
        if folder is not None: 
            data_files = [file for file in os.listdir(folder) if 'csv' in file]
            
            
            #data is a list of dataframes
            data = [pd.read_csv(folder + '\\' +i) for i in data_files]
            self.time_array = [(file['Time (seconds)']).tolist() for file in data]
            self.pos_com = [(file[' PosCmd ({}) {}'.format(self.axis, self.units)].tolist()) for file in data]
            self.pos_fbk = [(file[' PosFbk ({}) {}'.format(self.axis, self.units)].tolist()) for file in data]
            self.pos_err = [(file[' PosErr ({}) {}'.format(self.axis, self.units)].tolist()) for file in data]
            self.vel_com = [(file[' VelCmd ({}) {}'.format(self.axis, self.units)].tolist()) for file in data]
            self.vel_fbk = [(file[' VelFbk ({}) {}'.format(self.axis, self.units)].tolist()) for file in data]
            self.vel_err = [(file[' VelErr ({}) {}'.format(self.axis, self.units)].tolist()) for file in data]
            self.ai0 = [(file[' Ain 0 ({})'.format(self.axis)].tolist()) for file in data]
        elif df_list is not None:
            #Extending further to input a list of dataframes
            self.time_array = [(file['Time (seconds)']).tolist() for file in df_list]
            self.pos_com = [(file[' PosCmd ({}) {}'.format(self.axis, self.units)].tolist()) for file in df_list]
            self.pos_fbk = [(file[' PosFbk ({}) {}'.format(self.axis, self.units)].tolist()) for file in df_list]
            self.pos_err = [(file[' PosErr ({}) {}'.format(self.axis, self.units)].tolist()) for file in df_list]
            self.vel_com = [(file[' VelCmd ({}) {}'.format(self.axis, self.units)].tolist()) for file in df_list]
            self.vel_fbk = [(file[' VelFbk ({}) {}'.format(self.axis, self.units)].tolist()) for file in df_list]
            self.vel_err = [(file[' VelErr ({}) {}'.format(self.axis, self.units)].tolist()) for file in df_list]
            self.ai0 = [(file[' Ain 0 ({})'.format(self.axis)].tolist()) for file in df_list]
        
        
        
        
        #if type(self.pos_com[0]) == float: #checks if there is a single file present and sets the length of the data files
            #self.data_len = 1 #length of the data files is 1
        #else: #Checks if there is more than one file present and sets the length of the data files 
            #self.data_len = len(self.time_array) #length of the data files present is the number of arrays in the time_array signal
            
            
        
    
    
    
    
    
    
    
    def adjust_start(self):
        '''
            Shifts the starting point of all collected signals so that the start point corresponds with the start of the step. For the time array,
            the endpoint is shifted back by the same number of points to match the length. 
        Returns
        -------
        None.

        '''
        self.time_array = []
        self.pos_com = []
        self.pos_fbk = []
        self.pos_err = []
        self.vel_com = []
        self.vel_fbk = []
        self.vel_err = []
        self.ai0 = []
       
        #Empty variables for determining all of the indices where there is no movement, the index groupings where there is no movement, and the total move time for each file
        no_move = []
        no_move_range =  []
        self.move_time = []
        
        self.csv_path = f"{self.folder}/{self.df}"
        data = pd.read_csv(self.csv_path)
        try:
            self.time_array = data['Time (seconds)'].tolist()
            self.pos_com = (data[' PosCmd ({}) {}'.format(self.axis, self.units)]).tolist()
            self.pos_fbk = (data[' PosFbk ({}) {}'.format(self.axis, self.units)]).tolist()
            self.pos_err = (data[' PosErr ({}) {}'.format(self.axis, self.units)]).tolist()
        except KeyError:
            self.time_array = data['Time (sec)'].tolist()
            self.pos_com = (data[' PosCmd ({}) ({})'.format(self.axis, self.units)]).tolist()
            self.pos_fbk = (data[' PosFbk ({}) ({})'.format(self.axis, self.units)]).tolist()
            self.pos_err = (data[' PosErr ({}) ({})'.format(self.axis, self.units)]).tolist()
        try:
            self.vel_com = (data[' VelCmd ({}) {}'.format(self.axis, self.units)]).tolist()
            self.vel_fbk = (data[' VelFbk ({}) {}'.format(self.axis, self.units)]).tolist()
            self.vel_err = (data[' VelErr ({}) {}'.format(self.axis, self.units)]).tolist()
        except KeyError:
            self.vel_com = (data[' VelCmd ({}) ({}/sec)'.format(self.axis, self.units)]).tolist()
            self.vel_fbk = (data[' VelFbk ({}) ({}/sec)'.format(self.axis, self.units)]).tolist()
            self.vel_err = (data[' VelErr ({}) ({}/sec)'.format(self.axis, self.units)]).tolist()
        try:
            self.ai0 = (data[' Ain 0 ({})'.format(self.axis)]).tolist()
        except KeyError:
            pass


        #for file in range(self.data_len): #For every file that is present
        
        #Condition that checks if the length of the current time_array is not 1-this will fail during the second iteration for a single file
        if len(self.time_array) != 1: 
            no_move = np.where(np.array(self.vel_com) == 0)[0] #finds all indices where vel_com = 0 

            start_index = 0
            for idx, val in enumerate(no_move[1:], start = 1):
                    
                #Condition for grouping no move indices: successive indices that are part of one stretch of no movement will have a difference of 1
                if ((val-no_move[idx-1] !=1) & (idx<len(no_move)-1)): 
                    no_move_range.append([no_move[start_index], no_move[idx-1]])
                    start_index = idx
                #Condition for last strectch of no movement, makes sure that the difference between successive points is equal to 1
                elif ((val-no_move[idx-1] ==1) & (idx==len(no_move)-1)): 
                    no_move_range.append([no_move[start_index], no_move[len(no_move)-1]])
                
        #Shifts the starting point of all signals to the index where the first movement occurs, shifts endpoint of time_array back by the same number of indices
        self.time_array = self.time_array[:(len(self.time_array)-no_move_range[0][1])]
        self.pos_com = self.pos_com[no_move_range[0][1]:]
        self.pos_fbk = self.pos_fbk[no_move_range[0][1]:]
        self.pos_err = self.pos_err[no_move_range[0][1]:]
        self.vel_com = self.vel_com[no_move_range[0][1]:]
        self.ai0 = self.ai0[no_move_range[0][1]:]
        
        #Calculates the total duration of the move for each step file
        
        self.move_time = (no_move_range[1][0]-no_move_range[0][1])/self.sample_rate   
                
    def moving_window(self, process_window, window_direction):
        '''
        
        
        Parameters
        ----------
        process_window : float
            Duration of the desired process window in seconds
        window_direction : a1data.mode
            Enum found in mode class of a1data module used to describe direction of process window: forward, centered, or backward

        Returns
        -------
        window_dict : dict
            Dictionary containing the following values related to the desired window:
                Process Window: float
                    Process window that was chosen
                
                Process Window Direction: string
                    Specified calculation direction of the process window 

                Window Ranges: list
                    Upper and lower index range of each process window throughout the length of the position deviation signal
                

        '''
        #Calling the adjust_start() method to make sure the starting point of the signals are where the move starts
        self.adjust_start()
        #Calculates the index length that the process window spans over 
        self.window_len = process_window*self.sample_rate
        
        window_ranges = []
        
        #Determines starting and ending points based off window and calculation direction
        #for file in range(self.data_len):
            
        #Condition that checks if the length of the current time_array is not 1-this will fail during the second iteration for a single file
        if len(self.time_array) != 1: 
            if window_direction == a1data.mode.forward_window: #forward window process
                lowerSubtract = 0.0
                upperAdd = self.window_len
                window_direction = 'Forward'
            
            elif window_direction == a1data.mode.centered_window: #centered window process
                lowerSubtract = self.window_len/2
                upperAdd = self.window_len/2
                window_direction = 'Centered'
            elif window_direction == a1data.mode.backward_window: #backward window process
                lowerSubtract = self.window_len
                upperAdd = 0.0
                window_direction = 'Backward'
                
            time_vals = self.time_array
                
            #Finding the starting and ending point of each process window throughout the length of time_array
            start_pos = [0 if round(i*self.sample_rate - lowerSubtract) <0 else round(i*self.sample_rate - lowerSubtract) for i in time_vals]
            end_pos = [len(time_vals)-1 if int(i*self.sample_rate + upperAdd) > len(time_vals) else int(i*self.sample_rate + upperAdd) for i in time_vals]
   
            #Pairing each start and end index for each process window     
            window_ranges = ([[j, k] for j,k in zip(start_pos, end_pos)])
            
        #Dictionary with the window ranges and the specified process window direction 
        window_dict = {'Window Ranges': window_ranges,
                       'Process Window (sec)': process_window,
                       'Process Window Direction': window_direction}
        
        return window_dict
    
    
    def data_analysis(self, error_metric, process_window, window_direction, position_tolerance, df) -> dict:
        '''
        Parameters
        ----------
        error_metric : a1data.mode
            Enum found in mode class of a1data module used to specify which ASME B5.64 error metric should be found.
        process_window : float
            Duration of the desired process window in seconds.
        window_direction : a1data.mode
            Enum found in mode class of a1data module used to describe direction of process window: forward, centered, or backward
        position_tolerance : float
            Desired position tolerance that must be met for system to be considered settled 

        Returns
        -------
        data_dict : dict
            Dictionary with the following information related to the data analysis:
                Error Metric Values: list
                    Calculated error metric values for each step based of which metric was chosen
                
                Position Deviation Windows: list
                    Index ranges of each window throughout the signal used to determine each position deviation window
                
                Move and Settle Time: list
                    Move and Settle time found for each step file that was included

        '''
        self.df = df
        #Calling the moving window function to find the each window index range throughout the entire position deviation signal             
        window_dict = self.moving_window(process_window, window_direction)
        #Extracting the window index range
        moving_window = window_dict['Window Ranges']
        
        
        
        #Creating empty list of lists for the number of files that are present
        error_metric_vals = []
        t_ms = []
        dev_window = []
        self.pos_dev = []
        self.move_end_ind = []
        
        #for file in range(self.data_len): #for the number of files that were found
            #Condition that checks if the length of the current time_array is not 1-this will fail during the second iteration for a single file
            #Condition that checks if the length of the current time_array is not 1-this will fail during the second iteration for a single file
        if len(self.time_array) != 1: 
                
            #Equation 7.3.1 for calculating the dynamic position deviation
            self.pos_dev = [abs(i - self.pos_com[-1]) for i in self.pos_fbk]
 
            #Finding each position deviation window based off the index ranges in moving_window
            dev_window = [(self.pos_dev[i[0]:i[1]]) for i in moving_window] 
            
            #Calculating the desired error metric (Moving Average Error, Moving Standard Deviation, or Moving Peak Error)
            if error_metric == a1data.mode.MAE: 
                error_metric_vals = [round(sum(i) / self.window_len, 4) for i in dev_window]
  
            elif error_metric == a1data.mode.MSD: 
                std_vals = [np.std(i) for i in dev_window]
                error_metric_vals = np.nan_to_num(std_vals, 0)
                      
            elif error_metric == a1data.mode.MPE: 
                error_metric_vals  = [max(list(map(abs, i))) if len(i) >=1 else 0 for i in dev_window]
                
                
            t_ind = len(error_metric_vals) - 1 #Last index of time
             
            #finds last time point where the error metric is less than position tolerance
            while abs(error_metric_vals[t_ind]) <= position_tolerance: 
                t_ms = round((self.time_array[t_ind]), 4)
                self.move_end_ind = t_ind
                t_ind-=1
                
            #Makes sure that the error metric move and settle time is not less than the move duration. 
            #This is essentially the instantly settled condition check
            if t_ms < self.move_time: t_ms = self.move_time

        #Initializing Data Dict
        data_dict = {}
     
        #Storing the error metric values in the data dictionary
        data_dict['Error Metric Values'] = error_metric_vals        
     
        #Storing window ranges used for the position deviation and the move and settle time(s) for all files present 
        data_dict['Position Deviation Windows'] = moving_window
        data_dict['Move and Settle Time'] = t_ms
        data_dict['Move and Settle Time Index'] = self.move_end_ind
     
        return data_dict
    
        
    
    
    def aero_move_and_settle(self, time_spec : float, settle_window : float):
        '''
        Determines the aerotech move and settle time for a move and settle test.
        Creates a bar graph displaying the aerotech move and settle time for each step overlayed with the desired time specification
        
        Parameters
        ----------
        time_spec : float
            Specified time specification not to exceed in seconds.
        settle_window : float
            Specified position window for the system to settle within in self.units

        Returns
        -------
        aerosettle_dict : dict
            Dictionary containing the following information related to the Aerotech Move and Settle Time: 
                Aerotech Move and Settle Time: list
                    Contains the move and settle time of the step using Aerotech's method
                Passed Settle Time: list
                    Bool returning if the calculated move and settle time was less than the specification
                    
        fig1: figure
            Figure containing a bar plot of each step that was included, and if its move and settle time met the specified criteria 

        '''
        #data_files = [file for file in os.listdir(self.folder) if 'csv' in file]
        #self.data_len = len(data_files)
        
        
        #Creating empty list of lists for the actual move and settle time of step file, and if the test passed or not
        actual_ms_time = []
        test_passed = []
        
        #for file in range(self.data_len): #for every file that is present
        
        #Condition that checks if the length of the current time_array is not 1-this will fail during the second iteration for a single file
        if len(self.time_array) != 1:
            #Temporary index of where the last position deviation that is greater than the move and settle window is located
            temp_pos_ind = np.where(np.abs(self.pos_dev[::-1])>=settle_window)[0]
            last_pos_ind = np.min(temp_pos_ind) if len(temp_pos_ind) > 0 else -1
    
            #if there are no indices in temp_pos_ind, stores the length of the current velocity command minus 1
            if last_pos_ind == 'None':
                last_pos_ind = len(self.vel_com)-1
                    
            #Adjusts index to be relative from the beginning rather than the end of the list
            settled_ind = len(self.vel_com) - last_pos_ind -1
   
            #Temporary index of where the last velocity command that is greater than zero is located
            temp_vel_ind = np.where(np.abs(self.vel_com[::-1])>0)[0]
            last_vel_ind = np.min(temp_vel_ind) if len(temp_vel_ind) > 0 else -1
    
            #if length of temp_vel_ind is 0, then there was no move captured
            if last_vel_ind == 'None': 
                print('No move captured')
                
                
            #Adjusts index to be relative from the beginning rather than the end of the list
            move_end_ind = len(self.vel_com) - last_vel_ind -1
    
            #If the settled index is less than the end of move index, then the stage instantly settled after the move,
            #so the move and settle time is the duration of the move. 
            if settled_ind < move_end_ind: 
                actual_ms_time = self.move_time
            
            #Otherwise, calculates the total move and settle time based off the difference of the move end and settled indices, as well as total move time
            else: actual_ms_time = (round((settled_ind-move_end_ind)/self.sample_rate+self.move_time,4))

            test_passed = (actual_ms_time <= time_spec) #Bool returning if the calculated move and settle time was less than the specification
            
         
        #Dictionary with information related to calculating Aerotech Move and Settle time
        aero_settle_dict = {'Aerotech Move and Settle Time': actual_ms_time,
                           'Time Specification': time_spec,
                           'Move and Settle Window': settle_window,
                           'Passed Move and Settle Time': test_passed}
                 
        return aero_settle_dict
    
    def aero_move_and_settle_bars(self, aero_dictionary):
        actual_ms_time = aero_dictionary['Aerotech Move and Settle Time']
        test_passed = aero_dictionary['Passed Move and Settle Time']
        settle_window = aero_dictionary['Move and Settle Window']
        time_spec = aero_dictionary['Time Specification']
        bar_width = 0.25 
        pass_color = 'green'
        fail_color = 'red'
        
        #Creating empty list of lists for the x-axis graph labels
        data_label = [[] for _ in range(self.data_len)]
        
        #for file in range(self.data_len): #for every file that is present
        if len(self.time_array) != 1:  
            #Creates data label based off the file, axis, step size, units, and specified move and settle window 
            if (self.direction == a1data.mode.Bidirectional):
                data_label = (['{0}: {1} (-{2}) {3} to +/-{4} {3}'.format(self.df, self.axis, self.step_size, self.units, settle_window)])
               
            else: 
                data_label = (['{0}: {1} ({2}) {3} to +/-{4} {3}'.format(self.df, self.axis, self.step_size, self.units, settle_window)])

        #Section that creates a txt file with info regarding the entire test, 
        #and a bar plot comparing the actual move and settle time with the time specification
        
        
        fig1, ax1 = plt.subplots(constrained_layout=True,figsize=(10,6))
        with open(os.path.join(self.folder,'Aerotech Move and Settle Time Results.dat'),'w+') as f1:
            f1.write('Test, Spec, Achieved, DidPass\n')
            
            start_ind = 0
            xtick_Vals, xtick_Labels = [[] for _ in range(2)]
            xtick_Labels = []
            bars1, bars2, this_ax_ind = [[0]*self.data_len for _ in range(3)]
           
          
            #for file in range(self.data_len):
            this_ax_ind = np.arange(actual_ms_time)*bar_width + start_ind
               
                
            if test_passed: 
                color_1 = pass_color
                hatch_1 = None
            else: 
                color_1=fail_color
                hatch_1 = '//'
                
            bars1 = ax1.bar(data_label, actual_ms_time,bar_width,color=color_1) #actual move and settle time from sec to ms
            ax1.bar_label(bars1)
            bars2 = ax1.bar(data_label, time_spec, bar_width,color='black',alpha=0.1,hatch=hatch_1) #desired move and settle time from sec to ms
            ax1.bar_label(bars2)
            f1.write('{0}, {1}, {2}, {3}\n'.format(data_label, time_spec, actual_ms_time, test_passed))
            xtick_Vals.extend(this_ax_ind)
            xtick_Labels.extend(data_label)
          
            ax1.set_xticklabels(xtick_Labels, rotation = 45, ha = 'right')
           
            plt.title('Aerotech Move and Settle Times')
            plt.ylabel('Move and Settle Time [sec]')
            plt.savefig('ResultsTEST.png',dpi=100, bbox_inches='tight')
        
        return fig1

    def plot(self, data_dict, aero_settle_dict, **kwargs):
        '''
        
        Parameters
        ----------
        data_dict : dict
            Data dictionary that is returned from running the data_analysis function using ASME B5.64 method of evaluation
        aero_settle_dict : dict
            Data dicionary that is returned from running the aero_move_and_settle function using Aerotech method of evaluation
     
        **kwargs : Optional Parameters 
            after_move_end: bool
                User decision whether they want to view the entire plot or the portion after the move ends (True/False)
            fignum: int
                Specifies the figure number of the metric plot
            legend_loc: string
                Specifies the location of the legend on the plot
            legend_size: float
                Specifies the font size of the text in the legend, and therefore its size
            signal: list
                Specifies which data values to use. Can be right from the class object

        Returns
        -------
        metric_plot : figure
            Figure containing the error metric plotted against the position deviation. Average position deviation and all error metrics shown if multiple files are included. 

        '''
        #Default kwargs if none are specified
        default_kwargs = {'step_num': 0,
                          'after_move_end': True,
                          'fignum': 1,
                          'legend_loc': 'best', 
                          'legend_size': 15.0,                              
                          'signal': self.pos_dev}
                                                        #'legend_loc_x': 1.01,
                                                        #'legend_loc_y': 1.0,
        #Updates kwargs in case some are specified 
        kwargs = {**default_kwargs,**kwargs}
        step_num = kwargs['step_num']
        after_move_end = kwargs['after_move_end']
        fignum = kwargs['fignum']
        legend_loc = kwargs['legend_loc']
        legend_size = kwargs['legend_size']
        signal = kwargs['signal']
        
                                                    #legend_loc_x = kwargs['legend_loc_x']
                                                    #legend_loc_y = kwargs['legend_loc_y']
        index = next((i for i, char in enumerate(step_num) if char.isdigit()), None)
        if index is not None:
            step_num = int(step_num[index]) if index < len(step_num) else None
        #Creating labels for signal and move and settle times for the plot legend
        sig_label = 'Position Deviation'
        #asme_settle_label = 'ASME Move and Settle Time'
        aerotech_settle_label = 'Aerotech Move and Settle Time'
        
        #all_asme_ms_times = data_dict['Move and Settle Time']
        all_aero_ms_times = aero_settle_dict['Aerotech Move and Settle Time']
        all_error_metric_vals = data_dict['Error Metric Values']

        #Plot creation, starts with the single/averaged specified signal
        metric_plot = plt.figure(fignum, figsize=(40, 20))    
        if step_num == 'Average':
            #Finding the length of each step in signal that is being plotted and finding each difference from the max length
            lst_lens = [len(lst) for lst in signal]
            max_diff = [max(lst_lens) - ln for ln in lst_lens] 
    
            for idx, i in enumerate(max_diff): 
                
                #matching the array length for each step less than the max length by adding 0's. This is done for averaging purposes
                if i !=0: signal[idx].extend(np.zeros(i)) 
            
            #Taking absolute value and average of all steps in the signal, matching length of time array, averaging the move and settle times
            abs_sig = [np.absolute(dat) for dat in signal]
            plt_signal = np.average(abs_sig, axis = 0)
            time_vals = self.time_array[max_diff.index(0)]
            #asme_settle_time = round(np.average(all_asme_ms_times), 4)
            aerotech_settle_time = round(np.average(all_aero_ms_times),4)
            
            #Label specifiying the number of step files used, signal and the move and settle time labels specifying that the average was taken 
            metric_label = 'Error Metric ({} Steps)'.format(self.data_len)
            sig_label = sig_label +' (Avg.)'
            #asme_settle_label =asme_settle_label + ' (Avg.)'
            aerotech_settle_label = aerotech_settle_label + ' (Avg.)'
            error_metric_vals = all_error_metric_vals
            
        else: #Labels and signals used for a single file
            plt_signal = np.abs(signal)
            time_vals = self.time_array
            metric_label = 'Error Metric (Step {})'.format(step_num)
            #asme_settle_time = all_asme_ms_times
            aerotech_settle_time = all_aero_ms_times
            error_metric_vals = [all_error_metric_vals, [0]]

        time_spec = aero_settle_dict['Time Specification']    
        ms_times = [aerotech_settle_time, time_spec]
        
        if after_move_end == True:  
            start = int(np.average(self.move_end_ind))
            ms_label_locs = [i+0.015 for i in ms_times]
            end = np.where(np.array(time_vals) == time_spec)[0][0] + 50

        else: 
            start = 0
            ms_label_locs = [i/2 for i in ms_times]
            end = len(time_vals)-1

###Changing function so that it plots default step number, but also gives choice to plot average times and all error metric values 
       
        plt.plot(time_vals[start:end], plt_signal[start:end], linewidth = 10, label = sig_label)

    #Condition that checks if the length of the time_array at index 1 is not 1-this will fail if looking at a single file
        if len(error_metric_vals[1]) != 1: 
            for idx, val in enumerate(error_metric_vals): #For every step that is present
        
                #Plots every error metric signal. Only adds legend label to the last one that is plotted
                if idx == self.data_len-1: plt.plot(self.time_array[idx][start:end], val[start:end], linewidth = 5, linestyle = 'dashed', color = 'red', label = metric_label)
                else: plt.plot(self.time_array[idx][start:end], val[start:end], linewidth = 5, linestyle = 'dashed')

          
        else: 
        
            plt.plot(time_vals[start:end], error_metric_vals[0][start:end], linewidth = 5, linestyle = 'dashed', color = 'red', label = metric_label)
    
    
        #Adjusting the x-axis to stretch out the graph to minimze overlap between legend and plots
        plt.xlim(left = time_vals[start], right = time_vals[end])
        
        #Vertically stretching the graph to reduce overlap between the time annotations and the signal that is plotted
        min_val = np.min(plt_signal[start:end])
        max_val = np.max(plt_signal[start:end])

        if max_val > abs(min_val): limit = max_val
        else: limit = min_val 
         
        graph_bound = 1.5*limit    
        diff = graph_bound - limit
     
        if graph_bound >0: plt.ylim(top = graph_bound)
        else: plt.ylim(bottom = graph_bound) 
            
        
        ms_times = [aerotech_settle_time, time_spec]
        
        #Sets the colors of the move and settle times based off if the step passed or failed the time specification
        aero_color = 'green' 
        #if asme_settle_time > time_spec: asme_color = 'red'
        if aerotech_settle_time > time_spec: aero_color = 'red'
        
        #Vertical lines plotting where the time specification and the ASME and Aerotech move and settle times are located 
        plt.axvline(x = time_spec, label = 'Time Specification', color = 'k', linewidth = 6, linestyle = '-.')
        #plt.axvline(x = asme_settle_time, label = asme_settle_label, color = asme_color, linewidth = 5, linestyle = 'dashed')
        plt.axvline(x = aerotech_settle_time, label = aerotech_settle_label, color = aero_color, linewidth = 5, linestyle = 'dotted')
            
        divisors = [1.25, 1.8, 3.35] #For adjusting the spacing of the annocations
        diff_color = [aero_color, 'black'] #Colors of the annotations
          
        #Annotates where the time spec and move and settle times are from the start with respective colors 
        for j, k  in zip(ms_times, divisors):
            
            plt.annotate("", xy = (time_vals[start], (graph_bound-diff/k)), xytext=(j, graph_bound-diff/k), xycoords= "data",
                                  va="center", ha="center", size = 30, arrowprops=dict(arrowstyle="<-")) 
        
        #Separate for loop for labels, so that they are always in front of the vertical line markers
        for i, j, k, h  in zip(ms_label_locs, ms_times, divisors, diff_color):
             
            plt.annotate(str(j) + ' sec', xy=(i, graph_bound-diff/k), xycoords="data",
                                  va="center", ha="center", size = 30, color = h, bbox=dict(boxstyle="round", fc="w")) 
        
        
        #Annotation outside the graph with information related to the data analysis             
        # plt.annotate('Process Window: {} seconds ({})     Time Specification: {} sec     Settle Window: {} ({})'.format(
        #                     data_dict['Process Window (sec)'], data_dict['Process Window Direction'], 
        #                     time_spec, aero_settle_dict['Move and Settle Window'], self.units), 
        #                     xy = (0, -0.15), xycoords='axes fraction', size = 20)    
       
        #Configuring the plot. 
        #plt.title('{} with a {} second {} Process Window'.format(data_dict['Error Metric'][0], data_dict['Process Window (sec)'], data_dict['Process Window Direction']), fontsize = 20)
        
        plt.xlabel('Time (sec)', size = 40)
        plt.ylabel('Position Deviation ({})'.format(self.units), size = 40)
        plt.xticks(fontsize = 30)
        plt.yticks(fontsize = 30)
        plt.legend(loc = legend_loc, bbox_to_anchor=(1.01,1.01), fontsize = legend_size)  #bbox_to_anchor=(legend_loc_x, legend_loc_y)
        plt.tight_layout()
     
         
        return metric_plot


    

        
        
   


