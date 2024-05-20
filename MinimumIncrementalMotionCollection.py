# -*- coding: utf-8 -*-
"""
Created on Thu Jun 23 10:08:33 2022

@author: shunn
Modified by JStarr 12/28/23
Modified by TB on 01/08/2024
Deleted underscores from A1 commands
Modified the ai0 data collection to allow selecting which axis the external sensor feedback comes in on
"""
import automation1 as a1
import numpy as np
import time
import matplotlib.pyplot as plt
import sys
import math
import copy
from typing import Union
from operator import itemgetter
from itertools import groupby
import pdb

sys.path.append('../')

import a1data


class incremental_step(a1data.a1data):
    '''
    Represents a B5.64 compliant test procedure using the Automation 1 controller. When constructing an instance
    of this class, you specify the test parameters you want, then use the 'test' function to actually perform the
    incremental step test

    Parameters
    ----------
    axis : str
        The axis that the test will be run on.
    sample_rate : int
        Sample rate collected by the Automation 1 controller in Hz, must match up with one of the existing
        Automation 1 sample rate values.
    step_size : float
        Size of each step, units are the same as the controller parameter.
    s : float
        Standard deviation of the jitter, determined from a B5.64 In-Position Jitter test.
    t_ms : float
        The move-and-settle time, determined from a B5.64 Move-and-Settle test.
    sensitivity : float
        The sensitivity of the external position measurement sensor in units of self.units/V
    **kwargs : Optional Parameters
        units: str
            units of the automation 1 controller
        direction : a1data.mode
            Specify either a unidirectional test or a bidirectional test
        num_steps: int
            Number of steps in each direction. Must be greater than or equal to 10
        speed : float
            speed of the movement in units per second
        ramp_type : automation1.RampType
            Specify the type of ramp performed by the Automatiion 1 controller
        ramp_value : float
            Specify the acceleration of the move. Units are the same as the controller parameter.
        ramp_type_arg : float
            The ramping type additional argument for accelerations. 
            This is only used when $rampTypeAccel is RampType.SCurve and represents the s-curve percentage.
        t_ave : float
            The time in seconds between the settle of a step and the move of the next step. Must be at least 2 times the
            move-and-settle time, determined from a B5.64 Move-and-Settle test.
        start_pos : float
            The start position of the test in units
    Returns
    -------
    None.

    '''

    def __init__(self,axis,sample_rate, step_size, s, t_ms, sensitivity, probe_axis, num_steps, **kwargs):
        '''
        Represents a B5.64 compliant test procedure using the Automation 1 controller. When constructing an instance
        of this class, you specify the test parameters you want, then use the 'test' function to actually perform the
        incremental step test

        Parameters
        ----------
        axis : str
            The axis that the test will be run on.
        sample_rate : int
            Sample rate collected by the Automation 1 controller in Hz, must match up with one of the existing
            Automation 1 sample rate values.
        step_size : float
            Size of each step, units are the same as the controller parameter.
        s : float
            Standard deviation of the jitter, determined from a B5.64 In-Position Jitter test.
        t_ms : float
            The move-and-settle time, determined from a B5.64 Move-and-Settle test.
        sensitivity : float
            The sensitivity of the external position measurement sensor in units of self.units/V
        **kwargs : Optional Parameters
            units: str
                units of the automation 1 controller
            file : str
                Specifies the path of the file used to store the .csv files in self.test()
            direction : a1data.mode
                Specify either a unidirectional test or a bidirectional test
            num_steps: int
                Number of steps in each direction. Must be greater than or equal to 10
            speed : float
                speed of the movement in units per second
            ramp_type : automation1.RampType
                Specify the type of ramp performed by the Automatiion 1 controller
            ramp_value : float
                Specify the acceleration of the move. Units are the same as the controller parameter.
            ramp_type_arg : float
                The ramping type additional argument for accelerations. 
                This is only used when $rampTypeAccel is RampType.SCurve and represents the s-curve percentage.
            t_ave : float
                The time in seconds between the settle of a step and the move of the next step. Must be at least 2 times the
                move-and-settle time, determined from a B5.64 Move-and-Settle test.
            start_pos : float
                The start position of the test in units
        Returns
        -------
        None.

        '''
        #Necessary input arguments for the test to run
        super(incremental_step,self).__init__(axis, sample_rate,probe_axis, **kwargs)
        self.step_size = step_size
        self.s = s
        self.t_ms = t_ms
        self.sens = sensitivity
        self.probe_axis = probe_axis
        self.num_steps = num_steps
       
        
        #Input arguments that can be defined but may not be necessary
        default_kwargs = {'direction' : a1data.mode.Bidirectional,
                          #'num_steps' : 10,
                          't_ave' : 2 * t_ms,
                          'start_pos' : 0,
                          }
        
        #Update defaults if new values exist
        kwargs = {**default_kwargs, **kwargs}
        
        self.direction = kwargs['direction']
        #self.num_steps = kwargs['num_steps']
        self.t_ave = kwargs['t_ave']
        self.start_pos = kwargs['start_pos']
        
    def test(self, controller : a1.Controller):
        
        
        
        
        '''
        Represents the testing procedure of a B5.64 compliant incremental step test

        Parameters
        ----------
        controller : a1.Controller
            Active controller connection to Automation1

        Returns
        -------
        None.

        '''
        
        if self.direction == a1data.mode.Bidirectional:
            self.n = (int)(self.sample_rate*(self.t_ave+self.t_ms)*(np.add(self.num_steps,1))*2)
        else:
            self.n = (int)(self.sample_rate*(self.t_ave+self.t_ms)*(np.add(self.num_steps,1)))
                           
        data_config = super(incremental_step,self).test(controller)
        
        
        #Make sure step size and decrement are possible by encoder
        cpu = controller.runtime.parameters.axes.__getitem__(self.axis).units.countsperunit.value 
        self.step_size = ((int)(self.step_size*cpu))/cpu
        
        
        #Move such that the start point can be approached from the positive direction
        controller.runtime.commands.motion.movelinear(self.axis, [self.start_pos - self.step_size], self.speed) 
        time.sleep(self.t_ave)
        
        #Move to start position
        controller.runtime.commands.motion.movelinear(self.axis, [self.start_pos], self.speed) 
        time.sleep(self.t_ms + self.t_ave)
        
        #start data collection
        controller.runtime.data_collection.start(a1.DataCollectionMode.Snapshot, data_config)
        time.sleep(self.t_ms + self.t_ave)
        
        
        #Forward steps
        for i in range(1, self.num_steps+1):
            controller.runtime.commands.motion.movelinear(self.axis, [self.start_pos + i * self.step_size], self.speed) 
            time.sleep(self.t_ms + self.t_ave)
        #Reverse direction steps
        if(self.direction == a1data.mode.Bidirectional):
            end_pos = self.start_pos+self.num_steps*self.step_size
            for i in range(1, self.num_steps + 1):
                controller.runtime.commands.motion.movelinear(self.axis, [end_pos - i*self.step_size], self.speed) 
                time.sleep(self.t_ms + self.t_ave)
        
        #controller.runtime.data_collection.stop()
        
        #Wait for the results to complete
        results = controller.runtime.data_collection.get_results(data_config, self.n)
        
        #Results as n length arrays with all of the data points collected
        self.populate(results)
        
        #If analog data is chosen, must be multiplied by the sensitivity        
        if self.probe_axis != 'None':
            self.ai0 = [e * self.sens for e in self.ai0]
        
        if self.probe_axis != 'None':
            #Checks to see if probe direction sense matches the encoder and flips the sign if not
            high=np.absolute(max(self.ai0)-self.ai0[0])
            low=np.absolute(min(self.ai0)-self.ai0[0])
            
            if low>high:
                self.ai0=np.multiply(-1,self.ai0)
            
        self.ai0=np.subtract(self.ai0,self.ai0[0])
            
        #Shifts the entire step window to match encoder feedback at the start position
        self.ai0 = self.ai0+self.pos_fbk[0]
          
        
    def step_window(self, **kwargs):
        '''
        Parameters
        ----------
        **kwargs : Optional Parameters
            vel_com_threshold: float
                Specifies what the minimum velocity command value is when determining the ranges where the stage is moving

         Returns
         -------
         step_dict : dict
             Dictionary returning the following: 
             
                 Entire Step Indices: the entire index ranges of the position command for each step used to find the entire position feedback
                
                 Windowed Step Indices: index ranges of the position command after the move and settle time, used to find the windowed position feedback.
                  
                 Entire Position Feedback: entire position feedback values for each step based off entire step indices
                  
                 Windowed Position Feedback: position feedback values after the move and settle time based off windowed step indices
                  

        '''
        default_kwargs = {'vel_com_threshold': 1e-6}
        kwargs = {**default_kwargs,**kwargs}
        
        vel_com_threshold = kwargs['vel_com_threshold']
        vel_com_len = len(self.vel_com)
                
        #Finds all the indices where the absolte velocity command is greater than the threshold value
        moving_indices = [idx for idx, val in enumerate(self.vel_com) if abs(val)>vel_com_threshold]
        
        moving_window = []
        
        #Groups all consecutive velocity commands greater than the threshold value together
        for k,g in groupby(enumerate(moving_indices),lambda x:x[0]-x[1]):
            group = (map(itemgetter(1),g))
            group = list(map(int,group))
            moving_window.append([group[0],group[-1]])

        #Calculates a datapoint length of the move and settle window
        window = int((self.t_ms)*self.sample_rate)   
        
        #Step begins after the last index in each moving_window range. Ends one index prior to the first index of the next moving window
        pos_com_indices =  [[i[1]+1, moving_window[idx+1][0]-1] if idx < len(moving_window)-1 else [i[1]+1, vel_com_len] for idx,i in enumerate(moving_window)]
        
        #Same logic to find the step indices, but accouting for the move and settle window
        windowed_pos_com_indices = [[w[0]+window, w[1]] for w in pos_com_indices]
        
        #Finds the entire and windowed position feedback based off the position command indices
        if self.probe_axis == 'None':
            entire_pos_fbk = [self.pos_fbk[i[0] : i[1]] for i in pos_com_indices]
            windowed_pos_fbk = [self.pos_fbk[i[0] : i[1]] for i in windowed_pos_com_indices]
        else:
            entire_ai0 = [self.ai0[i[0] : i[1]] for i in pos_com_indices]
            windowed_ai0 = [self.ai0[i[0] : i[1]] for i in windowed_pos_com_indices]

        if self.probe_axis == 'None':
            step_dict = {'Entire Position Feedback' :entire_pos_fbk,
                        'Windowed Position Feedback': windowed_pos_fbk,
                        'Entire Step Indices': pos_com_indices,
                        'Windowed Step Indices': windowed_pos_com_indices,}
        else:
            step_dict = {'Entire ai0' :entire_ai0,
                        'Windowed ai0' :windowed_ai0,
                        'Entire Step Indices': pos_com_indices,
                        'Windowed Step Indices': windowed_pos_com_indices,}
        return step_dict    
    
    def data_analysis(self, step_reversal_error = None, remove_steps= None) -> dict:
        '''
        Represents the data analyis section of the Incremental Step Test, as specified in the B5.64. 

        Parameters
        ----------
        step_reversal_error : float, optional
            Incremental step reversal error that can be found and optionally passed after carrying out the Step Reversal Test
        remove_steps : int, optional
            Specified number of beginning reverse steps to disregard when conducting the data analysis. 

        Returns
        -------
        data_analysis_vals : dict
            Dictionary of the following ASME B5.64 metric values (will return 0 for reverse metrics if a unidirecitonal test is run):
                Forward Incremental Steps: list of forward incremental step sizes for each window based off the mean forward steps  
                
                Reverse Incremental Steps: list of reverse incremental step sizes for each window based off the mean reverse steps  
                
                Forward Sample Mean: sample mean of the forward incremental steps
                
                Reverse Sample Mean: sample mean of the reverse incremental steps
                
                Forward Sample Standard Deviation: sample standard deviation of forward incremental steps
                
                Reverse Sample Standard Deviation: sample standard deviation of reverse incremental steps
                
                Combined Sample Mean: sample mean of both the forward and reverse incremental steps
                
                Combined Sample Standard Deviation: sample standard deviation of both the forward and reverse steps

                Entire Step Indices: the entire index ranges of the position command for each step used to find the entire position feedback
                
                Windowed Step Indices: position feedback values after accoutning for the move and settle time
                
                Entire Position Feedback: entire position feedback values for each step based off entire step indices
                
                Windowed Position Feedback: position feedback values after accounting for the move and settle time based off windowed step indices
                
                Forward Steps: list of each range of the position feedback in each forward step
                
                Reverse Steps: list of each range of the position feedbac in each reverse step (if applicable)
                
                
        '''
        
        #Calling step windows function to get windowed position feedback lists
        #Windowed refers to the time during t_ave
        if self.probe_axis == 'None':
            step_dictionary = incremental_step.step_window(self)
            windowed_pos_fbk = step_dictionary['Windowed Position Feedback']
        else:
            step_dictionary = incremental_step.step_window(self)
            windowed_ai0 = step_dictionary['Windowed ai0']

        #Creating empty arrays for the calculations
        mean_forward_step_pos, mean_reverse_step_pos, forward_incremental_step, reverse_incremental_step= [[] for i in range(4)]
       
        #If unidirectional test is chosen
        if (self.direction == a1data.mode.Unidirectional):
             
            #No reverse steps, all step ranges in windowed_pos_fbk are forward steps                 
            if self.probe_axis == 'None':
                forward_steps = windowed_pos_fbk
                reverse_steps = [0]
            else:
                forward_steps = windowed_ai0
                reverse_steps = [0]
            
            #Equation 7.4.1/7.4.2 assumption for first step
            mean_forward_step_pos = [np.average(i) for i in forward_steps]
            
            #First forward incremental step must be relative to the start position
            forward_incremental_step = [mean_forward_step_pos[0] - abs(self.start_pos) if i == 0 else mean_forward_step_pos[i]-mean_forward_step_pos[i-1] for i in range(len(mean_forward_step_pos))]
                   
        
                
        elif (self.direction == a1data.mode.Bidirectional):
            #Number of forward and reverse steps are equal
            if self.probe_axis == 'None':
                midpoint = int(len(windowed_pos_fbk)/2)
            else:
                midpoint = int(len(windowed_ai0)/2)
           
            #Forward steps data is from beginning to midpoint of windowed_pos_fbk list, while reverse is from midpoint to end
            if self.probe_axis == 'None':
                forward_steps = windowed_pos_fbk[:midpoint]
                reverse_steps = windowed_pos_fbk[midpoint:]
            else:
                forward_steps = windowed_ai0[:midpoint]
                reverse_steps = windowed_ai0[midpoint:]

            #Equation 7.4.1/7.4.2 assumption for first step, absolute location of steps
            mean_forward_step_pos = [abs(np.average(i)) for i in forward_steps]
            mean_reverse_step_pos = [abs(np.average(j)) for j in reverse_steps]
            
            #Relative location of steps from the previous step and the start position for the first forward incremental step
            forward_incremental_step = [abs(mean_forward_step_pos[1] - abs(mean_forward_step_pos[0])) if i == 0 else abs(mean_forward_step_pos[i]-mean_forward_step_pos[i-1]) for i in range(len(mean_forward_step_pos))]
            reverse_incremental_step = [abs(mean_reverse_step_pos[0]-mean_forward_step_pos[-1]) if j == 0 else abs(mean_reverse_step_pos[j]-mean_reverse_step_pos[j-1]) for j in range(len(mean_reverse_step_pos))]
           
        #Checks if user entered step reversal error or a number of reverse steps to neglect 
        if step_reversal_error is not None:
            n = math.ceil((step_reversal_error/self.step_size))

        elif remove_steps is not None:
            n = remove_steps
        else: n = 0
            
        #Calculates number of reverse steps to use based off remove_steps input 
        num_reverse_steps = self.num_steps-n
        reverse_incremental_step = reverse_incremental_step[n:]


        #Equation 7.4.4/7.4.5: forward and reverse sample mean calculation
        forward_sample_mean = sum(forward_incremental_step)/self.num_steps
        reverse_sample_mean = sum(reverse_incremental_step)/num_reverse_steps
        
        
        #Equation 7.4.6/7.4.7: forward and reverse sample standard deviation
        forward_sum_dif_sq = sum([(x-forward_sample_mean)**2 for x in forward_incremental_step])
        reverse_sum_dif_sq = sum([(x-reverse_sample_mean)**2 for x in reverse_incremental_step]) 
        
        
        forward_sample_std = math.sqrt((1/(self.num_steps-1))*forward_sum_dif_sq)
        reverse_sample_std = math.sqrt((1/(num_reverse_steps-1))*reverse_sum_dif_sq)
              
        
        #Equation 7.4.8: combined sample mean 
        combined_sample_mean = (1/(self.num_steps + num_reverse_steps))*(sum(forward_incremental_step)+sum(reverse_incremental_step)) 
        
        #Equation 7.4.9: combined sample standard deviation
        combined_sum_dif_sq = sum([(x-combined_sample_mean)**2 for x in forward_incremental_step])+ sum([(x-combined_sample_mean)**2 for x in reverse_incremental_step]) 
        combined_sample_std = math.sqrt((1/(self.num_steps + num_reverse_steps-1))*combined_sum_dif_sq)
        
        #"Unwindowed" ranges refer to the data in each step including t_ms and t_ave
        #"Windowed" ranges refer to the part of each step during t_ave

        if self.probe_axis == 'None':
            data_analysis_vals = {'Forward Incremental Steps': forward_incremental_step,
                                'Reverse Incremental Steps': reverse_incremental_step,
                                'Forward Sample Mean': forward_sample_mean,
                                'Reverse Sample Mean': reverse_sample_mean,
                                'Forward Sample Standard Deviation': forward_sample_std,
                                'Reverse Sample Standard Deviation': reverse_sample_std,
                                'Combined Sample Mean': combined_sample_mean,
                                'Combined Sample Standard Deviation': combined_sample_std,
                                'Entire Step Indices': step_dictionary['Entire Step Indices'],
                                'Windowed Step Indices': step_dictionary['Windowed Step Indices'],
                                'Entire Position Feedback': step_dictionary['Entire Position Feedback'],
                                'Windowed Position Feedback': windowed_pos_fbk,
                                'Forward Steps': forward_steps,
                                'Reverse Steps': reverse_steps}
        else:
            data_analysis_vals = {'Forward Incremental Steps': forward_incremental_step,
                                'Reverse Incremental Steps': reverse_incremental_step,
                                'Forward Sample Mean': forward_sample_mean,
                                'Reverse Sample Mean': reverse_sample_mean,
                                'Forward Sample Standard Deviation': forward_sample_std,
                                'Reverse Sample Standard Deviation': reverse_sample_std,
                                'Combined Sample Mean': combined_sample_mean,
                                'Combined Sample Standard Deviation': combined_sample_std,
                                'Entire Step Indices': step_dictionary['Entire Step Indices'],
                                'Windowed Step Indices': step_dictionary['Windowed Step Indices'],
                                'Windowed ai0': windowed_ai0,
                                'Forward Steps': forward_steps,
                                'Reverse Steps': reverse_steps}
        
        return data_analysis_vals
    
    def plot(self, **kwargs):
        
        #Default kwargs if none are specified
        default_kwargs = {'step_num': 0,
                          'fignum': 1,
                          'legend_loc': 'best', 
                          'legend_size': 15.0,
                          'fig_size': (14,5)
                           }
        
        #Updates kwargs in case some are specified 
        kwargs = {**default_kwargs,**kwargs}
        step_num = kwargs['step_num']
        fignum = kwargs['fignum']
        legend_loc = kwargs['legend_loc']
        legend_size = kwargs['legend_size']
        fig_size = kwargs['fig_size']
        
        #Step label for title
        if ((self.direction == a1data.mode.Bidirectional) & (step_num > self.num_steps)):
            step_label = 'Reverse Step {}'.format(step_num-self.num_steps)
        else: step_label = 'Forward Step {}'.format(step_num)
        
        #Calling step_window() and extracting the entire and windowed step indices 
        step_window_dict = self.step_window()
        entire_steps = step_window_dict['Entire Step Indices']
        windowed_steps = step_window_dict['Windowed Step Indices']
        
        #Average and move and settle time range
        t_ave_range = windowed_steps[step_num]
        t_ms_range = [entire_steps[step_num][0], t_ave_range[0]]
      
        #Offset to look at the previous and next step on the graph
        graph_offset = 150
        
        #Creating new time and position feedback arrays from offset, average time, and move and settle time
        if self.probe_axis == 'None':
            time = self.time_array[t_ms_range[0] - graph_offset: t_ave_range[1] + int(graph_offset/15)]
            pos_fbk = self.pos_fbk[t_ms_range[0] - graph_offset: t_ave_range[1] + int(graph_offset/15)]
        else:
            time = self.time_array[t_ms_range[0] - graph_offset: t_ave_range[1] + int(graph_offset/15)]
            ai0 = self.ai0[t_ms_range[0] - graph_offset: t_ave_range[1] + int(graph_offset/15)]
    
        #Time array with the duration of the offset
        offset_time = t_ms_range[0]-(t_ms_range[0]-graph_offset)
        
        #Finding height bounds based off average postion during offset and the postion feedback when average time window begins
        if self.probe_axis == 'None':
            first_height = np.average(pos_fbk[:offset_time]) 
            second_height = self.pos_fbk[t_ave_range[0]] 
        else:
            first_height = np.average(ai0[:offset_time])
            second_height = self.ai0[t_ave_range[0]]
 
        #Determining placement of annotations based off heights
        if abs(first_height) < (second_height):
            lower_height = first_height
            upper_height = second_height 
            divisor = 4
        else:
            lower_height = second_height
            upper_height = first_height
            divisor = 1.5
       
        #Midpoint y-coordinate for the move and settle time annotation 
        ms_time_y_coords, avg_time_y_coords = (lower_height + (upper_height-lower_height)/divisor), first_height
     
        #Labels, colors, linewidths for each time
        labels = ['Step Start', 'Move and Settle Time End', 'Step End']
        colors = ['black', 'red', 'green']
        width = [1.5, 2.5, 3.5]
        
        #Time markers containing the average and move and settle time, and an array containing the difference between each 
        time_markers = [t_ms_range[0], t_ave_range[0], t_ave_range[1]-1]
        time_diff = np.diff(time_markers)
        
        #Plot creation with time and position feedback
        if self.probe_axis == 'None': 
            fig, ax =  plt.subplots(figsize = fig_size) 
            ax.plot(time, pos_fbk, linewidth = 3)
        else:
            fig, ax =  plt.subplots(figsize = fig_size) 
            ax.plot(time, ai0, linewidth = 3)
        
        #Plots vertical lines for each time marker
        for t,l,c,w in zip(time_markers, labels, colors, width):
            ax.axvline(x = self.time_array[t], label = l, linewidth = w, color = c, linestyle = '--') 
        
        #Section for determining the pixel locations for placing the move and settle time annotations
        blank_avg = ax.annotate('', xy = (self.time_array[time_markers[1]+ int(time_diff[1]/2)], avg_time_y_coords), xycoords= "data",
                              va="center", ha="center",  size = 20) 
        blank_ms = ax.annotate('    ', xy = (self.time_array[time_markers[1]], avg_time_y_coords), xycoords= "data",
                              va="center", ha="center",  size = 20) 
        
        fig.canvas.draw()
        bbox_avg = blank_avg.get_window_extent().transformed(ax.transData.inverted()) 
                                                
        bbox_ms = blank_ms.get_window_extent().transformed(ax.transData.inverted()) 
         
        #Pixel x-locations of average time label
        avg_time_coords_x = [[bbox_avg.extents[0], bbox_avg.extents[2]],  bbox_avg.extents[2]- bbox_avg.extents[0]]
        
        #Pixel x-locations of move and settle time label
        ms_time_coords_x = [[bbox_ms.extents[0], bbox_ms.extents[2]],  4*(bbox_ms.extents[2]- bbox_ms.extents[0])]
        
        #Average time double arrow annotation
        ax.annotate("", xy = (self.time_array[t_ave_range[0]], first_height),  xytext=(self.time_array[t_ave_range[1]-1], avg_time_y_coords), xycoords= "data",
                              va="center", ha="center", size = 20, arrowprops=dict(arrowstyle="<->")) 
     
        
        #Average time text annotation
        ax.annotate(r'$t_{ave}$ = ' +'{} sec'.format(self.t_ave), xy = (avg_time_coords_x[0][0] + avg_time_coords_x[1]/2, avg_time_y_coords), xycoords= "data", 
                              va="center", ha="center", bbox=dict(boxstyle="round", fc="w"), size = 20)
        
        #Move and Settle time double arrow annotation
        ax.annotate("", xy = (self.time_array[t_ms_range[0]], ms_time_y_coords),  xytext=(self.time_array[t_ms_range[1]], ms_time_y_coords), xycoords= "data", 
                              va="center", ha="center", size = 20, arrowprops=dict(arrowstyle="<->")) 
        
        #Move and Settle time text annotation
        ax.annotate(r'$t_{ms}$ = ' +'{} sec'.format(self.t_ms), xy = (ms_time_coords_x[0][0] + ms_time_coords_x[1], ms_time_y_coords), xycoords= "data", 
                              va="center", ha="center", bbox=dict(boxstyle="round", fc="w"), size = 20)

        if self.probe_axis == 'None':
            plt.title('Average and Move and Settle Time Comparison for {}'.format(step_label), size = 20)
            plt.xlabel('Time (sec)', size = 20)
            plt.ylabel('Position Feedback ({})'.format(self.units), size = 20)
            plt.xticks(fontsize = 18)
            plt.yticks(fontsize = 18)
            plt.legend(loc = legend_loc, bbox_to_anchor=(1.01,1.01), fontsize = legend_size)
        else:
            plt.title('Average and Move and Settle Time Comparison for {}'.format(step_label), size = 20)
            plt.xlabel('Time (sec)', size = 20)
            plt.ylabel('Analog Input ({})'.format(self.units), size = 20)
            plt.xticks(fontsize = 18)
            plt.yticks(fontsize = 18)
            plt.legend(loc = legend_loc, bbox_to_anchor=(1.01,1.01), fontsize = legend_size)            
        return fig




    
    def unidirectional_criteria(self, data, *directions : a1data.mode):
        return self.A1() & self.A2(data, *directions) & self.A3(data, *directions)
    
    def A1(self):
        #Criterion A1: 4*(standard deviation of in-position jitter) <incremental step
        criterion_A1 = 4*self.s < np.abs(self.step_size)
        # if(criterion_A1):
        #     print('A1 passed')
        # else:
        #     print('A1 failed')
        return criterion_A1
    
    def A2(self, data_analysis : dict, *directions : a1data.mode):
        criterion_A2 = True
        for direction in directions:
            if (direction == a1data.mode.positive_direction):
                #Criterion A2: sample mean of unidirectional compared to commanded step must be 10% less than commanded step
                criterion_A2 &= (abs(self.step_size - data_analysis['Forward Sample Mean'])/self.step_size) < 0.1   
            elif (direction == a1data.mode.negative_direction):
                #Criterion A2: sample mean of unidirectional compared to commanded step must be 10% less than commanded step
                criterion_A2 &= (abs(self.step_size - data_analysis['Reverse Sample Mean'])/self.step_size) < 0.1
            else:
                raise ValueError('Please specify either positive_direction or negative_direction')
      
        # if(criterion_A2):
        #     print('A2 passed')
        # else:
        #     print('A2 failed')
        
        return criterion_A2
        
    def A3(self, data_analysis, *directions : a1data.mode):
        criterion_A3 = True
        for direction in directions:
            if (direction == a1data.mode.positive_direction):
                #Criterion A3: 4*(standard deviation of undirectional step) < commanded step
                criterion_A3 &= 4*data_analysis['Forward Sample Standard Deviation'] < self.step_size
            elif (direction == a1data.mode.negative_direction): 
                #Criterion A3: 4*(standard deviation of undirectional step) < commanded step
                criterion_A3 &= 4*data_analysis['Reverse Sample Standard Deviation'] < self.step_size
                
            else:
                raise ValueError('Please specify either positive_direction or negative_direction')
        
        # if(criterion_A3):
        #     print('A3 passed')
        # else:
        #     print('A3 failed')
        
        return criterion_A3
    
    def bidirectional_criteria(self, data):
        return self.B1() & self.B2(data) & self.B3(data)
        
    def B1(self):
        #Criterion B1: 4*(standard deviation of in-position jitter) < commanded incremental step
        criterion_B1 = 4*self.s < self.step_size
        
        # if(criterion_B1):
        #     print('B1 passed')
        # else:
        #     print('B1 failed')
            
        return(criterion_B1)
    
    def B2(self,data_analysis):
        #Criterion B2: maximum sample mean of both directions must be less than 10% of commanded step
        forward_B2 = (abs(self.step_size - data_analysis['Forward Sample Mean'])/self.step_size)
        reverse_B2 = (abs(self.step_size - data_analysis['Reverse Sample Mean'])/self.step_size)
        criterion_B2 = max(forward_B2, reverse_B2) < 0.1
        
        # if(criterion_B2):
        #     print('B2 passed')
        # else:
        #     print('B2 failed')

        return criterion_B2
    
    def B3(self, data_analysis):
        #Criterion B3: 4*(max standard deviation between both directions) must be less than commanded step
        criterion_B3 = 4*max(data_analysis['Forward Sample Standard Deviation'], data_analysis['Reverse Sample Standard Deviation']) < self.step_size
        
        # if(criterion_B3):
        #     print('B3 passed')
        # else:
        #     print('B3 failed')
            
        return criterion_B3
        
        
        
   
        
        
class step_reversal(incremental_step):
    '''
    Represents a B5.64 compliant test procedure using the Automation 1 controller. When constructing an instance
    of this class, you specify the test parameters you want, then use the 'test' function to actually perform the
    step reversal error test

    Parameters
    ----------
    axis : str
        The axis that the test will be run on.
    sample_rate : int
        Sample rate collected by the Automation 1 controller in Hz, must match up with one of the existing
        Automation 1 sample rate values.
    step_size : float
        Size of each step, units are the same as the controller parameter.
    s : float
        Standard deviation of the jitter, determined from a B5.64 In-Position Jitter test.
    t_ms : float
        The move-and-settle time, determined from a B5.64 Move-and-Settle test.
    sensitivity : float
        The sensitivity of the external position measurement sensor in units of self.units/V
    **kwargs : Optional Parameters
        units: str
            units of the automation 1 controller
        direction : a1data.mode
            Specify either a unidirectional test or a bidirectional test
        num_steps: int
            Number of steps in each direction. Must be greater than or equal to 10
        speed : float
            speed of the movement in units per second
        ramp_type : automation1.RampType
            Specify the type of ramp performed by the Automatiion 1 controller
        ramp_value : float
            Specify the acceleration of the move. Units are the same as the controller parameter.
        ramp_type_arg : float
            The ramping type additional argument for accelerations. 
            This is only used when $rampTypeAccel is RampType.SCurve and represents the s-curve percentage.
        t_ave : float
            The time in seconds between the settle of a step and the move of the next step. Must be at least 2 times the
            move-and-settle time, determined from a B5.64 Move-and-Settle test.
        start_pos : float
            The start position of the test in units
        decrement : float
            The amount in units that the step size should decrease by each step taken

    Returns
    -------
    None.

    '''
    def __init__(self,axis,sample_rate, step_size, s, t_ms, sensitivity, probe_axis, num_steps, **kwargs):
        '''
        Represents a B5.64 compliant test procedure using the Automation 1 controller. When constructing an instance
        of this class, you specify the test parameters you want, then use the 'test' function to actually perform the
        step reversal error test

        Parameters
        ----------
        axis : str
            The axis that the test will be run on.
        sample_rate : int
            Sample rate collected by the Automation 1 controller in Hz, must match up with one of the existing
            Automation 1 sample rate values.
        step_size : float
            Size of each step, units are the same as the controller parameter.
        s : float
            Standard deviation of the jitter, determined from a B5.64 In-Position Jitter test.
        t_ms : float
            The move-and-settle time, determined from a B5.64 Move-and-Settle test.
        sensitivity : float
            The sensitivity of the external position measurement sensor in units of self.units/V
        **kwargs : Optional Parameters
            units: str
                units of the automation 1 controller
            direction : a1data.mode
                Specify either a unidirectional test or a bidirectional test
            num_steps: int
                Number of steps in each direction. Must be greater than or equal to 10
            speed : float
                speed of the movement in units per second
            ramp_type : automation1.RampType
                Specify the type of ramp performed by the Automatiion 1 controller
            ramp_value : float
                Specify the acceleration of the move. Units are the same as the controller parameter.
            ramp_type_arg : float
                The ramping type additional argument for accelerations. 
                This is only used when $rampTypeAccel is RampType.SCurve and represents the s-curve percentage.
            t_ave : float
                The time in seconds between the settle of a step and the move of the next step. Must be at least 2 times the
                move-and-settle time, determined from a B5.64 Move-and-Settle test.
            start_pos : float
                The start position of the test in units
            decrement : float
                The amount in units that the step size should decrease by each step taken

        Returns
        -------
        None.

        '''
        super(step_reversal,self).__init__(axis,sample_rate, step_size, s, t_ms, sensitivity, probe_axis, num_steps, **kwargs)
        
        default_kwargs = {'decrement' : .05*self.step_size,
                          'window_mult' : 3}
        
        kwargs = {**default_kwargs, **kwargs}
        
        self.decrement = kwargs['decrement']
        self.window_mult = 3 #Window size scale factor
        self.probe_axis = probe_axis
        self.num_steps = num_steps
        
        #Set direction to unidriectional for the data_analysis function to work properly
        self.direction = a1data.mode.Unidirectional
        
    def test(self, controller : a1.Controller):
        '''
        Represents the test procedure of a B5.64 compliant step reversal test

        Parameters
        ----------
        controller : a1.Controller
            Active controller connection to Automation1

        Returns
        -------
        B_inc : float
            Step Reversal Error as per ASME B5.64

        '''
        
        self.n = (int)(self.sample_rate*self.num_steps*(self.t_ave+self.t_ms)*self.window_mult)
        
        
        data_config = super(incremental_step,self).test(controller)
        
        #Move such that the start point can be approached from the positive direction
        controller.runtime.commands.motion.movelinear(self.axis, [self.start_pos - self.step_size], self.speed) 
        time.sleep(self.t_ave + self.t_ms)
        
        #Move to start position
        controller.runtime.commands.motion.movelinear(self.axis, [self.start_pos], self.speed) 
        time.sleep(self.t_ms + self.t_ave)
        
        #start data collection
        controller.runtime.data_collection.start(a1.DataCollectionMode.Snapshot, data_config)
        
        #Make sure step size and decrement are possible by encoder
        cpu = controller.runtime.parameters.axes.__getitem__(self.axis).units.countsperunit.value 

        self.step_size = ((int)(self.step_size*cpu))/cpu
        self.num_steps = (int)(self.step_size / self.decrement)
        
        step = copy.deepcopy(self.step_size)

        
        #Switch to relative motion
        controller.runtime.commands.motion_setup.setuptasktargetmode(a1.TargetMode.Incremental)
        for i in range(0, self.num_steps):
            time.sleep(self.t_ms + self.t_ave)
            controller.runtime.commands.motion.movelinear(self.axis, [step], self.speed)
            step = ((-1)**(i+1))*(abs(step)-self.decrement)
            
            #Make sure step size is possible by the counts per unit parameter
            step = ((int)(step*cpu))/cpu

        
        #Switch to absolute motion
        controller.runtime.commands.motion_setup.setuptasktargetmode(a1.TargetMode.Absolute) 
 
        
        results = controller.runtime.data_collection.get_results(data_config, self.n)
        
        self.populate(results)
        
        data = self.data_analysis()
        
        if any(element < 0.50 for element in data['step_error']): #If there's an element in step_error less than 50%
            index = np.argmin(np.abs(np.abs(np.array(data['step_error']))-.5)) #Index of the error closest to 0.5
            return abs(data['x_cs'][index]/2)
        else:
            self.step_size *= 1.1 #Increase step size by 10%
            return self.test(controller) #TRY AGAIN
             
    def data_analysis(self):
        ########
        sr_start = []
        
        if self.probe_axis != 'None':
            self.ai0 = [e * self.sens for e in self.ai0]
        
        
        if self.probe_axis != 'None':
            #Checks to see if probe direction sense matches the encoder and flips the sign if not
            high=np.absolute(max(self.ai0)-self.ai0[0])
            low=np.absolute(min(self.ai0)-self.ai0[0])
            
            if low>high:
                self.ai0=np.multiply(-1,self.ai0)
            
            #Averages the points in the first second of data collection to establish probe feedback offset from zero
            for i, e in enumerate(self.time_array):
                if (e >= 0) & (e < self.t_ave):
                    sr_start.append(e)
            avg_sr_start = np.average(sr_start)
            print(avg_sr_start)
            
            #Shifts the entire step window to be centered about zero
            if avg_sr_start > 0:
                self.ai0=self.ai0-np.absolute(avg_sr_start)
            elif avg_sr_start < 0:
                self.ai0=self.ai0+np.absolute(avg_sr_start)
            
            #Shifts the entire step window to match encoder feedback at the start position
            self.ai0 = self.ai0+self.pos_fbk[0]
            
        ########
        step = copy.deepcopy(self.step_size)
        num_cycles = (int)(self.step_size / self.decrement)
        
        X_cs = [self.start_pos]
        
        #Generate list of all of the commanded steps
        for i in range(0, num_cycles):
            X_cs.append(round(X_cs[-1] + step,6))
            step = ((-1)**(i+1))*(abs(step)-self.decrement)

        
        
        # #Find Index of the first instance of a specific position command
        # start_indices = np.empty(len(X_cs))
        # for j, X_csi in enumerate(X_cs):
        #     for i, e in enumerate(self.pos_com):
        #         if abs(e - X_csi) < .00001:
        #             #start_indices[j] = (i, e)
        #             start_indices[j] = i
        #             break
        # start_indices = np.delete(start_indices, 0).astype(int)
        
        # #Start indices contains the first value where the velocity switches from 0 to some number, then ignores the rest
        # #in that block
        # start_indices = np.empty(len(X_cs))
        # j = 0
        # for i in range(len(start_indices)):
        #     if abs(self.vel_com[i]) > .01:
        #         start_indices[j] = i
        #         j += 1
        #     while abs(self.vel_com[i]) > .01:
        #         i += 1
    
        #delay by n_t_ms datapoints

        # start_indices += n_t_ms
        # end_indices = start_indices + n_t_ave
        
        
        #Use stepwindow function in incremental step class to determine the start indices
        step_dict = self.step_window()
        buckets = step_dict['Windowed Step Indices']
        
        #Deprecated way to determine windows
        #buckets = list(zip(start_indices,end_indices))


        #Create a blank X_inc array. Iterate through buckets and put the average pos_fbk in X_inc
        #Needs to be an external sensor stored in Analog input 0 if performed actually
        X_inc = []
        for b in buckets:
            ##########TB
            if self.probe_axis == 'None':
                X_inc.append(np.mean(self.pos_fbk[b[0]:b[1]]))
            else:
                X_inc.append(np.mean(self.ai0[b[0]:b[1]]))
            ##########TB
        X_cs.pop(0)
        
        #Need to convert from absolute commands to relative commands
        X_cs_rel = np.empty(len(X_cs))
        X_inc_rel = np.empty(len(X_inc))
        for i, e in enumerate(zip(X_cs,X_inc)):
            if i == 0:
                X_cs_rel[i] = X_cs[i]
                X_inc_rel[i] = X_inc[i]
            else:
                X_cs_rel[i] = X_cs[i] - X_cs[i-1]
                X_inc_rel[i] = X_inc[i] - X_inc[i-1]
        
        
        step_error = []
        for e in zip(X_cs_rel,X_inc_rel):
            step_error.append(abs(e[0]-e[1])/e[0])
        
        data_dict = {'x_cs' : X_cs_rel,
                     'x_inc' : X_inc_rel,
                     'step_error':step_error,
            }
        return data_dict
    
    # def plot(self):
    #     super(step_reversal,self).plot()
    #     plt.plot(self.time_array,self.pos_com, '-b')

    def B_inc(self):
        data = self.data_analysis()
        
        if any(element < 0.50 for element in data['step_error']): #If there's an element in step_error less than 50%
            index = np.argmin(np.abs(np.abs(np.array(data['step_error']))-.5)) #Index of the error closest to 0.5
            return abs(data['x_cs'][index]/2)
        
    def plot(self, *args):
        super(incremental_step,self).plot(*args)
    
    
    
class min_incremental_motion(incremental_step):
    '''
    Represents a B5.64 compliant test procedure using the Automation 1 controller. When constructing an instance
    of this class, you specify the test parameters you want, then use the 'test' function to actually perform the
    minimum incremental motion test

    Parameters
    ----------
    axis : str
        The axis that the test will be run on.
    sample_rate : int
        Sample rate collected by the Automation 1 controller in Hz, must match up with one of the existing Automation 1 sample rate values.
    step_size : float
        Size of each step, units are the same as the controller parameter.
    s : float
        Standard deviation of the jitter, determined from a B5.64 In-Position Jitter test.
    t_ms : float
        The move-and-settle time, determined from a B5.64 Move-and-Settle test.
    sensitivity : float
        The sensitivity of the external position measurement sensor in units of self.units/V
    machine_res : float
        The resolution of the machine's encoder in units of self.units
    step_reversal : step_reversal
        Non-initialized parameter referencing a B5.64 compliant step reversal test in the context of the larger
        minimum incremental motion test
    **kwargs : Optional Parameters
        units: str
            units of the automation 1 controller
        direction : a1data.mode
            Specify either a unidirectional test or a bidirectional test
        num_steps: int
            Number of steps in each direction. Must be greater than or equal to 10
        speed : float
            speed of the movement in units per second
        ramp_type : automation1.RampType
            Specify the type of ramp performed by the Automatiion 1 controller
        ramp_value : float
            Specify the acceleration of the move. Units are the same as the controller parameter.
        ramp_type_arg : float
            The ramping type additional argument for accelerations. 
            This is only used when $rampTypeAccel is RampType.SCurve and represents the s-curve percentage.
        t_ave : float
            The time in seconds between the settle of a step and the move of the next step. Must be at least 2 times the
            move-and-settle time, determined from a B5.64 Move-and-Settle test.
        start_pos : float
            The start position of the test in units
        decrement : float
            The amount in units that the step size should decrease by each step taken

    Returns
    -------
    None.
    '''
    def __init__(self,axis,sample_rate, step_size, s, t_ms, sensitivity, machine_res, **kwargs):
        '''
        Represents a B5.64 compliant test procedure using the Automation 1 controller. When constructing an instance
        of this class, you specify the test parameters you want, then use the 'test' function to actually perform the
        minimum incremental motion test

        Parameters
        ----------
        axis : str
            The axis that the test will be run on.
        sample_rate : int
            Sample rate collected by the Automation 1 controller in Hz, must match up with one of the existing Automation 1 sample rate values.
        step_size : float
            Size of each step, units are the same as the controller parameter.
        s : float
            Standard deviation of the jitter, determined from a B5.64 In-Position Jitter test.
        t_ms : float
            The move-and-settle time, determined from a B5.64 Move-and-Settle test.
        sensitivity : float
            The sensitivity of the external position measurement sensor in units of self.units/V
        machine_res : float
            The resolution of the machine's encoder in units of self.units
        **kwargs : Optional Parameters
            units: str
                units of the automation 1 controller
            direction : a1data.mode
                Specify either a unidirectional test or a bidirectional test
            num_steps: int
                Number of steps in each direction. Must be greater than or equal to 10
            speed : float
                speed of the movement in units per second
            ramp_type : automation1.RampType
                Specify the type of ramp performed by the Automatiion 1 controller
            ramp_value : float
                Specify the acceleration of the move. Units are the same as the controller parameter.
            ramp_type_arg : float
                The ramping type additional argument for accelerations. 
                This is only used when $rampTypeAccel is RampType.SCurve and represents the s-curve percentage.
            t_ave : float
                The time in seconds between the settle of a step and the move of the next step. Must be at least 2 times the
                move-and-settle time, determined from a B5.64 Move-and-Settle test.
            start_pos : float
                The start position of the test in units
            decrement : float
                The amount in units that the step size should decrease by each step taken

        Returns
        -------
        None.

        '''
        super(min_incremental_motion,self).__init__(axis,sample_rate, step_size, s, t_ms, sensitivity, **kwargs)
        
        #Starting step size is max of 2 * s and machine resolution BE SURE TO CHANGE BACK
        self.step_size = max(4*self.s,machine_res)
        
        #This test must be bidirectional no matter what
        self.direction = a1data.mode.Bidirectional
        
        self.step_reversal = None #Have a step reversal error test be a property of this test.
        
    def test(self, controller : a1.Controller, remove_backlash: Union[bool, int] = False):
        '''
        Represents the test procedure for a B5.64 compliant minimum incremental motion test

        Parameters
        ----------
        controller : a1.Controller
            Active controller connection to Automation1
        remove_backlash : Union[bool,int] , optional
            True  if backlash affected steps should be removed from the reverse steps. 
            An int to define the number of reverse steps that should be removed.
            The default is False.

        Returns
        -------
        Dictionary
            Dictionary contains the minimum incremental step size in units of self.units.
            If a step reversal error test was run, the step reversal error in units of self.units is also returned

        '''
        
        print('Running incremental step with a {} {} step size'.format(self.step_size, self.units))
        super(min_incremental_motion,self).test(controller)
        
        self.step_reversal = step_reversal(self.axis, self.sample_rate, self.step_size*2, self.s, self.t_ms, self.sens, units = self.units)
        
        #If user does not specify backlash to be removed, backlash is not removed
        if remove_backlash == False: 
            data = self.data_analysis()
        #Else if user specifies an amount of backlash steps to be removed, those steps are removed
        elif type(remove_backlash) == int:
            data = self.data_analysis(remove_steps = remove_backlash)
        #If user specifies they want backlash affected steps to be removed, that value is determined by a step reversal test, then removed
        else:
            #If backlash is to be removed, have to calculate the B value first
            print("Running step reversal")
            B = self.step_reversal.test(controller)
            data = self.data_analysis(step_reversal_error = B)
        
        if self.unidirectional_criteria(data): #if it passes unidirectional criteria
            if remove_backlash: #If backlash steps are removed, step reversal must be reported no matter what
                print("Done")
                return {'min step size' : self.step_size,
                        'step reversal' : B,
                        }
            elif self.bidirectional_criteria(data): #If the backlash steps are not removed and it passes bidirectionally, no step reversal needed
                print("Done")
                return {'min step size' : self.step_size}
            else:
                print("Running step reversal")
                B = self.step_reversal.test(controller)
                print("Done")
                return {'min step size' : self.step_size,
                        'step reversal' : B,
                        }
        else:
            #Adjust Step Size
            self.step_size *= 1.2
            self.step_reversal.step_size = 2*self.step_size
            
            #Run Move and Settle Test here?
            
            
            return self.test(controller, remove_backlash)
            
        
        
        
        
            
        
        
        