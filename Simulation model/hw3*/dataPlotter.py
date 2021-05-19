import matplotlib.pyplot as plt 
from matplotlib.lines import Line2D
import numpy as np

plt.ion()  # enable interactive drawing


class dataPlotter:
    ''' 
        This class plots the time histories for the vtol data.
    '''

    def __init__(self):
        # Number of subplots = num_of_rows*num_of_cols
        self.num_rows = 6    # Number of subplot rows
        self.num_cols = 1    # Number of subplot columns

        # Crete figure and axes handles
        self.fig, self.ax = plt.subplots(self.num_rows, self.num_cols, sharex=True)

        # Instantiate lists to hold the time and data histories
        self.time_history = []  # time
        self.zref_history = []  # reference position z_r
        self.zt_history = []  # position zt
        self.zv_history = []  # position zv
        self.h_history = []  # position h
        self.theta_history = []  # angle theta
        self.Force_left_history = []  # control force
        self.Force_right_history = []  # control force

        # create a handle for every subplot.
        self.handle = []
        self.handle.append(myPlot(self.ax[0], ylabel='zt', title='target Data'))
        self.handle.append(myPlot(self.ax[1], ylabel='zv', title='vtol distance'))
        self.handle.append(myPlot(self.ax[2], ylabel='h', title='vtol height'))
        self.handle.append(myPlot(self.ax[3], ylabel='theta(deg)', title='vtol angle'))
        self.handle.append(myPlot(self.ax[4], xlabel='t(s)', ylabel='force_left(N)',title='vtol thurst'))
        self.handle.append(myPlot(self.ax[5], xlabel='t(s)', ylabel='force_right(N)',title='vtol thurst'))

    def update(self, t, reference, states, ctrl1, ctrl2):
        '''
            Add to the time and data histories, and update the plots.
        '''
        # update the time history of all plot variables
        self.time_history.append(t)  # time
        self.zref_history.append(reference)  # reference base position
        self.zt_history.append(states.item(0))  # base position
        self.zv_history.append(states.item(1))
        self.h_history.append(states.item(2))
        self.theta_history.append(states.item(3))  # arm angle (converted to degrees)
        self.Force_left_history.append(ctrl1)  # force on the base
        self.Force_right_history.append(ctrl2)  # force on the base

        # update the plots with associated histories
        self.handle[0].update(self.time_history, [self.zt_history, self.zref_history])
        self.handle[1].update(self.time_history, [self.zv_history, self.zref_history])
        self.handle[2].update(self.time_history, [self.h_history, self.zref_history])
        self.handle[3].update(self.time_history, [self.theta_history])
        self.handle[4].update(self.time_history, [self.Force_left_history])
        self.handle[5].update(self.time_history, [self.Force_right_history])


class myPlot:
    ''' 
        Create each individual subplot.
    '''
    def __init__(self, ax,
                 xlabel='',
                 ylabel='',
                 title='',
                 legend=None):
        ''' 
            ax - This is a handle to the  axes of the figure
            xlable - Label of the x-axis
            ylable - Label of the y-axis
            title - Plot title
            legend - A tuple of strings that identify the data. 
                     EX: ("data1","data2", ... , "dataN")
        '''
        self.legend = legend
        self.ax = ax                  # Axes handle
        self.colors = ['b', 'g', 'r', 'c', 'm', 'y', 'b']
        # A list of colors. The first color in the list corresponds
        # to the first line object, etc.
        # 'b' - blue, 'g' - green, 'r' - red, 'c' - cyan, 'm' - magenta
        # 'y' - yellow, 'k' - black
        self.line_styles = ['-', '-', '--', '-.', ':']
        # A list of line styles.  The first line style in the list
        # corresponds to the first line object.
        # '-' solid, '--' dashed, '-.' dash_dot, ':' dotted

        self.line = []

        # Configure the axes
        self.ax.set_ylabel(ylabel)
        self.ax.set_xlabel(xlabel)
        self.ax.set_title(title)
        self.ax.grid(True)

        # Keeps track of initialization
        self.init = True   

    def update(self, time, data):
        ''' 
            Adds data to the plot.  
            time is a list, 
            data is a list of lists, each list corresponding to a line on the plot
        '''
        if self.init == True:  # Initialize the plot the first time routine is called
            for i in range(len(data)):
                # Instantiate line object and add it to the axes
                self.line.append(Line2D(time,
                                        data[i],
                                        color=self.colors[np.mod(i, len(self.colors) - 1)],
                                        ls=self.line_styles[np.mod(i, len(self.line_styles) - 1)],
                                        label=self.legend if self.legend != None else None))
                self.ax.add_line(self.line[i])
            self.init = False
            # add legend if one is specified
            if self.legend != None:
                plt.legend(handles=self.line)
        else: # Add new data to the plot
            # Updates the x and y data of each line.
            for i in range(len(self.line)):
                self.line[i].set_xdata(time)
                self.line[i].set_ydata(data[i])

        # Adjusts the axis to fit all of the data
        self.ax.relim()
        self.ax.autoscale()
           
