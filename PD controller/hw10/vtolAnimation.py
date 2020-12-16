import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np 
import vtolParam as P


class vtolAnimation:
    '''
        Create satellite animation
    '''
    def __init__(self):
        # Used to indicate initialization
        self.flag_init = True
        
        # Initializes a figure and axes object
        self.fig, self.ax = plt.subplots()

        # Initializes a list object that will be used to contain
        # handles to the patches and line objects.
        self.handle = []
        
        plt.axis([0*P.length, 2.0*P.length, 0*P.length,
                  2.0*P.length])
        plt.plot([-10.0*P.length, 10.0*P.length], [0, 0], 'b--')
        self.length = P.length
        self.width = P.width
        self.radius = P.radius
        self.w = P.w
        self.h = P.h
        
            
    def update(self, state):
         # Process inputs to function
        zt = state.item(0)   # target
        zv = state.item(1)   #Horizontal position 
        h = state.item(2)   #verticle position 
        theta = state.item(3)     # angle of arm, rad

        self.drawtarget(zt,h)
        self.drawbody(zv,h,theta)
        self.drawwing1(zv,h,theta)
        self.drawwing2(zv,h,theta)
        self.drawarm1(zv,h,theta)
        self.drawarm2(zv,h,theta)
        self.ax.axis('equal')
        
        # Set initialization flag to False after first call
        if self.flag_init == True:
            self.flag_init = False

#––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
       
    def drawtarget(self, zt,h):
        # specify bottom left corner of rectangle
        x = zt-P.w/2.0
        y = 0
        corner = (x, y)
        # create rectangle on first call, update on subsequent calls
        if self.flag_init == True:
            # Create the Rectangle patch and append its handle
            # to the handle list
            self.handle.append(
                mpatches.Rectangle(corner, P.w, P.h,
                                   fc = 'red', ec = 'black'))
            # Add the patch to the axes
            self.ax.add_patch(self.handle[0])
        else:
            self.handle[0].set_xy(corner)  # Update patch   

    def drawbody(self, zv,h,theta):
        # specify bottom left corner of rectangle
        x = zv-P.w/2.0
        y = h-P.h/2.0
        corner = (x, y)
        # create rectangle on first call, update on subsequent calls
        if self.flag_init == True:
            # Create the Rectangle patch and append its handle
            # to the handle list
            self.handle.append(
                mpatches.Rectangle(corner, P.w, P.h,
                                   fc = 'blue', ec = 'black'))
            # Add the patch to the axes
            self.ax.add_patch(self.handle[1])
        else:
            self.handle[1].set_xy(corner)  # Update patch
            # self.handle.append(
            #     mpatches.Rectangle(corner, P.w, P.h,theta*180/np.pi,
            #                        fc = 'blue', ec = 'black'))
            
            
            
    def drawwing1(self, zv,h,theta):
        # specify center of circle
        x = zv+((P.radius+P.d)*(np.cos(theta)))
        y = h+((P.radius+P.d)*(np.sin(theta)))
        center = (x,y)
        # create circle on first call, update on subsequent calls
        if self.flag_init == True:
            # Create the CirclePolygon patch and append its handle
            # to the handle list
            self.handle.append(
                mpatches.CirclePolygon(center, radius=P.radius,
                    resolution=15, fc='limegreen', ec='black'))
            # Add the patch to the axes
            self.ax.add_patch(self.handle[2])
        else:
            self.handle[2]._xy = center

    def drawwing2(self, zv,h,theta):
        # specify center of circle
        x = zv-(P.radius+P.d)*np.cos(theta)
        y = h-(P.radius+P.d)*np.sin(theta)
        center = (x,y)
        # create circle on first call, update on subsequent calls
        if self.flag_init == True:
            # Create the CirclePolygon patch and append its handle
            # to the handle list
            self.handle.append(
                mpatches.CirclePolygon(center, radius=P.radius,
                    resolution=15, fc='limegreen', ec='black'))
            # Add the patch to the axes
            self.ax.add_patch(self.handle[3])
        else:
            self.handle[3]._xy = center

    def drawarm1(self, zv,h,theta):
        # specify x-y points of the rod
        X = [zv, zv+P.d*np.cos(theta)]
        Y = [h, h+P.d*np.sin(theta)]
        # create rod on first call, update on subsequent calls
        if self.flag_init == True:
            # Create the line object and append its handle
            # to the handle list.
            line, =self.ax.plot(X, Y, lw=1, c='black')
            self.handle.append(line)
        else:
            self.handle[4].set_xdata(X)
            self.handle[4].set_ydata(Y)
            
    def drawarm2(self, zv,h,theta):
        # specify x-y points of the rod
        X = [zv, (zv-P.d*np.cos(theta))]
        Y = [h, (h-P.d*np.sin(theta))]
        # create rod on first call, update on subsequent calls
        if self.flag_init == True:
            # Create the line object and append its handle
            # to the handle list.
            line, =self.ax.plot(X, Y, lw=1, c='black')
            self.handle.append(line)
        else:
            self.handle[5].set_xdata(X)
            self.handle[5].set_ydata(Y)