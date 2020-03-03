#Importing libraries
import time
import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys, time, math
import tkinter 


#Constants
PORT = 'COM3' #Port of arduino
#Opening the port
def tester():
        arduino = serial.Serial(port = PORT)
        while 1:
                arduinoOutput = arduino.readline()
                graphNumber =int(arduinoOutput.decode())
                print(graphNumber)

def startCode():
        #Opening the port
        arduino = serial.Serial(port = PORT)

        def data_gen():
                t = data_gen.t

                while True:
                        t+=1
                        #Read output from arduino serial port
                        arduinoOutput = arduino.readline()

                        #Convert the number into format readable by python
                        graphNumber =int(arduinoOutput.decode())
                        print(graphNumber)
                        yield t, graphNumber    
        
        def run(data):
                # update the data
                t,y = data
                if t>-1:
                        xdata.append(t)
                        ydata.append(y)
                        if t>xsize: # Scroll to the left.
                                ax.set_xlim(t-xsize, t)
                        line.set_data(xdata, ydata)
                return line,

        def on_close_figure(event):
                sys.exit(0)
       


        while 1 :
                xsize=100    
                data_gen.t = -1
                fig = plt.figure()
                fig.canvas.mpl_connect('close_event', on_close_figure)
                ax = fig.add_subplot(111)
                line, = ax.plot([], [], lw=2,color = 'red')
                ax.set_ylim(-100, 100)
                ax.set_xlim(0, xsize)
                ax.grid()
                xdata, ydata = [], []

                # Important: Although blit=True makes graphing faster, we need blit=False to prevent
                # spurious lines to appear when resizing the stripchart.
                ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=100, repeat=False)
                plt.show()



#Start the code here
from tkinter import*
#creating tkinter window
window = Tk()
window.title("UAS GUI")
window.configure(background="white")
window.geometry("350x400")
window.resizable(0,0)

#Adding gif to the window
mainImage = PhotoImage(file = "UASlogo.gif")
Label(window, image = mainImage, bg = "white").grid(row = 10, column = 0, padx = 15)

#startbutton
startbutton = Button(window,text="Print graph",width = 16, background = 'blue', fg = 'white', activebackground = 'grey', command = startCode)
startbutton.place(x=110,y=100)

#Set button
startbutton = Button(window,text="Set Speed",width = 16, background = 'blue', fg = 'white', activebackground = 'grey', command = startCode)
startbutton.place(x=110,y=150)

#loop the window always
window.mainloop()
