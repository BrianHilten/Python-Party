# This is a collection of classes used in SRL's Mission Control Application
# Window: 
# Functionality- Contains tkinter TopLevel 
# Plotter:
# Functionality- Contains everything needed to plot data
import tkinter as tk

# Top level class: Window
# Child class: Plotter

class Window:
    def __init__(self, master=None):
        self.title("Real-Time Data Visualization Dashboard")
        self.geometry("850x550")