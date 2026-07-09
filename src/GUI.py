import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
import time

# Create main window
root = tk.Tk()
root.title('Real-Time Data Visualization in Tkinter')
root.geometry('1200x700')

# Create a Matplotlib figure
fig = Figure(figsize=(8, 5), dpi=100)
ax = fig.add_subplot(111)
ax.set_title('Real-Time Data Plot')
ax.set_xlabel('Time')
ax.set_ylabel('Value')

# Embed the Matplotlib figure in Tkinter
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

root.mainloop()