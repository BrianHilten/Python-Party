# Main GUI for SRL's Mission Control Application
# Starting code originally sourced from: https://medium.com/@kshipreet24comp/real-time-data-visualization-in-tkinter-3242991a25f8
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import time
import zmq
import ast



ZMQ_SUB_PORT = 5002 # Port to subscribe to via ZMQ

zmq_context = zmq.Context()

# Try connecting to ZMQ Port
try:
    subscriber = zmq_context.socket(zmq.SUB)
    subscriber.connect(f"tcp://127.0.0.1:{ZMQ_SUB_PORT}")
    subscriber.setsockopt(zmq.SUBSCRIBE, b'')
    print(f'Subscribed to {ZMQ_SUB_PORT}!')
except:
    print("Couldn't Connect to ZMQ Socket")
    exit

poller = zmq.Poller()
poller.register(subscriber, zmq.POLLIN)
running = False
x_data, y_data = [], []
data_source = "None" # For selecting the main data source
data_sub_source = -1 # For selecting a specific piece of data to plot from the data source packet

log_channels = {
    "ekf": 0,
    "sensor": 0,
    "state": 0,
}

def start_plot():
    global running
    running = True
    status_label.config(text="Status: Running ✅", fg="green")
    update_plot()

def stop_plot():
    global running
    running = False
    status_label.config(text="Status: Stopped ⛔", fg="red")

def select_source(source):
    global data_source
    data_source = source
    source_label.config(text=data_source, fg="black")


def select_subsource(subsource):
    global data_sub_source
    global data_source
    global running
    for ch in log_channels:
        log_channels[ch] = 0 # Reset channels so we only have one running in a window at a time
    
    match subsource:
        case "Position (x)":
            data_source = "ekf"
            ax.set_ylabel(subsource)
            log_channels["ekf"] = 1
            data_sub_source = 2 # Expected index within EKF packet published by ZMQ
        case "Altitude":
            data_source = "sensor"
            ax.set_ylabel(subsource)
            log_channels["sensor"] = 1
            data_sub_source = 14

    source_label.config(text=f"{data_source} -> {subsource}", fg="black")
    if not running:
        start_plot()

def update_plot():
    if not running:
        return
    sockets = dict(poller.poll(timeout=0))
    if subscriber in sockets:
        topic, bytes = subscriber.recv_multipart()
        topic_str = topic.decode('utf-8')
        payload = ast.literal_eval(bytes.decode('utf-8'))
        value = payload[data_sub_source]
        # print(f"GUI Received: {topic_str}: {payload}")
        if log_channels.get(topic_str, 0) == 1 and data_sub_source != -1:
            x_data.append(time.time() % 50)
            y_data.append(value)
            if len(x_data) > 50:
                x_data.pop(0)
                y_data.pop(0)

            # Automatically scale chart
            line.set_data(range(len(y_data)), y_data)
            ax.set_xlim(0, max(len(y_data), 1))
            if y_data:
                y_min, y_max = min(y_data), max(y_data)
                pad = max((y_max - y_min) * 0.1, 1)
                ax.set_ylim(y_min - pad, y_max + pad)
            canvas.draw()
    root.after(100, update_plot)

# Main Window
root = tk.Tk()
root.title("Real-Time Data Visualization Dashboard")
root.geometry("850x550")

# 3. Create Plot
fig = Figure(figsize=(8, 4.5), dpi=100)

ax = fig.add_subplot(111)
line, = ax.plot([], [], 'b-', linewidth=2, label="Live Feed")
ax.set_ylim(0, 100)
ax.set_title("Real-Time Data Plot")
ax.set_xlabel("Time")
ax.set_ylabel("Value")
ax.legend()


canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

# Frame
frame = tk.Frame(root)
frame.pack(pady=10)

start_btn = tk.Button(frame, text="Start", command=lambda: start_plot(), width=10, bg="lightgreen")
stop_btn = tk.Button(frame, text="Stop", command=lambda: stop_plot(), width=10, bg="tomato")
source_btn = tk.Menubutton(frame, text="Data Source", width=10, bg="gray")
source_dropdown_menu = tk.Menu(source_btn, tearoff=0)

# Main source choices
source_dropdown_menu.add_command(label="None", command=lambda: select_source("None"))
source_dropdown_menu.add_command(label="Sensors", command=lambda: select_source("Sensors"))

# Nested submenu for EKF fields
ekf_submenu = tk.Menu(source_dropdown_menu, tearoff=0)
source_dropdown_menu.add_cascade(label="EKF", menu=ekf_submenu)
ekf_submenu.add_command(label="Position (x)", command=lambda: select_subsource("Position (x)"))

# Nested submenu for Sensor fields
sensor_submenu = tk.Menu(source_dropdown_menu, tearoff=0)
source_dropdown_menu.add_cascade(label="Sensors", menu=sensor_submenu)
sensor_submenu.add_command(label="Altitude", command=lambda: select_subsource("Altitude"))

status_label = tk.Label(frame, text="Status: Stopped ⛔", fg="red")
source_label = tk.Label(frame, text=data_source, fg="red")

start_btn.grid(row=0, column=0, padx=10)
stop_btn.grid(row=0, column=1, padx=10)
source_btn.grid(row=0, column=2, padx=10)
status_label.grid(row=0, column=4, padx=10)
source_label.grid(row=0, column=5, padx=10)

source_btn["menu"] = source_dropdown_menu

root.mainloop()