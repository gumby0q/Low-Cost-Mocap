from TestFlight import TestFlightBaseComponent, TestFlightMediator
import csv
from datetime import datetime
import numpy as np
import json
import threading
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk

serial_data_header = ["xPWM", "yPWM", "zPWM", "yawPWM",
                        "xPos", "yPos", "zPos", "yawPos",
                        "xVelSetpoint", "yVelSetpoint", "zVelSetpoint",
                        "xVelOutput", "yVelOutput", "zVelOutput", "yawPosOutput",
                        "groundEffectMultiplier", "millis","armed", "max_t_diff_us"]


class DataProcessor(TestFlightBaseComponent):
    def __init__(self, mediator=None,):
        super().__init__(mediator)

        self.file_path = "api/csv_data/"
        self.csv_file_name_postfix = "_seria_data.csv"
        self.csv_file_name = ""
        self.is_saving_running = False

    def create_csv_file(self):
        # Initialize the CSV file with headers
        timestamp = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
        csv_file_name = self.file_path + timestamp + self.csv_file_name_postfix
        self.csv_file_name = csv_file_name
        with open(csv_file_name, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(serial_data_header)

    def append_csv_row(self, data):
        csv_file_name = self.csv_file_name
        with open(csv_file_name, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(data)
        # print("Component 2 does C.")
        # self.mediator.notify(self, "C")


class BaseComponentWrapper(TestFlightBaseComponent):
    def __init__(self, component, mediator=None,):
        super().__init__(mediator)
        self.c = component
        self.c.parentComponent = self


class CamerasC(BaseComponentWrapper):
    def dosmth():
        pass
        # print("Component 2 does C.")
        # self.mediator.notify(self, "C


class SerialPortC(BaseComponentWrapper):
    def __init__(self, component, serialLock, mediator=None,):
        super().__init__(component, mediator)
        self.serial_thread = None
        self.serialLock = serialLock

    def start(self):
        # serial -----------------------------------------------------
        def read_serial_data():
            while True:
                if self.c.in_waiting > 0:
                    with self.serialLock:
                        try:
                            data = self.c.readline().decode('utf-8').rstrip()

                            try:
                                json_data = json.loads(data)
                                # print(f"json_data: {json_data['data']}")
                                self.mediator.notify(self, "serial_data", json_data)

                            except Exception as e:
                                print(f"Received: {data} {e}")
                            
                        except Exception as e:
                            print(f"Error reading data: {e}")

        # Create and start a thread to read serial data
        serial_thread = threading.Thread(target=read_serial_data)
        serial_thread.daemon = True
        serial_thread.start()
        
        self.serial_thread = serial_thread
        # serial -----------------------------------------------------


class SocketIOC(BaseComponentWrapper):
    def dosmth():
        pass
        # print("Component 2 does C.")
        # self.mediator.notify(self, "C


class CalculationLoopC(TestFlightBaseComponent):
    def __init__(self, mediator=None,):
        super().__init__(mediator)
        self.lock = None
        self.calculation_thread = None
        self.delta_t = 0.2

        self.lock = threading.Lock()

        # chart things

        # Shared variables
        # Fixed length for the chart data array
        self.max_length = 300
        # Assuming data_np_array has 19 columns
        self.chart_data = np.zeros((self.max_length, len(serial_data_header)))  
        self.current_index = 0
        self.is_full = False


        # # Set up the plot
        # fig, ax = plt.subplots()

        # # Plotting function for multiple lines
        # def update_plot(frame, calc_loop):
        #     data = calc_loop.get_chart_data()
        #     ax.clear()
        #     # for i in range(data.shape[1]):
        #     #     ax.plot(data[:, i], label=f'Line {i+1}')
        #     x_data = data[:, 16]
        #     y_data = data[:, 18]
        #     line, = ax.plot(x_data, y_data, label=f'Line {1}')

        #     plt.legend()
        #     # plt.xticks(rotation=45, ha='right')
        #     # plt.subplots_adjust(bottom=0.30)
        #     plt.title('Live Serial Data')
        #     plt.xlabel('Time Index')
        #     plt.ylabel('Data')

        # self.update_plot_func = update_plot
        # self.fig = fig
        # self.ax = ax
        # # self.ani = None
        # self.ani = animation.FuncAnimation(self.fig, self.update_plot_func, fargs=(self,), interval=250)
        # # self.ani.event_source.stop()
        # # plt.show()

    def start_chart(self, root):
        # Show the plot
        # self.ani.event_source.start()
        # pass

        # # Embed the plot in a Tkinter window
        # canvas = FigureCanvasTkAgg(self.fig, master=root)
        # canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        # canvas.draw()

        # def _quit():
        #     root.quit()
        #     root.destroy()
        
        # quit_button = tk.Button(master=root, text="Quit", command=_quit)
        # quit_button.pack(side=tk.BOTTOM)
        
        # root.protocol("WM_DELETE_WINDOW", _quit)
        # root.mainloop()
        pass

    def stop_chart(self):
        # self.ani.event_source.stop()
        # plt.close()
        pass

    def add_serial_data_row(self, data_np_array):
        # data_np_array
        with self.lock:
            self.chart_data[self.current_index] = data_np_array
            self.current_index = (self.current_index + 1) % self.max_length
            if self.current_index == 0:
                self.is_full = True

    def get_chart_data(self):
        with self.lock:
            if self.is_full:
                return np.vstack((self.chart_data[self.current_index:], self.chart_data[:self.current_index]))
            else:
                return self.chart_data[:self.current_index]
            
    def clear_chart_data(self):
        with self.lock:
            self.current_index = 0
            self.chart_data = np.zeros((self.max_length, len(self.chart_data)))
            self.is_full = False


    def start(self):
        delta_t = self.delta_t

        # Calculation function to be executed by the calculation thread
        def calculation_loop(delta_t):
            # global calculated_variable1, calculated_variable2
            # global tick_cntr
            while True:
                time.sleep(delta_t)
                # Perform calculations
                # new_value1 = time.time()  # Example calculation
                # new_value2 = new_value1 * 2  # Another example calculation

                # tick_cntr = tick_cntr + 1
                # print("loop tick", tick_cntr, new_value1, delta_t)
                # Safely update shared variables
                # with self.lock:
                    # calculated_variable1 = new_value1
                    # calculated_variable2 = new_value2


        calculation_thread = threading.Thread(target=calculation_loop, args=(delta_t,))
        calculation_thread.daemon = True  # Daemonize thread to exit when main program exits
        calculation_thread.start()

