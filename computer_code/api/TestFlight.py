
from scipy import linalg, optimize, signal
from scipy.spatial.transform import Rotation
import copy
import json
import os
import time
import numpy as np
import cv2 as cv
from KalmanFilter import KalmanFilter
import tkinter as tk
import serialHelpers as sH

class Mediator:
    """
    The Mediator interface declares a method used by components to notify the
    mediator about various events. The Mediator may react to these events and
    pass the execution to other components.
    """

    def notify(self, sender, event, data):
        pass


class TestFlightBaseComponent:
    """
    The Base Component provides the basic functionality of storing a mediator's
    instance inside component objects.
    """

    def __init__(self, mediator=None):
        self._mediator = mediator

    @property
    def mediator(self):
        return self._mediator

    @mediator.setter
    def mediator(self, mediator):
        self._mediator = mediator


# class Component1(BaseComponent):
#     """
#     Concrete Components implement various functionality. They don't depend on other
#     components. They also don't depend on any concrete mediator classes.
#     """
#     def do_a(self):
#         print("Component 1 does A.")
#         self.mediator.notify(self, "A")

#     def do_b(self):
#         print("Component 1 does B.")
#         self.mediator.notify(self, "B")


# class Component2(BaseComponent):
#     def do_c(self):
#         print("Component 2 does C.")
#         self.mediator.notify(self, "C")

#     def do_d(self):
#         print("Component 2 does D.")
#         self.mediator.notify(self, "D")


# class ConcreteMediator(Mediator):
#     def __init__(self, component1, component2):
#         self._component1 = component1
#         self._component1.mediator = self
#         self._component2 = component2
#         self._component2.mediator = self

#     def notify(self, sender, event):
#         if event == "A":
#             print("Mediator reacts on A and triggers following operations:")
#             self._component2.do_c()
#         elif event == "D":
#             print("Mediator reacts on D and triggers following operations:")
#             self._component1.do_b()
#             self._component2.do_c()


# if __name__ == "__main__":
#     # The client code.
#     c1 = Component1()
#     c2 = Component2()
#     mediator = ConcreteMediator(c1, c2)

#     print("Client triggers operation A.")
#     c1.do_a()

#     print("\n", end="")

#     print("Client triggers operation D.")
#     c2.do_d()


class TestFlightMediator(Mediator):
    def __init__(self, 
                 camerasComponent, 
                 serialPortComponent, 
                 socketioComponent,
                 calculationLoopComponent,
                 dataProcessorComponent,):
        self._camerasComponent = camerasComponent
        self._camerasComponent.mediator = self
        self._serialPortComponent = serialPortComponent
        self._serialPortComponent.mediator = self
        self._socketioComponent = socketioComponent
        self._socketioComponent.mediator = self
        self._calculationLoopComponent = calculationLoopComponent
        self._calculationLoopComponent.mediator = self
        self._dataProcessorComponent = dataProcessorComponent
        self._dataProcessorComponent.mediator = self

        self.last_run_time = 0
        self._old_armed_status_0 = False

    def start(self):
        self._serialPortComponent.start()
        self._calculationLoopComponent.start()

        # # Start the chart in Tkinter main loop
        # root = tk.Tk()
        # root.wm_title("Live Plotting")
        # self._calculationLoopComponent.start_chart(root)


    def notify(self, sender, event, data=None):
        if event == "A":
            print("Mediator reacts on A and triggers following operations:")
            # self._component2.do_c()
        elif event == "D":
            print("Mediator reacts on D and triggers following operations:")
            # self._component1.do_b()
            # self._component2.do_c()

        elif event == "frame_parsed":
            self._processFrameParsed(sender, data)

        elif event == "serial_status_log":
            self._socketioComponent.c.emit("serial-port-log", data)

        # serial data >>>>
                #     self._calculationLoopComponent.add_serial_data_row(np_data)
        elif event == "serial_data":
            if self._dataProcessorComponent.is_saving_running:
                # print("data type ", type(data))
                string_list = data['data'].split(',')
                self._dataProcessorComponent.append_csv_row(string_list)

                np_data = np.array(string_list).astype(np.float32)

                # type should be 'PID_LOG' there, passed from device
                self._socketioComponent.c.emit("serial-port-data", {"type": data['type'], "data": np_data.tolist() })

                # # np_data[:,ind] = np_data[:,ind].astype(np.float32)
                # with self._calculationLoopComponent.lock:
                #     self._calculationLoopComponent.add_serial_data_row(np_data)

        elif event == "armed_0":
            # print("Mediator reacts on armed_0 ", data)
            if data == True:
                if self._old_armed_status_0 == False:
                    self._old_armed_status_0 = True

                    self._dataProcessorComponent.create_csv_file()
                    self._dataProcessorComponent.is_saving_running = True

                    # self._calculationLoopComponent.start_chart()
            else:
                if self._old_armed_status_0 == True:
                    self._old_armed_status_0 = False
                    self._dataProcessorComponent.is_saving_running = False
                    # self._calculationLoopComponent.stop_chart()
        # serial data <<<<



    def _processFrameParsed(self, sender, data):
        filtered_objects = data
        serialLock = self._camerasComponent.c.serialLock
        serial = self._serialPortComponent.c

        # skip no data?
        if len(filtered_objects) <= 0:
            return

        filtered_object = filtered_objects[0]

        time_now = time.time()
        dt = time_now - self.last_run_time
        self.last_run_time = time.time()
        
        # dt is about 11 ms 0.011 absolute value
        
        # serial_data = { 
        #     "pos": [round(x, 4) for x in filtered_object["pos"]] + [filtered_object["heading"]],
        #     "vel": [round(x, 4) for x in filtered_object["vel"]]
        # }
        # with serialLock:
        #     serial.write(f"{filtered_object['droneIndex']}{json.dumps(serial_data)}".encode('utf-8'))
        #     time.sleep(0.001)

        # _floats = [round(x, 4) for x in filtered_object["pos"]] + [filtered_object["heading"]] + [round(x, 4) for x in filtered_object["vel"]]
        _floats = [float(x) for x in filtered_object["pos"]] + [float(filtered_object["heading"])] + [float(x) for x in filtered_object["vel"]]
        # print('_floats', _floats)

        serial_data = sH.pack_floats_data_for_serial(sH.M_ID_POS_VEL, _floats)
        with serialLock:
            serial.write(serial_data)
            time.sleep(0.001)

        return 0



