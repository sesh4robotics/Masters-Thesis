from Test_Rig_UI import Ui_MainWindow
from filters import Discrete_Low_Pass_Filter

import math
import sys
import datetime
import time
import concurrent.futures

from PyQt5 import QtCore, QtGui, QtWidgets

from serial_communication import serial_comms
import serial.tools.list_ports


def servo_saturation(Value):           #Saturtion is disabled
    if(Value > 113):
        Output = 113
    elif(Value < 0):
        Output = 0
    else:
        Output = Value
    
    Output = Value
    
    return Output

def bend_rot_to_ABC():

    global Servo_1_Output
    global Servo_2_Output
    global Servo_3_Output

    Servo_1_Output = servo_saturation(-13*Bending*math.cos(math.radians(Rotation))/20 + Servo_1_Zero)
    Servo_2_Output = servo_saturation(-13*Bending*math.cos(math.radians(240 - Rotation))/20 + Servo_2_Zero)
    Servo_3_Output = servo_saturation(-13*Bending*math.cos(math.radians(120 - Rotation))/20 + Servo_3_Zero)

def potentiometer(a, b):
    pass

def current_sensor(a, b):
    pass

def select_new_serial_port():

    if(len(ui.Serial_Ports.currentText()) != 0):
        text = ui.Serial_Ports.currentText().split()[0]
        u1.Change_Port(text)

def GUI_parts():

    if(u1.Serial_Connection_Status() == 1):

        ui.Serial_Status_Label.setText("Connected")
        ui.Serial_Status_Label.setStyleSheet("background-color: rgb(0,255,0)") #Green
    else:
        ui.Serial_Status_Label.setText("Not Connected")
        ui.Serial_Status_Label.setStyleSheet("background-color: rgb(255,0,0)") #Red

    global port_list

    if(port_list != serial.tools.list_ports.comports()):
        ui.Serial_Ports.clear()
        port_list = serial.tools.list_ports.comports()
        for port, desc, hwid in sorted(port_list):
            ui.Serial_Ports.addItem("{} {}".format(port, desc))

    QtWidgets.QApplication.processEvents()


if __name__ == "__main__":
    Bending = 0
    Rotation = 0

    Servo_1_Zero = 66.5
    Servo_2_Zero = 66.5
    Servo_3_Zero = 66.5

    Main_Loop_Time = 20  #Milliseconds

    app = QtWidgets.QApplication(sys.argv)

    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)

    ui.bending.setValue(Bending)
    ui.rotation.setValue(Rotation)
    ui.servo_a_zero.setValue(Servo_1_Zero)
    ui.servo_b_zero.setValue(Servo_2_Zero)
    ui.servo_c_zero.setValue(Servo_3_Zero)
    
    u1 = serial_comms('COM15', 115200)

    port_list = serial.tools.list_ports.comports()
    for port, desc, hwid in sorted(port_list):
        ui.Serial_Ports.addItem("{} {}".format(port, desc))

    ui.Serial_Connect.clicked.connect(select_new_serial_port)

    MainWindow.show()

    currentfilter1 = Discrete_Low_Pass_Filter(0.1, Main_Loop_Time)
    currentfilter2 = Discrete_Low_Pass_Filter(0.1, Main_Loop_Time)
    currentfilter3 = Discrete_Low_Pass_Filter(0.1, Main_Loop_Time)

    b = datetime.datetime.now()
    c = datetime.datetime.now()
    d = datetime.datetime.now()


    while True:
        a = datetime.datetime.now()

        if((a - b).microseconds/1000 >= Main_Loop_Time):
            
            filteredcurrent1 = currentfilter1.Order_1_LP_Filter(u1.current[0])
            filteredcurrent2 = currentfilter2.Order_1_LP_Filter(u1.current[1])
            filteredcurrent3 = currentfilter3.Order_1_LP_Filter(u1.current[2])

            bend_rot_to_ABC()

            u1.Send_Servo_Reference_Values(1, Servo_1_Output)
            u1.Send_Servo_Reference_Values(2, Servo_2_Output)
            u1.Send_Servo_Reference_Values(3, Servo_3_Output)            
            
            if((a - b).microseconds/1000 - Main_Loop_Time > Main_Loop_Time):
                print((a - b).microseconds/1000 - Main_Loop_Time)

            b = datetime.datetime.now()

        if((a - c).microseconds/1000 >= 100):

            Servo_1_Zero = ui.servo_a_zero.value()
            Servo_2_Zero = ui.servo_b_zero.value()
            Servo_3_Zero = ui.servo_c_zero.value()

            Bending = ui.bending.value()
            Rotation = ui.rotation.value()

            ui.servo_a_potentiometer.display(u1.potentiometer[0])
            ui.servo_b_potentiometer.display(u1.potentiometer[1])
            ui.servo_c_potentiometer.display(u1.potentiometer[2])

            ui.servo_a_current.display(filteredcurrent1)
            ui.servo_b_current.display(filteredcurrent2)
            ui.servo_c_current.display(filteredcurrent3)

            with concurrent.futures.ProcessPoolExecutor() as executor:
                executor.map(GUI_parts())
                    
            c = datetime.datetime.now()

        u1.Check_Serial_Parallel()

    
    sys.exit(app.exec_())

#--------- THE END ----------









