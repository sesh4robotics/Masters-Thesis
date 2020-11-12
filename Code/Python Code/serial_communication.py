import serial
import time
import concurrent.futures

class serial_comms:
 
    def __init__(self, COM_Port, Baud_Rate):

        self.Port = COM_Port
        self.Baud = Baud_Rate
        self.parts = []
        self.potentiometer = [0.00]*3
        self.current = [0.00]*3
        self.serialopen = 0

        try:
            self.ser = serial.Serial(self.Port, self.Baud, timeout=1)
            self.ser.open()
        except:
            pass

    def Change_Port(self, COM_Port):

        self.Port = COM_Port
        time.sleep(0.01)
        self.ser = serial.Serial(self.Port, self.Baud, timeout=0)
        self.Serial_Reconnect()

    def Serial_Reconnect(self):

        try:
            self.ser.close()
            time.sleep(0.01)
            self.ser.open()
        except:
            pass

    def Serial_Close(self):

        self.ser.close()

    def Serial_Connection_Status(self):

        try:
            if(self.ser.inWaiting()):
                self.serialopen = 1
        except:
                self.serialopen = 0
                self.Serial_Reconnect()
        
        return self.serialopen

    def Check_Serial_Parallel(self):
        
        with concurrent.futures.ProcessPoolExecutor() as exe:
            exe.map(self.Check_Serial())

    def Check_Serial(self):

        try:
            self.line = self.ser.readline().strip()   # read a '\n' terminated line
            self.parts = self.line.decode().strip('\x00').split(" ")
            
            #print(self.parts)

            if(len(self.parts) > 0) and self.line:

                if(self.parts[0] == 'p'):       # 'p' is for potentiometer values

                    if(self.parts[2] == None):
                        pass
                    else:
                        self.potentiometer[int(self.parts[1]) - 1] = float(self.parts[2])
                        pass


                if(self.parts[0] == 'c'):       # 'i' is for current value
                    if(self.parts[2] == None):
                        pass
                    else:
                        self.current[int(self.parts[1]) - 1] = float(self.parts[2])
                        
                        if(self.parts[2] > 0):
                            print("1")

                        #print(self.current)
                        pass

        except:
            pass

    def Send_Servo_Reference_Values(self, servo, reference):

        try:

            st = 'r' + " " + str(servo) + " " + str(reference) + '\n'
            with concurrent.futures.ProcessPoolExecutor() as executor1:
                executor1.map(self.ser.write(st.encode()))       # 'r' is for reference value identifier

        except:
            pass