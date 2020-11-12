# Discrete Time Low Pass Filter Module (Will be expanded later!)

# Made by: Seshagopalan T M 

# Call the function 'Order_1_LP_Filter' with your object in your code to dynamically provide filtered values
# This implementation is the same as the Simulink block titled 'Low-Pass Filter (Discrete or Continuous)' with the discrete implemntation
# This filter implements forward Euler

class Discrete_Low_Pass_Filter:
    
    def __init__(self, Filter_Time_Constant, Sampling_Time, Upper_Saturation = float('inf') , Lower_Saturation = float('-inf') , Filter_Gain = 1, Sample = 0.0, Previous_Output = 0.0, Output = 0.0):
        self.filter_constant = Filter_Time_Constant
        self.sampling_time = Sampling_Time
        self.upper_saturation = Upper_Saturation
        self.lower_saturation = Lower_Saturation
        self.filter_gain = Filter_Gain
        self.sample = Sample
        self.previous_output = Previous_Output
        self.output = Output

    def Reset_Filter(self, Sample = 0.0, Previous_Output = 0.0, Output = 0.0):
        self.previous_output = Previous_Output
        self.sample = Sample
        self.output = Output

    def Update_Filter_Constant(self, Filter_Time_Constant):
        self.filter_constant = Filter_Time_Constant

    def Update_Sampling_Time(self, Sampling_Time):
        self.sampling_time = Sampling_Time
    
    def Update_Filter_Gain(self, Filter_Gain):
        self.filter_gain = Filter_Gain

    def Update_Saturation(self, Upper_Saturation = float('inf') , Lower_Saturation = float('-inf')):
        self.upper_saturation = Upper_Saturation
        self.lower_saturation = Lower_Saturation

    def Order_1_LP_Filter(self, Sample):
        self.sample = Sample

        self.discrete_time_fraction = self.sampling_time/self.filter_constant

        if(self.discrete_time_fraction > 1):
            self.discrete_time_fraction = 1
        
        self.previous_output = self.output
        self.output = (1-(self.discrete_time_fraction))*self.previous_output + (self.sample)*((self.discrete_time_fraction))
        return self.previous_output

    #def Order_2_Filter(self, Sample):
    #    self.sample = Sample
    #    self.previous_sample = self.sample
    #    return self.output
