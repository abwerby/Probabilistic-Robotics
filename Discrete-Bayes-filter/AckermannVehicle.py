import numpy as np 

class AckermannVehicle:
    def __init__(self, alpha, BaseLen=1, x_=0, y_=0, th_=0, samplingTime=0.01, sampleSize_=30):
        '''
            Baselen: the base length of the vehicle.
            alpha: list of the probablistic model paramters.
            x_, y_, th_ = init pose of the vehicle
            samplingTime: sampling time of the system.
            sampleSize: the sample size of the error normal distrubutions.
        '''
        self.x  = x_
        self.y  = y_
        self.th = th_
        self.dt = samplingTime
        self.l  = BaseLen  
        self.alpha = alpha 
        self.sampleSize = sampleSize_
    
    def move(self, v, ph):
        '''
            excucute one sample time of the control input
            v: linera velocity in m/s.
            ph: steer angle in rad.
        '''
        # add the error value
        v  = v  + np.random.normal(0, self.alpha[0]*v**2 + self.alpha[1]*ph**2, self.sampleSize).mean()  
        ph = ph + np.random.normal(0, self.alpha[2]*v**2 + self.alpha[3]*ph**2, self.sampleSize).mean()

        # vehicle kinmatics
        self.th = self.th + (v / self.l) * ph    * self.dt
        self.x  = self.x  + v * np.cos(self.th)  * self.dt
        self.y  = self.y  + v * np.sin(self.th)  * self.dt
        

    def SampleMove(self, v, ph, t = 1):
        '''
        retrun array of sample of x, y, th pose of the vehicle 
        v: linera velocity in m/s.
        ph: steer angle in rad.
        mean: the mean of normal distrubution of the error in control input.
        std:  the std of normal distrubution of the error in control input.
        t: the time of the command take place.
        '''
        v  = v  + np.random.normal(0, self.alpha[0]*v**2 + self.alpha[1]*ph**2, self.sampleSize)
        ph = ph + np.random.normal(0, self.alpha[2]*v**2 + self.alpha[3]*ph**2, self.sampleSize)

        TH = self.th + (v / self.l) * ph * t
        X  = self.x  + v * np.cos(TH)  * t
        Y  = self.y  + v * np.sin(TH)  * t
        

        return np.array(X), np.array(Y), np.array(TH)

    def __repr__(self):
        return " X = " + str(self.x) + ", Y = " + str(self.y) + ", Theta =  " + str(self.th)









