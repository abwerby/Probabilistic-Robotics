import numpy as np 
import matplotlib.pyplot as plt

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
        

        return X, Y, TH

    def __repr__(self):
        return " X = " + str(self.x) + ", Y = " + str(self.y) + ", Theta =  " + str(self.th)


alpha = [0.05, 0.1, 0.07, 0.06]
car = AckermannVehicle(alpha, sampleSize_=200)

# print init loc
print(car)

# list to track the vehicle pose
xl = []
yl = []

# simulation loop
for i in range(100):
    car.move(1, np.deg2rad(30))
    xl.append(car.x)
    yl.append(car.y)

# print final loc
print(car)

# plot vehicle trajectory
plt.plot(xl, yl)
plt.show()

###########################################################################
#                     plot the sampling output                            #
#                                                                         #
###########################################################################

# reset car pose
car.x = 0
car.y = 0
car.th = 0

# plot start pose
plt.scatter(car.x, car.y)

X, Y, TH = car.SampleMove(1, np.deg2rad(20), t=2)

# plot final pose sample
plt.scatter(X, Y, alpha=0.3)

# plot line from start to the mean of the end pose
plt.plot([car.x, X.mean()], [car.y, Y.mean()])
plt.show()








