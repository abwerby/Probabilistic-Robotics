import numpy as np 
import matplotlib.pyplot as plt

class DiffVehicle:
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
    
    def move(self, vr, vl):
        '''
            excucute one sample time of the control input
            v: linera velocity in m/s.
            ph: steer angle in rad.
        '''
        # add the error value
        v = ((vr+vl)/2)
        w = ((vr-vl)/self.l)

        v = v + np.random.normal(0, self.alpha[0]*v**2 + self.alpha[1]*w**2, self.sampleSize).mean()  
        w = w + np.random.normal(0, self.alpha[2]*v**2 + self.alpha[3]*w**2, self.sampleSize).mean()

        # vehicle kinmatics
        self.th = self.th + w                    * self.dt
        self.x  = self.x  + v * np.cos(self.th)  * self.dt
        self.y  = self.y  + v * np.sin(self.th)  * self.dt
        

    def SampleMove(self, vr, vl, t = 1):
        '''
        retrun array of sample of x, y, th pose of the vehicle 
        v: linera velocity in m/s.
        ph: steer angle in rad.
        mean: the mean of normal distrubution of the error in control input.
        std:  the std of normal distrubution of the error in control input.
        t: the time of the command take place.
        '''
        v = ((vr+vl)/2)
        w = ((vr-vl)/self.l)

        v  = v  + np.random.normal(0, self.alpha[0]*v**2 + self.alpha[1]*w**2, self.sampleSize)
        w = w + np.random.normal(0, self.alpha[2]*v**2 + self.alpha[3]*w**2, self.sampleSize)

        TH = self.th + w * t
        X  = self.x  + v * np.cos(TH)  * t
        Y  = self.y  + v * np.sin(TH)  * t
        

        return X, Y, TH

    def __repr__(self):
        return " X = " + str(self.x) + ", Y = " + str(self.y) + ", Theta =  " + str(self.th)


alpha = [0.05, 0.1, 0.07, 0.06]
robot = DiffVehicle(alpha, sampleSize_=200)

# print init loc
print(robot)

# list to track the vehicle pose
xl = []
yl = []

# simulation loop
for i in range(100):
    robot.move(1, 1.5)
    xl.append(robot.x)
    yl.append(robot.y)

# print final loc
print(robot)

# plot vehicle trajectory
plt.plot(xl, yl)
plt.show()

###########################################################################
#                     plot the sampling output                            #
#                                                                         #
###########################################################################

# reset robot pose
robot.x = 0
robot.y = 0
robot.th = 0

# plot start pose
plt.scatter(robot.x, robot.y)

X, Y, TH = robot.SampleMove(1, 1.5, t=2)

# plot final pose sample
plt.scatter(X, Y, alpha=0.3)

# plot line from start to the mean of the end pose
plt.plot([robot.x, X.mean()], [robot.y, Y.mean()])
plt.show()








