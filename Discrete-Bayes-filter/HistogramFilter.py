import numpy as np
import matplotlib.pyplot as plt
from AckermannVehicle import AckermannVehicle



def plot_historgram(bel):
    plt.cla()
    plt.bar(range(0,bel.shape[0]),bel,width=1.0)
    plt.axis([0,bel.shape[0]-1,0,1])
    plt.draw()
    plt.pause(0.5)


# initial beleifes
bel_x = np.hstack((1,np.zeros(999)))
bel_y = np.hstack((1,np.zeros(999)))

# vehicle probabilty model
alpha = [0.05, 0.1, 0.07, 0.06]
car = AckermannVehicle(alpha, sampleSize_=100)

x = 0
y = 0
th = 0
# simultion loop
for i in range(50):
    x_, y_, th_ = car.SampleMove(1, np.deg2rad(0), t=0.1)

    # increment to prevoius value
    x += x_
    y += y_
    th += th_

    # discrete the value
    x1 = x * 100
    x1 = x1.astype(int)
    y1 = y * 100
    y1 = y1.astype(int)

    # add one to locations
    np.add.at(bel_x, x1, 1)
    np.add.at(bel_y, y1, 1)

    # normlize the belefies
    bel_x = bel_x / bel_x.sum()
    bel_y = bel_y / bel_y.sum()

    # 
    bel_x *= bel_x 
    bel_y *= bel_y 

    # normlize to be sum to one 
    bel_x = bel_x / bel_x.sum()
    bel_y = bel_y / bel_y.sum()

    # plot
    plot_historgram(bel_x)


print(np.argmax(bel_x))
print(np.argmax(bel_y))

print(bel_x)
print(bel_y)

plt.show()