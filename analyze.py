import numpy
import matplotlib.pyplot

backLegSensorValues = numpy.load('data/backLegSensorValues.npy')
frontLegSensorValues = numpy.load('data/frontLegSensorValues.npy')
blTargetAngles = numpy.load('data/blTargetAngles.npy')
flTargetAngles = numpy.load('data/flTargetAngles.npy')

# matplotlib.pyplot.title('Front and Back Leg Sensors')
# matplotlib.pyplot.plot(backLegSensorValues, label='backLegSensorValues', linewidth=3)
# matplotlib.pyplot.plot(frontLegSensorValues, label='frontLegSensorValues', linewidth=1)
# matplotlib.pyplot.legend()
# matplotlib.pyplot.show()

matplotlib.pyplot.plot(blTargetAngles, label='Back leg motor values', linewidth=4)
matplotlib.pyplot.plot(flTargetAngles, label='Front leg motor values', linewidth=1)

matplotlib.pyplot.legend()
matplotlib.pyplot.legend()
matplotlib.pyplot.show()

