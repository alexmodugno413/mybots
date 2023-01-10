import numpy
import matplotlib.pyplot

backLegSensorValues = numpy.load('data/backLegSensorValues.npy')
frontLegSensorValues = numpy.load('data/frontLegSensorValues.npy')

matplotlib.pyplot.title('Front and Back Leg Sensors')
matplotlib.pyplot.plot(backLegSensorValues, label='backLegSensorValues', linewidth=3)
matplotlib.pyplot.plot(frontLegSensorValues, label='frontLegSensorValues', linewidth=1)
matplotlib.pyplot.legend()
matplotlib.pyplot.show()

