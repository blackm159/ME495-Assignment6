import numpy as np

simLength = 1000

backLegSensorValues = np.zeros(1000)
frontLegSensorValues = np.zeros(1000)

amplitude = np.pi/4.0
frequency = 10
phaseOffset = np.pi/8.0

backLegAmplitude = np.pi/4.0
backLegFrequency = 10
backLegPhaseOffset = np.pi/8.0

frontLegAmplitude = np.pi/4.0
frontLegFrequency = 10
frontLegPhaseOffset = 0

frontLegMotorValues = frontLegAmplitude * np.sin(frontLegFrequency * 
    np.linspace(0, 2*np.pi, 1000) + frontLegPhaseOffset)
backLegMotorValues = backLegAmplitude * np.sin(backLegFrequency * 
    np.linspace(0, 2*np.pi, 1000) + backLegPhaseOffset)

numberOfGenerations = 1
populationSize = 1

numSensorNeurons = 7
numMotorNeurons = 6

motorJointRange = 0.3 #this does something with angle values