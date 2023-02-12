import numpy as np
import pyrosim.pyrosim as pyrosim
import os
import random
import time
import constants as c

class SOLUTION:
    def __init__(self, nextAvailableID):
        self.myID = nextAvailableID
        # a = 3
        # b = 2
        self.weights = np.empty(shape=(c.numSensorNeurons,c.numMotorNeurons), dtype='object')
        # self.weights = np.empty(shape=(a,b), dtype='object')
        for row in range(0, c.numSensorNeurons):
            for col in range(0, c.numMotorNeurons):
                self.weights[row,col] = np.random.rand()
        self.weights = self.weights * 2 - 1

    def Evaluate(self, directOrGUI):
        pass
        # self.Create_Body()
        # self.Create_Brain()

        # os.system("start /B py simulate.py " + directOrGUI + " " + str(self.myID))

        # fitnessFileName = "fitness" + str(self.myID) + ".txt"
        # while not os.path.exists(fitnessFileName):
        #     time.sleep(0.01)

        # fitnessFile = open(fitnessFileName, "r")
        # self.fitness = float(fitnessFile.read())
        # print("self.fitness = " + str(self.fitness))
        # fitnessFile.close()

    def Start_Simulation(self, directOrGUI):
        
        self.Create_Body()
        self.Create_Brain()

        #print("start /B py simulate.py " + directOrGUI + " " + str(self.myID))
        os.system("start /B py simulate.py " + directOrGUI + " " + str(self.myID))
        

    def Wait_For_Simulation_To_End(self):
        fitnessFileName = "fitness" + str(self.myID) + ".txt"
        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)

        fitnessFile = open(fitnessFileName, "r")
        self.fitness = float(fitnessFile.read())
        # print("\nself.fitness = " + str(self.fitness))
        fitnessFile.close()
        os.system("del " + fitnessFileName)

    def Mutate(self):
        randRow = random.randint(0,c.numSensorNeurons-1)
        randCol = random.randint(0,c.numMotorNeurons-1)

        self.weights[randRow, randCol] = random.random()*2 - 1

    def Set_ID(self, newID):
        self.myID = newID

    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        length = 1
        width = 1
        height = 1
        x = -2
        y = -2
        z = height/2

        # pyrosim.Send_Cube(name="Box", pos = [x,y,z], size = [length,width,height])

        pyrosim.End()

    def Create_Body(self):
        self.Create_World()

        pyrosim.Start_URDF("body.urdf")

        a = 0.5

        pyrosim.Send_Cube(name="Torso", pos = [0, 0, 1.25], size = [0.5, 0.5, 0.5])

        pyrosim.Send_Cube(name="LeftLegR", pos = [-a/2, 0, 0], size = [a, 0.2, 0.2])
        pyrosim.Send_Cube(name="RightLegR", pos = [a/2, 0, 0], size = [a, 0.2, 0.2])

        pyrosim.Send_Cube(name="UpLegR", pos = [0, 0, a/2], size = [0.2, 0.2, a])
        pyrosim.Send_Cube(name="DownLegR", pos = [0, 0, -a/2], size = [0.2, 0.2, a])

        pyrosim.Send_Cube(name="FrontLegR", pos = [0, a/2, 0], size = [0.2, a, 0.2])
        pyrosim.Send_Cube(name="BackLegR", pos = [0, -a/2, 0], size = [0.2, a, 0.2])

        # pyrosim.Send_Cube(name="LeftLegP", pos = [-a, 0, 0], size = [a, 0.2, 0.2])
        # pyrosim.Send_Cube(name="RightLegP", pos = [a, 0, 0], size = [a, 0.2, 0.2])

        # pyrosim.Send_Cube(name="UpLegP", pos = [0, 0, a], size = [0.2, 0.2, a])
        # pyrosim.Send_Cube(name="DownLegP", pos = [0, 0, -a], size = [0.2, 0.2, a])


        pyrosim.Send_Joint( name = "Torso_LeftLegR", parent= "Torso", child = "LeftLegR", \
            type = "prismatic", position = [-0.25, 0, 1.25], jointAxis="1 0 0")
        pyrosim.Send_Joint( name = "Torso_RightLegR", parent= "Torso", child = "RightLegR", \
            type = "prismatic", position = [0.25, 0, 1.25], jointAxis="1 0 0")

        pyrosim.Send_Joint( name = "Torso_UpLegR", parent= "Torso", child = "UpLegR", \
            type = "prismatic", position = [0, 0, 1.5], jointAxis="0 0 1")
        pyrosim.Send_Joint( name = "Torso_DownLegR", parent= "Torso", child = "DownLegR", \
            type = "prismatic", position = [0, 0, 1], jointAxis="0 0 1")

        pyrosim.Send_Joint( name = "Torso_FrontLegR", parent= "Torso", child = "FrontLegR", \
            type = "prismatic", position = [0, 0.25, 1.25], jointAxis="0 1 0")
        pyrosim.Send_Joint( name = "Torso_BackLegR", parent= "Torso", child = "BackLegR", \
            type = "prismatic", position = [0, -0.25, 1.25], jointAxis="0 1 0")


        # pyrosim.Send_Joint( name = "LeftLegR_LeftLegP", parent= "LeftLegR", child = "LeftLegP", \
        #     type = "prismatic", position = [-a/2, 0, 0], jointAxis="1 0 0")
        # pyrosim.Send_Joint( name = "RightLegR_RightLegP", parent= "RightLegR", child = "RightLegP", \
        #     type = "prismatic", position = [a/2, 0, 0], jointAxis="1 0 0")

        # pyrosim.Send_Joint( name = "UpLegR_UpLegP", parent= "UpLegR", child = "UpLegP", \
        #     type = "prismatic", position = [0, 0, a/2], jointAxis="0 0 1")
        # pyrosim.Send_Joint( name = "DownLegR_DownLegP", parent= "DownLegR", child = "DownLegP", \
        #     type = "prismatic", position = [0, 0, -a/2], jointAxis="0 0 1")



        pyrosim.End()

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")
        
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "LeftLegR")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "RightLegR")
        pyrosim.Send_Sensor_Neuron(name = 3 , linkName = "UpLegR")
        pyrosim.Send_Sensor_Neuron(name = 4 , linkName = "DownLegR")
        pyrosim.Send_Sensor_Neuron(name = 5 , linkName = "FrontLegR")
        pyrosim.Send_Sensor_Neuron(name = 6 , linkName = "BackLegR")
        # pyrosim.Send_Sensor_Neuron(name = 5 , linkName = "LeftLegP")
        # pyrosim.Send_Sensor_Neuron(name = 6 , linkName = "RightLegP")
        # pyrosim.Send_Sensor_Neuron(name = 7 , linkName = "UpLegP")
        # pyrosim.Send_Sensor_Neuron(name = 8 , linkName = "DownLegP")


        pyrosim.Send_Motor_Neuron(name = 7 , jointName = "Torso_LeftLegR")
        pyrosim.Send_Motor_Neuron(name = 8 , jointName = "Torso_RightLegR")
        pyrosim.Send_Motor_Neuron(name = 9 , jointName = "Torso_UpLegR")
        pyrosim.Send_Motor_Neuron(name = 10 , jointName = "Torso_DownLegR")
        pyrosim.Send_Motor_Neuron(name = 11 , jointName = "Torso_FrontLegR")
        pyrosim.Send_Motor_Neuron(name = 12 , jointName = "Torso_BackLegR")
        # pyrosim.Send_Motor_Neuron(name = 13 , jointName = "LeftLegR_LeftLegP")
        # pyrosim.Send_Motor_Neuron(name = 14 , jointName = "RightLegR_RightLegP")
        # pyrosim.Send_Motor_Neuron(name = 15 , jointName = "UpLegR_UpLegP")
        # pyrosim.Send_Motor_Neuron(name = 16 , jointName = "DownLegR_DownLegP")



        for currentRow in range(0, c.numSensorNeurons):
            for currentColumn in range(0,c.numMotorNeurons):
                # print("i = " + str(i) + ", j = " + str(j))
                pyrosim.Send_Synapse( sourceNeuronName = currentRow , targetNeuronName = currentColumn+c.numSensorNeurons , weight = self.weights[currentRow][currentColumn] )

        pyrosim.End()