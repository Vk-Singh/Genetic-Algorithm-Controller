from controller import Supervisor
from controller import Keyboard
from controller import Display

import numpy as np
import ga,os,sys,struct
import math

class SupervisorGA:
    def __init__(self):
        # Simulation Parameters
        # Please, do not change these parameters
        self.time_step = 32 # ms
        self.time_experiment = 60 # s
        
        # Initiate Supervisor Module
        self.supervisor = Supervisor()
        # Check if the robot node exists in the current world file
        self.robot_node = self.supervisor.getFromDef("Controller")
        if self.robot_node is None:
            sys.stderr.write("No DEF Controller node found in the current world file\n")
            sys.exit(1)
        # Get the robots translation and rotation current parameters    
        self.trans_field = self.robot_node.getField("translation")  
        self.rot_field = self.robot_node.getField("rotation")
        
        # Check Receiver and Emitter are enabled
        self.emitter = self.supervisor.getDevice("emitter")
        self.receiver = self.supervisor.getDevice("receiver")
        self.receiver.enable(self.time_step)
        
        # Initialize the receiver and emitter data to null
        self.receivedData = "" 
        self.receivedWeights = "" 
        self.receivedFitness = "" 
        self.emitterData = ""
        
        ### Define here the GA Parameters ?
        self.num_generations = 80
        self.num_population = 50
        self.num_elite = 5
        
        # size of the genotype variable
        self.num_weights = 0
        
        # Creating the initial population
        self.population = []
        
        # All Genotypes
        self.genotypes = []
        
        # Display: screen to plot the fitness values of the best individual and the average of the entire population
        self.display = self.supervisor.getDevice("display")
        self.width = self.display.getWidth()
        self.height = self.display.getHeight()
        self.prev_best_fitness = 0.0;
        self.prev_average_fitness = 0.0;
        self.display.drawText("Fitness (Best - Red)", 0,0)
        self.display.drawText("Fitness (Average - Green)", 0,10)
        
        # Black mark
        self.mark_node = self.supervisor.getFromDef("Mark")
        if self.mark_node is None:
            sys.stderr.write("No DEF Mark node found in the current world file\n")
            sys.exit(1)
        self.mark_trans_field = self.mark_node.getField("translation")      

    def createRandomPopulation(self):
        # Wait until the supervisor receives the size of the genotypes (number of weights)
        if(self.num_weights > 0):
            # Define the size of the population
            pop_size = (self.num_population,self.num_weights)
            # Create the initial population with random weights
            self.population = np.random.uniform(low=-1.0, high=1.0, size=pop_size)

    def handle_receiver(self):

        while(self.receiver.getQueueLength() > 0):
            self.receivedData = self.receiver.getData().decode("utf-8")
            typeMessage = self.receivedData[0:7]
            # Check Message 
            if(typeMessage == "weights"):
                self.receivedWeights = self.receivedData[9:len(self.receivedData)] 
                self.num_weights = int(self.receivedWeights)
                #print ("Self weights = {}".format(self.num_weights) )
            elif(typeMessage == "fitness"):  
                self.receivedFitness = float(self.receivedData[9:len(self.receivedData)])
            self.receiver.nextPacket()
        
    def handle_emitter(self):
        if(self.num_weights > 0):
            # Send genotype of an individual
            string_message = str(self.emitterData)
            string_message = string_message.encode("utf-8")
            #print("Supervisor send:", string_message)
            self.emitter.send(string_message)     
        
    def run_seconds(self,seconds):
        #print("Run Simulation")
        stop = int((seconds*600)/self.time_step)
        iterations = 0
        while self.supervisor.step(self.time_step) != -1:
            self.handle_emitter()
            self.handle_receiver()
            if(stop == iterations):
                break    
            iterations = iterations + 1
                
    def evaluate_genotype(self,genotype,generation):
        # Here you can choose how many times the current individual will interact with both environments
        # At each interaction loop, one trial on each environment will be performed
        numberofInteractionLoops = 1
        currentInteraction = 0
        fitnessPerTrial = []
        while currentInteraction < numberofInteractionLoops:
            #######################################
            # TRIAL: TURN RIGHT
            #######################################
            # Send genotype to robot for evaluation
            self.emitterData = str(genotype)
            
            # Reset robot position and physics
            INITIAL_TRANS = [0.007, 0, 0.35]
            INITIAL_TRANS_BOT = [0.007, 0, 0.35]
            self.trans_field.setSFVec3f(INITIAL_TRANS)
            INITIAL_ROT = [-0.5, 0.5, 0.5, 2.09]
            self.rot_field.setSFRotation(INITIAL_ROT)
            self.robot_node.resetPhysics()
            
            # Reset the black mark position and physics
            INITIAL_TRANS = [0.01, -0.03425, 0.193]
            self.mark_trans_field.setSFVec3f(INITIAL_TRANS)
            self.mark_node.resetPhysics()

            # Evaluation genotype 
            self.run_seconds(self.time_experiment)
        
            # Measure fitness
            fitness = self.receivedFitness
                                   
            # Check for Reward and add it to the fitness value here
            # ADD CODE HERE

            current_loc = self.trans_field.getSFVec3f()
            print("mark")
            print("location", current_loc)
            # dist = np.linalg.norm(np.asarray(current_loc) - np.asarray(INITIAL_TRANS_BOT))
            # print ("DIST = {}".format(dist))
            fitness = fitness + 0.75 * ( - (100 * current_loc[2])) + 0.25 * (100 * current_loc[0])
            print ("FITNESS TEST MARK = {}".format(fitness))
            # Add fitness value to the vector
            fitnessPerTrial.append(fitness)
            
            #######################################
            # TRIAL: TURN LEFT No mark
            #######################################
            # Send genotype to robot for evaluation
            self.emitterData = str(genotype)
            
            # Reset robot position and physics
            INITIAL_TRANS = [0.007, 0, 0.35]
            self.trans_field.setSFVec3f(INITIAL_TRANS)
            INITIAL_ROT = [-0.5, 0.5, 0.5, 2.09]
            self.rot_field.setSFRotation(INITIAL_ROT)
            self.robot_node.resetPhysics()
            
            # Reset the black mark position and physics
            INITIAL_TRANS = [0.01, -0.1, 0.193]
            self.mark_trans_field.setSFVec3f(INITIAL_TRANS)
            self.mark_node.resetPhysics()
        
            # Evaluation genotype 
            self.run_seconds(self.time_experiment)
        
            # Measure fitness
            fitness = self.receivedFitness
            #print("Fitness: {}".format(fitness))

            # Check for Reward and add it to the fitness value here
            # ADD CODE HERE
            print("no mark")
            current_loc = self.trans_field.getSFVec3f()
            # dist = np.linalg.norm(np.asarray(current_loc) - np.asarray(INITIAL_TRANS_BOT))
            fitness = fitness + 0.75 * ( - (100 * current_loc[2])) + 0.25 * (-(100 * current_loc[0]))
            print("location", current_loc)
            print ("FITNESS TEST No Mark = {}".format(fitness))

            # Add fitness value to the vector
            fitnessPerTrial.append(fitness)
            
            # End 
            currentInteraction += 1
        
        fitness_mean = np.mean(fitnessPerTrial)
        print ("FITNESS TEST Mean = {}".format(fitness_mean))
        fitness = math.exp(fitness_mean/10)
        current = (generation,genotype,fitness)
        self.genotypes.append(current)  
        print ("FITNESS TEST FINAL = {}".format(fitness))
        
        return fitness

    def run_demo(self):
        # Read File
        genotype = np.load("Best.npy")
        # Send Genotype to controller
        self.emitterData = str(genotype)
        # YOU MAY NEED TO ADJUST THE ROBOT INITIAL POSITION AND ORIENTATION HERE <<<<<<<<<<<<<<<<
        # Reset robot position and physics
        INITIAL_TRANS = [0.007, 0, 0.35]
        self.trans_field.setSFVec3f(INITIAL_TRANS)
        INITIAL_ROT = [-0.5, 0.5, 0.5, 2.09]
        self.rot_field.setSFRotation(INITIAL_ROT)
        self.robot_node.resetPhysics()

        # Reset the black mark position and physics
        INITIAL_TRANS = [0.01, -0.03425, 0.193]
        self.mark_trans_field.setSFVec3f(INITIAL_TRANS)
        self.mark_node.resetPhysics()
        # Evaluation genotype


        self.run_seconds(self.time_experiment)


        # Reset robot position and physics
        INITIAL_TRANS = [0.007, 0, 0.35]
        self.trans_field.setSFVec3f(INITIAL_TRANS)
        INITIAL_ROT = [-0.5, 0.5, 0.5, 2.09]
        self.rot_field.setSFRotation(INITIAL_ROT)
        self.robot_node.resetPhysics()

        # Reset the black mark position and physics
        INITIAL_TRANS = [0.01, -0.1, 0.193]
        self.mark_trans_field.setSFVec3f(INITIAL_TRANS)
        self.mark_node.resetPhysics()

        self.emitterData = str(genotype)
        self.run_seconds(self.time_experiment)
    
    def run_optimization(self):
        # Wait until the number of weights is updated
        while(self.num_weights == 0):
            self.handle_receiver()
            self.createRandomPopulation()
        
        print("starting GA optimization ...\n")
        
        # For each Generation
        for generation in range(self.num_generations):
            print("Generation: {}".format(generation))
            current_population = []   
            # Select each Genotype or Individual
            for population in range(self.num_population):
                genotype = self.population[population]
                # Evaluate
                fitness = self.evaluate_genotype(genotype,generation)
                # Save its fitness value
                current_population.append((genotype,float(fitness)))
                #print(current_population)
                
            # After checking the fitness value of all indivuals
            # Save genotype of the best individual
            best = ga.getBestGenotype(current_population);
            average = ga.getAverageGenotype(current_population);
            np.save("Best.npy",best[0])
            save_name = "Generation_{}.npy".format(generation)
            np.save(save_name, ga.rankPopulation(current_population))
            self.plot_fitness(generation, best[1], average);
            
            # Generate the new population using genetic operators
            if (generation < self.num_generations - 1):
                self.population = ga.population_reproduce(current_population,self.num_elite);
        
        #print("All Genotypes: {}".format(self.genotypes))
        print("GA optimization terminated.\n")   
    
    
    def draw_scaled_line(self, generation, y1, y2): 
        # Define the scale of the fitness plot
        XSCALE = int(self.width/self.num_generations);
        YSCALE = 100;
        self.display.drawLine((generation-1)*XSCALE, self.height-int(y1*YSCALE), generation*XSCALE, self.height-int(y2*YSCALE));
    
    def plot_fitness(self, generation, best_fitness, average_fitness):
        if (generation > 0):
            self.display.setColor(0xff0000);  # red
            self.draw_scaled_line(generation, self.prev_best_fitness, best_fitness);
    
            self.display.setColor(0x00ff00);  # green
            self.draw_scaled_line(generation, self.prev_average_fitness, average_fitness);
    
        self.prev_best_fitness = best_fitness;
        self.prev_average_fitness = average_fitness;
  
    
if __name__ == "__main__":
    # Call Supervisor function to initiate the supervisor module   
    gaModel = SupervisorGA()
    
    # Function used to run the best individual or the GA
    keyboard = Keyboard()
    keyboard.enable(50)
    
    # Interface
    print("Best Individual or (S|s)earch for New Best Individual:")
    while gaModel.supervisor.step(gaModel.time_step) != -1:
        resp = keyboard.getKey()
        if(resp == 83 or resp == 65619):
            print("(R|r)un Best Individual or (S|s)earch for New Best Individual optimisation:")
            gaModel.run_optimization()
            
        elif(resp == 82 or resp == 65619):
            print("(R|r)un Best Individual or (S|s)earch for New Best Individual:")
            gaModel.run_demo()
            
        
