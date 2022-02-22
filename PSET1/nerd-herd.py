'''
Harvard CS 286 Spring 2022
'''

import random
from ninja_turtles import NinjaTurtles
import rospy
import time
import math

class Robot:
    def __init__(self, x, y, name):
        # index for row position in grid env
        self.x = x
        # index for column position in grid env
        self.y = y
        self.neighbors = set()
        self.sim_bot = NinjaTurtles(x,y,name)
        self.sim_bot.remove_bot('turtle1')
        self.sim_bot.add_bot()

class Env:
    def __init__(self, bots, size=10):
        # number of rows and columns in square grid
        self.size = size 
        # list of Robot objects
        self.bots = bots 
        # number of Robots in this Env
        self.num_bots = len(bots)
        # 2D list containing sets, empty or otherwise, of Robot objects at each coordinate location
        self.grid = self.update_grid()
        
        # List to store different flocks
        self.flocks = []
        
        self.steps = 0

    ################ General Helper Functions ######################        
    def move_bot(self, bot, move_cmd):
        '''
        Update position of bot (Robot obj) using move_cmd (tuple).
        Note that move_cmd = (x,y) where x and y are each integers between -1 and 1, inclusive.
        '''
        bot.x += move_cmd[0]
        bot.y += move_cmd[1]
        if bot.x >= self.size:
            bot.x = self.size-1
        if bot.x < 0:
            bot.x = 0
        if bot.y >= self.size:
            bot.y = self.size-1
        if bot.y < 0:
            bot.y = 0
        
        bot.sim_bot.go_to(bot.x,bot.y)

    
    def update_grid(self):
        grid = [[set() for i in range(self.size)] for i in range(self.size)]
        for b in self.bots:
            grid[b.x][b.y].add(b)
        return grid


    def display_grid(self):
        self.update_grid()
        # prints grid with number of bots in each coordinate location
        print("Grid["+("%d" %self.size)+"]["+("%d" %self.size)+"]")
        for j in range(self.size-1,-1,-1):
            print(j ,'|', end =" ")
            for i in range(0,self.size):
                print(len(self.grid[i][j]), end ="  ")
            print()
        
        print("--", end="  ")
        for i in range(0,self.size):
            print("-", end ="  ")
        print()

        print("  ", end="  ")
        for i in range(0,self.size):
            print(i, end ="  ")
        print()
    
    
    def _move_towards_step(self, bot, loc):
        '''
        Moves a bot (Robot obj) by one step towards the location.
        '''

        x = 1 if (loc[0] > bot.x) else -1 if (loc[0] < bot.x) else 0
        y = 1 if (loc[1] > bot.y) else -1 if (loc[1] < bot.y) else 0

        return (x,y)


    def _move_away_step(self, bot, loc):
        '''
        Moves a bot (Robot obj) by one step away from a location.
        '''

        to_x, to_y = self._move_towards_step(bot, loc)

        xs = [-1, 0, 1]
        xs.remove(to_x)

        ys = [-1, 0, 1]
        ys.remove(to_y)

        #  To ensure the bot moves in a random direction away from a location
        x = random.choice(xs)
        y = random.choice(ys)

        return (x, y)


    def _move_random_step(self):
        '''
        Moves a bot (Robot obj) by one step towards a random location.
        '''
        # Get a random direction
        x_step, y_step = random.randint(-1, 1), random.randint(-1, 1)

        # Make sure this random direction is not staying in place
        while (x_step == 0 and y_step == 0):
            x_step, y_step = random.randint(-1, 1), random.randint(-1, 1)

        return (x_step, y_step)
    

    def get_centroid(self, group = None):
        '''
        Calulcate the centroid of a flock using bot (Robot Obj) positions
        '''
        # Use optional arguments to account for providing centroids of:
        #       --> The global list of robots 
        #       --> Each individual flock

        if group is None:
            group = self.bots

        x_c, y_c = 0,0

        for bot in group:
            x_c += bot.x
            y_c += bot.y
        
        x_c /= len(group)
        y_c /= len(group)

        return (round(x_c), round(y_c))
        

    
    def bot_sense(self, bot, sense_r):
        '''
        Get the neighboring robots of a bot (Robot obj) within its sensing radius
        Hint: self.grid stores the positions of all the bots (Robot obj) in a given iteration. This can be used to find the neightbors of a bot using its position.
        Note: A bot is not a neighbor of itself.
        '''

        def dist(x1, y1, x2, y2):
            return math.sqrt(((x2 - x1) ** 2) + ((y2 - y1) ** 2))
        
        for obot in self.bots:
            if bot is not obot:
                if dist(bot.x, bot.y, obot.x, obot.y) <= sense_r:
                    bot.neighbors.add(obot)



    def update_flocks(self):
        '''
        Generate flock(s) at each timestep based on the position of each robot and the robots within its neighborhood
        '''
        
        flocks = []

        seen = set()

        # Helper function that creates a flock based on a bot, neighbors, neighbor's neighbors, etc.. 
        def flock_search(bot, flock):
            if bot not in seen:
                seen.add(bot)
                flock.append(bot)
                for b in bot.neighbors:
                    flock_search(b, flock)

        # Look through all bots to create flocks
        for bot in self.bots:
            if bot not in seen:
                flock = []
                flock_search(bot, flock)
                flocks.append(flock)
        
        # Update global flock list
        self.flocks = flocks

    ################ General Helper Functions ######################


    ################ Centralized communication ######################
    def flock(self, loc, t=5):
        '''
        Aggregate all bots to grid coordinate loc (tuple)
        Then have the flock safe wander for t (int) steps.
        Afterwards, disperse. 
        Display the grid after each of these steps, including after each safe wander interation.
        '''
        print("AGGREGATE")
        self.aggregate(loc)
        self.display_grid()
        time.sleep(3)
        
        for count in range(t):
            print("SAFE WANDER", count)
            self.safe_wander(True)
            self.display_grid()
        
        time.sleep(3)
        print("DISPERSE")
        self.disperse()
        self.display_grid()


    def aggregate(self, loc):
        '''
        Move all bots to grid coordinate loc (tuple).
        After this method is called, all aggregation should be complete (each bot will have taken all
        steps, likely more than one, to completely aggregate.)
        
        Use move_bot() and _move_towards() functions
        '''
        # Have each bot move a step at a time until reaching the location
        while True:
            all_at_loc = True

            for bot in self.bots:
                cmd = self._move_towards_step(bot, loc)
                self.move_bot(bot, cmd)

                if bot.x != loc[0] or bot.y != loc[1]:
                    all_at_loc = False
        
            if all_at_loc:
                break

        
    
    def safe_wander(self, flock=False):
        '''
        Move each bot one step. 
        If flock, all bots in a flock move in same random direction.
        Otherwise, each bot moves in its own random direction

        Use move_bot() and _move_random_step() functions
        '''

        if flock:
            cmd = self._move_random_step()
            for bot in self.bots:
                self.move_bot(bot, cmd)
        else:
            for bot in self.bots:
                self.move_bot(bot, self._move_random_step())
            
    def disperse(self):
        '''
        Move all bots away from centroid, each in a random direction, for 3 steps.
        Use the move_bot(), _move_away_step() and get_centroid() functions.
        '''

        cen = self.get_centroid()
        for _ in range(3):
            for bot in self.bots:
                self.move_bot(bot, self._move_away_step(bot, cen))
    
    ################ Centralized communication ######################


    ################ Decentralized Communication ######################
    def flock_sense(self, sense_r, t=5):
        '''
        Aggregate all bots using sensing radius sense_r.
        Then have the flock(s) safe wander for t (int) steps.
        Afterwards, disperse flock/s beyond aggregation centroid/s. 
        Display the grid after each of these steps, including after each safe wander interation.
        '''
        print("AGGREGATE")
        self.aggregate_sense(sense_r)
        time.sleep(3)

        for count in range(t):
            print("SAFE WANDER", count)
            self.safe_wander(True)
        
        time.sleep(3)
        print("DISPERSE")
        self.disperse_sense()
        self.display_grid()


    # Helper function to combine bot_sense & update_flock
    def create_flocks(self, sense_r):
        for bot in self.bots:
            self.bot_sense(bot, sense_r)

        self.update_flocks()

    # Helper function to have all flocks move to their centroid
    def move_to_centroid(self):
        for flock in self.flocks:
            centroid = self.get_centroid(flock)

            # Have each bot move a step at a time until reaching the location/centroid
            while True:
                flock_at_loc = True
                for bot in flock:

                    self.move_bot(bot, self._move_towards_step(bot, centroid))

                    if bot.x != centroid[0] or bot.y != centroid[1]:
                        flock_at_loc = False
                
                if flock_at_loc: 
                    break

    def aggregate_sense(self, sense_r):
        '''
        Aggregate bots into one or more flocks, each using sensing radius of sense_r (int).
        Use bot_sense() and update_flocks() functions
        '''
        # Continuously have bots wander and merge together until they are in a single flock
        while True:
            self.create_flocks(sense_r)
            self.move_to_centroid()

            if len(self.flocks) == 1:
                break
            
            self.safe_wander_sense(True)

    def safe_wander_sense(self, flock=False):
        '''
        Move each bot one step. 
        If flock, all bots in a flock move in same random direction.
        Otherwise, each bot moves in its own random direction
        '''
        
        if flock:
            for f in self.flocks:
                cmd = self._move_random_step()
                for bot in f:
                    self.move_bot(bot, cmd)
        else:
            for bot in self.bots:
                self.move_bot(bot, self._move_random_step())


    def disperse_sense(self):
        '''
        Move all bots away from their respective flock's centroid.
        '''

        for flock in self.flocks:
            cen = self.get_centroid(flock)
            for _ in range(3):
                for bot in flock:
                    self.move_bot(bot, self._move_away_step(bot, cen))

    ################ Decentralized Communication ######################


if __name__ == "__main__":
    rospy.init_node("Cowabunga")
    
    ########################### TESTING ###############################

    ################ Environment 1 ################

    bot1 = Robot(1,1,'t1')
    bot2 = Robot(9,1,'t2')
    bot3 = Robot(9,9,'t3')
    bot4 = Robot(1,9,'t4')

    bots = [bot1, bot2, bot3, bot4]

    env = Env(bots, 10)

    env.display_grid()
    
    ################ Environment 2 ################

    # bot1 = Robot(1,1,'t1')
    # bot2 = Robot(1,7,'t2')
    # bot3 = Robot(3,3,'t3')
    # bot4 = Robot(3,6,'t4')
    # bot5 = Robot(5,3,'t5')
    # bot6 = Robot(5,6,'t6')
    # bot7 = Robot(7,1,'t7')
    # bot8 = Robot(7,7,'t8')

    # bots = [bot1, bot2, bot3, bot4, bot5, bot6, bot7, bot8]

    # env = Env(bots, 8)

    # env.display_grid()


    # CENTRALIZED TESTS
    #  env.flock((5,5))

    # DECENTRALIZED TESTS
    env.flock_sense(2)
    # env.flock_sense(5)
    # env.flock_sense(10)