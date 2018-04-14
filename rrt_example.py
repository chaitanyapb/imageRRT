# Created on Thu Mar 08 18:44:53 2018
# Author: Chaitanya Pb

#%% Package imports

import time
import rrts
import pygame
from img_env import ImageEnv

#%% Define variables

RED = (255, 0, 0) # Red color
GREEN = (0, 255, 0) # Green color
BLUE = (0, 0, 255) # Blue color

START = (50, 150) # Start position
GOAL = (450, 350) # Goal position

step_size = 10 # RRT Step Size

IMG_NAME = "images/random.png" # Image file

#%% Start PyGame

pygame.init()
clock = pygame.time.Clock()

img = pygame.image.load(IMG_NAME)
pixacc = pygame.surfarray.array2d(img)

screen = pygame.display.set_mode(img.get_size())
screen.fill((0, 0, 0))
screen.blit(img, (0, 0))

pygame.draw.circle(screen, RED, START, 5)
pygame.draw.circle(screen, GREEN, GOAL, 5)

#%% Instantiate Image Environment

env = ImageEnv(pixacc)
start = time.time()

#%% RRT (Connect or Extend)

rrt = rrts.RRT(env, step_size, 'extend')
rrt.start_tree(START, GOAL)

samples = 0
done = False
goalFound = False
drawPath = True

# Until closed
while not done:
    
    # Check for PyGame Quit
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
    
    # If goal not found, grow RRT
    if not goalFound:
        goalFound, new_nodes = rrt.extend_tree(stopAtGoal=True)
        samples += 1
        
        # Draw new growth in PyGame
        for node in new_nodes:
            pygame.draw.line(screen, RED, node.state, node.parent.state)
    
    # If goal found, find path and draw
    if goalFound and drawPath:
        time_taken = time.time() - start
        path, cost = rrt.find_path(GOAL)
        
        # Draw found path in PyGame
        for node in path:
            if node.parent != None:
                pygame.draw.line(screen, GREEN, node.state, node.parent.state, 3)
        drawPath = False
    
    # Update PyGame display
    pygame.display.update()

tree_size = len(rrt.tree)

#%% Close PyGame

#pygame.image.save(screen, "imagerrt.png")
pygame.quit()

#%% Print statistics

print "Samples Drawn =", samples
print "Time Taken =", time_taken, "s"
print "Path Length =", len(path)-1
print "Path Cost =", cost
print "Total Tree Size =", tree_size
