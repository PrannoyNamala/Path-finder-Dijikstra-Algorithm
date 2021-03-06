# Path-finder-Dijikstra-Algorithm

This project is a submission towars project-2 for the course ENPM661 taken in the Spring 2021 semester. 

Libraries used:
Numpy
Math
{visualization Libraries}
Opencv-Python
Pygame

There are two scrits in this repository. 
The script map_creator.py creates the maze on which the algorithm needs to be implemented. With the map given as reference and using opecv functions for drawing the obstacles, the map was created with white pixels representing obstacles in a graysecle image. The script consists of a function which is being used in the implementation.py script to create the map.

The script implementation.py is the scrippt to run to see the results. The input to the script is the location of start and end nodes. The output is a video visualizing the implementation. In the video, the obstacles are black, the visited neodes are represented by red and the shortest path is given by blue. The visualization is implemented using Pygame and Opencv. Dijkstra's algorithm has been used to plot the path from the given start node to goal node. 

To run the program, run the script implementation.py and enter the start and goal node. The result will be in the animation.avi video file. Please make sure not to close the pygame gameDisplay window until it closes itself. This will result in an imcomplete visualization video. For faster visualization, frame rate can be increeased (currently set to 750 frames per second). Running on terminal gives the best results. 

The video has been compresed to upload into github. 
