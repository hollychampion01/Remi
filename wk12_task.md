4.1.1 \- Maze path generation:

Current implementation:

1. Convert to grayscale  
2. Black out the background using the hsv picker (for easier autocrop via edge detection with opencv)  
3. Autocrop  
4. Occupancy map (from individual assignment (not used in later code below))  
5. Using canny edges to detect obstacles  
6. Creating a dual grid system (wall node and path node)   
7. Connects all path nodes that is not in an obstacle  
8. bfs/direction implementation

4.1.2  
Current implementation:  
Using 3.3 code

Future implementation concept:

1. Set distance to travel from cell to cell  
2. Using Lidar side lidar to hug the wall and maintain a set distance between the lidar and the side wall with 4 different conditions (right & left wall present, left wall only present, right wall only present, no wall present (just don’t rely on lidar)) (maybe include front lidar sensor as backup?)  
3. Rotate 90 degrees (because of the implementation in step 2, would be more lenient on errors)  
4. Have it follow the string command text

4.2  
In maze\_analysis\_2:

1. Prm start and end node is using the position based off the grid system in bfs (grid position to pixel coordinate conversion)

Maybe it needs:

1. Code to rotate in the angle that the prm path  
2. Distance would not be the same as bfs or equal to each line

4.3

Using 4.1.2 code for the navigation

Extra implementation:  
Initially giving a start and end position with orientation.

1. Turns on  
2. Scans with lidar  
3. Detects which walls are present  
4. Display the findings within the oled display  
5. Goes to the next cell

Exploration logic needs to be implemented  
Robot needs to return back to the starting position and complete the maze again in the shortest path

