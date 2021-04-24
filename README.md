# Path Finding for Multi-cell Agents

#### CMPT 417 - Intelligent Systems 

#### Group 25: Richard Swann, Matthew Gould 

------

This project extends the classic multi-agent path finding (MAPF) solutions to deal with agents that occupy two cells instead of one and have a orientation. 
All of the Python code is contained in the root directory and the test instance definitions are included in a subdirectory "Instances".
The code is an extension of the starter code supplied for the CMPT 417 Individual Project and operates in the same way.
That is by invoking "run_experiments.py" with appropriate command line arguments. 

The only major differences is that the test instance definitions support defining multi-cell agents. The format is documented in 
\Instances\Test Space Definitions.txt as below

Multicell agent test file
`
8 8 - Width and Height of Test Space
. . @ . . . . .
. . . @ . . . .
. . @ . . . . .
. . . . . . . .
. . . . . . . @
. . . . . @ . .
. . . . . . @ .
. . . . . . . .
5 - Number of start location and goal location lines
0 1 1 3 0 - agent number, start x, start y, goal x, goal y (for multi cell agents there are 2 lines with the same agent)
0 7 6 0 0   (first agent number must be 0)
1 4 3 1 1
2 0 0 5 4
2 1 0 5 6
`




![example](example.gif)
