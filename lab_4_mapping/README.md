Requirements:
* ROS

Simulation showing a robot follow a given path. A ultrasonic sensor turned inwards detects objects within a certain range. A map is oncstructed in the terminal using the information.   


To run:  
```
roscore
python sparki-simulator.py
python lab_4.py
```

Output is displayed as follow:
```
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
_ _ R _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ X X _ _ _ _ _ _ _ _ _ _ _ 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ X X X _ _ _ _ _ _ _ _ _ _ 
_ _ _ _ _ _ _ X X _ _ _ _ _ _ _ _ _ _ _ _ _ _ X X X _ _ _ _ _ _ _ _ _ _ 
_ _ _ _ _ _ _ _ X X _ _ _ _ _ _ _ _ _ _ _ _ X X X _ X _ _ _ _ _ _ _ _ _ 
_ _ _ _ _ _ X _ _ X _ _ _ _ _ _ _ _ _ _ _ X X X _ X X _ _ _ _ _ _ _ _ _ 
_ _ _ _ _ _ X X _ X _ X _ _ _ _ _ _ _ X X X X _ X X X _ _ _ _ _ _ _ _ _ 
_ _ _ _ _ X X _ _ _ _ X _ _ _ _ _ _ _ X X X X _ _ X X X _ _ _ _ _ _ _ _ 
_ _ _ _ _ X _ _ _ _ _ _ _ _ _ _ _ X X _ X X _ _ _ X _ _ _ _ _ _ _ _ _ _ 
_ _ _ _ _ X _ _ _ X X X X X _ _ _ _ X _ X X _ _ _ X X _ X _ _ _ _ _ _ _ 
_ _ _ _ _ _ _ _ _ _ _ _ _ X X _ _ _ _ X X _ _ _ _ _ _ _ X _ _ _ _ _ _ _ 
_ _ _ _ _ _ _ _ _ _ _ _ _ X _ _ _ _ X X _ _ _ _ _ _ X _ _ _ _ _ _ _ _ _ 
_ _ _ _ _ _ _ X X X _ _ _ X X X _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
_ _ _ _ _ _ X X X X X X X X X X X X X X X X X X X X _ _ _ _ _ _ _ _ _ _ 
_ _ _ _ _ _ _ _ _ _ _ X X _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

 ```
 _ represents and empty grid spot  
 X is an object  
 R is the robots position

