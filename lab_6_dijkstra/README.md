Randomly generate a 4x4 grid with obstacles and then uses dijskstra's to find a path from the bottom left corner to the top right corner.  

To Run:  
```
python lab_6.py
```

Output is displayed as follow:
```
starting vertex:  (0, 0)
destination vertex:  (3, 3)
 .  . [ ] . 
 .  . [ ] . 
 .  . [ ] . 
 .  .  .  . 


path:  0 -> 1 -> 2 -> 3 -> 7 -> 11 -> 15
 ```
 . represents and empty grid spot  
 [ ] is an obstacle  
 The grid is indexed 0 to 15 starting in the bottom left corner and moving right then up
