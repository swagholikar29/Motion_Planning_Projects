# Motion_Planning_Projects

This project implements various search based planners (**Dijsktra, A star, D star**) and sampling-based planners(**RRT, RRT star and Informed RRT star**).The planners are
implemented on a 2d map. The map is given as input in the .csv format in the matrix form where '0' value corresponds to free location and '1' value 
corresponds to object location. Each of the folder contains *main.py* file which is responsible for executing the code. The output folder contains the output from the algorithm
as the image file showing the map and the resultant path. To run any of the algorithm, download the corresponding folder and run 
*python3 main.py*. The map can be changed by chaning the values in the .csv file. 

## Resultant path from search based planners:
### Dijkstra
![Dijsktra](Dijkstra_A-star_Algorithms_Implementation/output/dijsktra.png)
### A*
![A star](Dijkstra_A-star_Algorithms_Implementation/output/Astar.png)
### D* static path
![D_star static](Informed_RRT_star_D_star_Algorithms_Implementation/output/Dstar_static.png)
### D* dynamic path
![D_star step 1](Informed_RRT_star_D_star_Algorithms_Implementation/output/Dstar_dyn1.png)
## Resultant path from sampling based planners:
### RRT
![RRT star](RRT_RRT-star_Implementation/output/RRT.png)
### RRT*
![RRT star](RRT_RRT-star_Implementation/output/RRT_star.png)

### PRM bridge planner
![PRM bridge planner](PRM_Algorithm_Implementation/Outputs/Bridge_Sampling.png)
### PRM gaussian planner
![PRM gaussian planner](PRM_Algorithm_Implementation/Outputs/Gaussian_Sampling.png)
### PRM random planner
![PRM random planner](PRM_Algorithm_Implementation/Outputs/Random_Sampling.png)
### PRM uniform planner
![PRM uniform planner](PRM_Algorithm_Implementation/Outputs/Uniform_Sampling.png)
### Informed RRT*
![Informed RRT*](Informed_RRT_star_D_star_Algorithms_Implementation/output/Infomed_RRT_star.png)

