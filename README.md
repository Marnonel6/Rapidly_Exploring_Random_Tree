
# Rapidly-exploring Random Tree <br><br>

### Brief overview
A rapidly exploring random tree (RRT) is an algorithm designed to efficiently search nonconvex, high-dimensional spaces by randomly building a space-filling tree. The tree is constructed incrementally from samples drawn randomly from the search space and is inherently biased to grow towards large unsearched areas of the problem. RRTs were developed by Steven M. LaValle and James J. Kuffner Jr. They easily handle problems with obstacles and differential constraints (nonholonomic and kinodynamic) and have been widely used in autonomous robotic motion planning.

### Results
* RRT with circular obstacles

![RRT_a](https://user-images.githubusercontent.com/60977336/207223342-acc95c96-8a5f-41d0-8866-852356b79a2d.gif)

![RRT_m](https://user-images.githubusercontent.com/60977336/207223345-e3ba63a7-cd43-41b8-97ae-d2876072e3e3.gif)


### Algorithm descriptions
* pseudocode for the basic algorithm with no obstacles:

![Screenshot from 2022-12-12 22-01-34](https://user-images.githubusercontent.com/60977336/207223618-1f7d96b1-0d95-4e3a-93bc-c78345fa2c8a.png)

* **CHECK_POINT_COLLISION** Checks if the point collides with the obstacles
* **CHECK_EDGE_COLLISION** Checks if the edge collides with the obstacles
* **CHECK_COLLISION_FREE_PATH** Check if there is a collision-free path to the goal position <br><br>
