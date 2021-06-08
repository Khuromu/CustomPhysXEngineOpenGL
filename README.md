BROAD PHASE:
For the exercize I tried to apply the Sweep & Prune algorithm first, and could not handle the Dynamic AABB Tree.
Because I only scratched the surface, I deleted the beginning of my tree.

Press F4 for Debug Render to display the AABBs.

For sorting in the SPBroadPhase, I redefined the operator < like after some tries based on this method:
https://stackoverflow.com/questions/1380463/sorting-a-vector-of-custom-objects


NARROW PHASE:
To begin computing collisions, I'd rather modify the Math.h and CPolygon classes, for I'd rather start from the polygon itself.

Press F4 to see the Gravity center of each shape as well as Minkowski Difference.

I used the lesson and this helpful pdf to understand GJK better:
http://box2d.org/files/GDC2010/GDC2010_Catto_Erin_GJK.pdf

And for the Convex Hull computation, I tried to do something adapted from the Jarvis' algorithm here:
https://www.geeksforgeeks.org/convex-hull-set-1-jarviss-algorithm-or-wrapping/