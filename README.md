# Box2D-Project-CS251
Team No. - 23  Name - Cipher

Instructions-
1.Important-While changing the edited file in src (.cpp or .hpp types) if you have any problem in "make" command,just go the replaced file and add
add a comment ,so that terminal recognozes the change and succefully compiles the new files. 
2.To drag an object,double click it and move it around.






PART1)

Changes : 
1) Before 
	for(int i=0;i<10;i++)
  After 

	for(int i = 0;i<0;i++)  // To make no. of spheres = 0

2) 
Before : friction of dominos was set to 0.1f;
After: Friction of last domino set to 0.012f so, it can work inplace of ball

Part 2) 
	
	Decreased The Density of heavy sphere to 0.2f only.


Part 3) 

1)Overrided the three functions 
mouse_move() , mouse_up(), mouse_down,

2)Overrided  the Querycallback function

ALL the dynamic body present in gui can be dragged using these.

CITATIONS-

1) http://www.binarytides.com/mouse-joint-box2d-javascript/
2)learnt from the Codes of Sample Testbed as provided in the cs251_base.tar.gz archives 
(The querycallback function was implemented to make all dynamic balls dragable not only one).
3)https://www.youtube.com/watch?v=dF5dUiJtYng
4)http://www.box2d.org/manual.html#_Toc258082974
