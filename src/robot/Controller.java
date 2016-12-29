package robot;

import javax.vecmath.Vector3d; 
import javax.vecmath.Vector3f; 
 
 
import simbad.sim.*; 
import simbad.demo.*; 
import simbad.gui.Simbad;

public class Controller extends Demo
{
	public Controller()
	{
	    //v0 means x, v1 means z, v2 means y of the center position
        // length means the length of wall, height means the height of wall, wd means the width
    	Wall w1 = new Wall(new Vector3d(10, 0, 0), 20, 2, this);
    	//For default, the wall is horizontal
        w1.rotate90(1); 
        add(w1);

        Wall w2 = new Wall(new Vector3d(-10, 0, 0), 20, 2, this);
        w2.rotate90(1);
        add(w2);

        Wall w3 = new Wall(new Vector3d(0, 0, 10), 20, 2, this);
        add(w3); 
        Wall w4 = new Wall(new Vector3d(0, 0, -10), 20, 2, this);
        add(w4); 


        Wall w5 = new Wall(new Vector3d(6, 0, 3), 8, 2, this);
        add(w5); 
        Wall w6 = new Wall(new Vector3d(-4, 0,-4), 12, 2, this);
        add(w6); 

        Wall w9 = new Wall(new Vector3d(-8, 0, -7), 2, 2, this);
        w9.rotate90(1); 
        add(w9); 
        Wall w10 = new Wall(new Vector3d(5, 0, 5),4, 2, this);
        w10.rotate90(1);
        add(w10); 

        Box b1 = new Box(new Vector3d(0,0,0), new Vector3f(3, 3, 3),this);
        add(b1); 
        Box b2 = new Box(new Vector3d(4,0,-4), new Vector3f(3, 3, 1),this);
        add(b2);

        Box b3 = new Box(new Vector3d(2,0,4), new Vector3f(1, 2, 1),this);
        add(b3);

        Arch a1=new Arch(new Vector3d(3,0,7),this);
        a1.rotate90(1);
        add(a1);
         
        add(new AttractForceRobot(new Vector3d(-9, 0, -8), "AttractForceRobot"));
        add(new MyMovRobot(new Vector3d(0, 0, 3), "MyMovRobot"));
	}

	public static void main(String[] args)
	{
		//System.setProperty("j3d.implicitAntialiasing", "true");
		Simbad frame = new Simbad(new Controller(), false);
	}
}