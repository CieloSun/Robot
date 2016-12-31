package robot;
import javax.vecmath.Vector3d; 
import javax.vecmath.Vector3f;
import simbad.sim.*; 
import simbad.demo.*; 
import simbad.gui.Simbad;
/**
 * Created by 63289 on 2016/12/28.
 */
public class Controller extends Demo
{
	public Controller()
	{
	    Vector3d goal3d=new Vector3d(8, 0, 8);
        light1IsOn = true;
        light1SetPosition(goal3d.getX(),goal3d.getY(),goal3d.getZ());
        light1Color=white;
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
        Wall w5 = new Wall(new Vector3d(6, 0, 3), 2, 2, this);
        add(w5); 
        Wall w6 = new Wall(new Vector3d(-4, 0,-3), 12, 2, this);
        add(w6);
        Wall w7 = new Wall(new Vector3d(-6, 0, -8), 2, 2, this);
        w7.rotate90(1);
        add(w7);
        Wall w8 = new Wall(new Vector3d(5, 0, 4),2, 2, this);
        w8.rotate90(1);
        add(w8);
        Box b1 = new Box(new Vector3d(0,0,0), new Vector3f(3, 5, 3),this);
        add(b1); 
        Box b2 = new Box(new Vector3d(8,0,-4), new Vector3f(3, 3, 1),this);
        add(b2);
        Box b3 = new Box(new Vector3d(2,0,9), new Vector3f(1, 2, 1),this);
        add(b3);
        Box b4 = new Box(new Vector3d(-6,0,2), new Vector3f(1, 2, 8),this);
        add(b4);
        //显示光敏机器人
        add(new LightRobot(new Vector3d(-9, 0, 3),goal3d,"LightRobot"));
        //显示人工势场机器人
        add(new ForceRobot(new Vector3d(-9, 0, -8),goal3d, "ForceRobot"));
	}
	public static void main(String[] args)
	{
		Simbad frame = new Simbad(new Controller(), false);
	}
}