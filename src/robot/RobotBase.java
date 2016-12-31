package robot;
import simbad.sim.Agent;
import simbad.sim.LampActuator;
import simbad.sim.RangeSensorBelt;
import simbad.sim.RobotFactory;

import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;
/**
 * Created by 63289 on 2016/12/29.
 */
public class RobotBase extends Agent {
    // 全局目标坐标
    protected Vector2d goal = new Vector2d(8, 8);
    protected Vector3d goal3d = new Vector3d(8, 0, 8);
    protected Vector3d origin = null;
    protected RangeSensorBelt bumpers;
    protected LampActuator lamp;
    public RobotBase(Vector3d origin, Vector3d goal3d, String name){
        super(origin,name);
        //震动感应器
        bumpers = RobotFactory.addBumperBeltSensor(this);
        //指示灯
        lamp = RobotFactory.addLamp(this);
        this.origin = origin;
        this.goal3d = goal3d;
        this.goal.setX(goal3d.getX());
        this.goal.setY(goal3d.getZ());
    }
    public void initBehavior() {
    }
    protected boolean checkGoal() {
        Point3d currentPos = new Point3d();
        getCoords(currentPos);
        Point3d goalPos = new Point3d(goal3d.x, goal3d.y, goal3d.z);
        return currentPos.distance(goalPos) <= 1;
    }
}
