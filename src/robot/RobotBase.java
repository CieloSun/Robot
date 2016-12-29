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
    protected RangeSensorBelt sonars, bumpers;
    protected LampActuator lamp;
    protected Vector3d origin = null;
    protected double speed = 2;

    public void initBehavior() {
        setTranslationalVelocity(speed);
    }

    public RobotBase(Vector3d origin, Vector3d goal3d, String name) {
        super(origin, name);
        bumpers = RobotFactory.addBumperBeltSensor(this);//the bumper sensor
        sonars = RobotFactory.addSonarBeltSensor(this);//the sonar sensor
        lamp = RobotFactory.addLamp(this);//the instruction lump
        this.origin = origin;// the origin position
        this.goal3d = goal3d;
        this.goal.setX(goal3d.getX());
        this.goal.setY(goal3d.getZ());
    }

    protected int getQuadrant(Vector2d vector) //cal the quadrant of the agent
    {
        double x = vector.x;
        double y = vector.y;
        if (x > 0 && y > 0)// first quadrant
        {
            return 1;
        } else if (x < 0 && y > 0)// second quadrant
        {
            return 2;
        } else if (x < 0 && y < 0)// third quadrant
        {
            return 3;
        } else if (x > 0 && y < 0)// fouth quadrant
        {
            return 4;
        } else if (x > 0 && y == 0)// x+
        {
            return -1;
        } else if (x == 0 && y > 0)// y+
        {
            return -2;
        } else if (x < 0 && y == 0)// x-
        {
            return -3;
        } else if (x == 0 && y < 0)// y-
        {
            return -4;
        } else {
            return 0;//original porint
        }
    }

    protected double getAngle(Vector2d v1, Vector2d v2) //cal rad of two vectors
    {

        double k = v1.y / v1.x;
        double y = k * v2.x;
        switch (getQuadrant(v1)) {
            case 1:
            case 4:
            case -1:
                if (v2.y > y) {
                    return v1.angle(v2); //两个向量之间的夹角弧度
                } else if (v2.y < y) {
                    return 2 * Math.PI - v1.angle(v2);
                } else {
                    if (v1.x * v2.x < 0) {
                        return Math.PI;
                    } else {
                        return 0;
                    }
                }
            case 2:
            case 3:
            case -3:
                if (v2.y > y) {
                    return 2 * Math.PI - v1.angle(v2);
                } else if (v2.y < y) {
                    return v1.angle(v2);
                } else {
                    if (v1.x * v2.x < 0) {
                        return Math.PI;
                    } else {
                        return 0;
                    }
                }
            case -2:
                int i = getQuadrant(v2);
                if (i == -4) {
                    return Math.PI;
                } else if (i == -2 || i == -1 || i == 1 || i == 4) {
                    return 2 * Math.PI - v1.angle(v2);
                } else {
                    return v1.angle(v2);
                }
            case -4:
                int j = getQuadrant(v2);
                if (j == -1) {
                    return Math.PI;
                } else if (j == -4 || j == -1 || j == 1 || j == 4) {
                    return v1.angle(v2);
                } else {
                    return 2 * Math.PI - v1.angle(v2);
                }
            default:
                return -1;
        }

    }

    //利用速度方向和受力方向进行运动
    protected Vector2d transform(Vector2d v, Vector2d point) {
        Vector2d global = new Vector2d(1, 0); //（1,0）,means the x-axis
        double alfa = getAngle(global, v); //the rad of v with x
        double beta = getAngle(point, v); //the rad of point with v

        double k1 = Math.cos(alfa + beta) / Math.cos(beta);
        double k2 = Math.sin(alfa + beta) / Math.sin(beta);

        double x = point.x * k1;
        double y = point.y * k2;

        return new Vector2d(x, y);

    }

    protected void checkGoal() //检查是否到达目的地
    {
        boolean checkGoal = false;
        Point3d currentPos = new Point3d();
        getCoords(currentPos); //当前坐标
        Point3d goalPos = new Point3d(goal3d.x, goal3d.y, goal3d.z);
        if (currentPos.distance(goalPos) <= 0.5) // 如果当前距离目标点小于0.5那么即认为是到达
        {
            checkGoal = true;
        } else {
            checkGoal = false;
        }
        if (checkGoal) {
            // 到达目标点，停止运动
            setTranslationalVelocity(0);
            setRotationalVelocity(0);
            lamp.setOn(true);
            return;
        } else {
            lamp.setOn(false);
            setTranslationalVelocity(speed);
        }
    }

    protected Vector3d getVelocity() {
        return this.linearVelocity; //linear velocity
    }

    protected void checkHit() {
        // 检测是否碰撞
        if (bumpers.oneHasHit()) {
            lamp.setBlink(true);
            double left = sonars.getFrontLeftQuadrantMeasurement();
            double right = sonars.getFrontRightQuadrantMeasurement();
            double front = sonars.getFrontQuadrantMeasurement();
            if ((front < 0.7) || (left < 0.7) || (right < 0.7)) {
                if (left < right) {
                    setRotationalVelocity(-1 - (0.1 * Math.random()));// 随机向右转
                } else {
                    setRotationalVelocity(1 - (0.1 * Math.random()));// 随机向左转
                }
                setTranslationalVelocity(0);
            }
        } else lamp.setBlink(false);
    }
}
