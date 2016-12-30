package robot;

import simbad.sim.Agent;
import simbad.sim.LampActuator;
import simbad.sim.RangeSensorBelt;
import simbad.sim.RobotFactory;

import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

/**
 * Created by 63289 on 2016/12/28.
 * 使用人工势场法进行运动的agent
 */
public class ForceRobot extends RobotBase {

    protected static final double repelConstant = 100.0;// 斥力系数
    protected double attractConstant = 30.0;// 引力系数
    protected double speed=0.5;
    public ForceRobot(Vector3d origin, Vector3d goal3d, String name) {
        super(origin, goal3d, name);
        sonars = RobotFactory.addSonarBeltSensor(this);//the sonar sensor
        speed=1;
    }
    public void initBehavior() {
        setTranslationalVelocity(speed);
    }
    public void performBehavior() {
        if (getCounter() % 5 == 0) {
            checkGoal();
            //获取速度
            Vector3d velocity = getVelocity();
            //前进的方向向量
            Vector2d direct = new Vector2d(velocity.z, velocity.x);
            Point3d p = new Point3d();
            getCoords(p);
            Vector2d pos = new Vector2d(p.z, p.x);
            // 正前方障碍物距离
            double d0 = sonars.getMeasurement(0);
            //左前方障碍物距离
            double d1 = sonars.getMeasurement(1);
            //右前方障碍物距离
            double d2 = sonars.getMeasurement(8);
            //三个方向的斥力
            double rf0 = repelForce(d0, 4.0);
            double rf1 = repelForce(d1, 4.0);
            double rf2 = repelForce(d2, 4.0);
            // 计算斥力的合力
            double k1 = Math.cos(2 * Math.PI / 9);
            double k2 = Math.sin(2 * Math.PI / 9);
            Vector2d vf0 = new Vector2d(0 - rf0, 0);
            Vector2d vf1 = new Vector2d((0 - rf1 * k1), (0 - rf1 * k2));
            Vector2d vf2 = new Vector2d((rf2 * k1), (rf2 * k2));
            Vector2d composition = new Vector2d(vf0.x + vf1.x + vf2.x, vf0.y + vf1.y + vf2.y);//合力
            Vector2d repelForceVector = transform(direct, composition);
            Vector2d toGoal = new Vector2d((goal.x - pos.x), (goal.y - pos.y));
            double disGoal = toGoal.length();
            //利用目标的吸引力来引导
            double goalForce = attractForce(disGoal);
            Vector2d goalForceVector = new Vector2d((goalForce * toGoal.x / disGoal), (goalForce * toGoal.y / disGoal));
            double x = repelForceVector.x + goalForceVector.x;
            double y = repelForceVector.y + goalForceVector.y;
            //合力
            Vector2d allForces = new Vector2d(x, y);
            double angle = getAngle(direct, allForces);
            // 判断转动方向
            if (angle < Math.PI) {
                setRotationalVelocity(angle);
            } else if (angle > Math.PI) {
                setRotationalVelocity((angle - 2 * Math.PI));
            }
            checkHit();
        }
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
                    return v1.angle(v2);
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
        Vector2d global = new Vector2d(1, 0);
        double alfa = getAngle(global, v);
        double beta = getAngle(point, v);

        double k1 = Math.cos(alfa + beta) / Math.cos(beta);
        double k2 = Math.sin(alfa + beta) / Math.sin(beta);

        double x = point.x * k1;
        double y = point.y * k2;

        return new Vector2d(x, y);

    }
    protected Vector3d getVelocity() {
        return this.linearVelocity; //linear velocity
    }
    protected boolean checkHit() {
        if (bumpers.oneHasHit()) {
            lamp.setBlink(true);
            double left = sonars.getFrontLeftQuadrantMeasurement();
            double right = sonars.getFrontRightQuadrantMeasurement();
            double front = sonars.getFrontQuadrantMeasurement();

            if ((front < 0.7) || (left < 0.7) || (right < 0.7)) {
                if (left < right) {
                    // 随机向右转
                    setRotationalVelocity(-1 - (0.1 * Math.random()));
                } else {
                    // 随机向左转
                    setRotationalVelocity(1 - (0.1 * Math.random()));
                }
                setTranslationalVelocity(0);
            }
            return true;
        } else {
            lamp.setBlink(false);
            return false;
        }
    }
    //计算吸引力
    protected double attractForce(double distance)
    {
        double force = attractConstant * distance;
        return force;
    }
    //计算斥力
    protected double repelForce(double distance, double range)
    {
        double force = 0;
        Point3d p = new Point3d();
        getCoords(p);
        Vector2d pos = new Vector2d(p.z, p.x);
        Vector2d toGoal = new Vector2d((goal.x - pos.x), (goal.y - pos.y));
        double disGoal = toGoal.length();
        double n = 0.5;
        if (distance <= range)
        {
            force = (1 / distance - 1 / range) * (1 / distance - 1 / range) * repelConstant;
        }

        return force;
    }
}
