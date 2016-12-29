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
public class AttractForceRobot extends Agent {
    private static final double repelConstant = 100.0;// 斥力系数
    private static final double attractConstant = 30.0;// 引力系数
    private boolean debug = false;// 调试标记
    // 全局目标坐标
    private Vector2d goal = new Vector2d(8, 8);
    private Vector3d goal3d = new Vector3d(8, 0, 8);

    private RangeSensorBelt sonars, bumpers;
    private LampActuator lamp;
    private Vector3d origin = null;

    public void initBehavior() {
    }

    public AttractForceRobot(Vector3d position, String name) {
        super(position, name);

        bumpers = RobotFactory.addBumperBeltSensor(this);//the bumper sensor
        sonars = RobotFactory.addSonarBeltSensor(this);//the sonar sensor
        lamp = RobotFactory.addLamp(this);//the instruction lump
        origin = position;// the origin position

    }

    public Vector3d getVelocity() {
        return this.linearVelocity; //linear velocity
    }

    private int getQuadrant(Vector2d vector) //cal the quadrant of the agent
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

    private double getAngle(Vector2d v1, Vector2d v2) //cal rad of two vectors
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
                        if (debug)
                            System.out.println("NO");
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
                        if (debug)
                            System.out.println("here");
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

    private Vector2d transform(Vector2d v, Vector2d point) {
        Vector2d global = new Vector2d(1, 0); //（1,0）,means the x-axis
        double alfa = getAngle(global, v); //the rad of v with x
        double beta = getAngle(point, v); //the rad of point with v

        double k1 = Math.cos(alfa + beta) / Math.cos(beta);
        double k2 = Math.sin(alfa + beta) / Math.sin(beta);

        double x = point.x * k1;
        double y = point.y * k2;

        return new Vector2d(x, y);

    }

    private double repelForce(double distance, double range) //计算斥力
    {
        double force = 0;
        Point3d p = new Point3d();
        getCoords(p); //获取当前坐标
        Vector2d pos = new Vector2d(p.z, p.x); //计算当前向量
        Vector2d toGoal = new Vector2d((goal.x - pos.x), (goal.y - pos.y)); //当前指向目标的向量
        double disGoal = toGoal.length();
        double n = 0.5;
        if (distance <= range) //距离小于range则没有斥力
        {
            force = (1 / distance - 1 / range) * (1 / distance - 1 / range) * repelConstant;//计算斥力
        }

        return force;
    }

    private double attractForce(double distance) //计算吸引力
    {
        double force = attractConstant * distance;
        return force;
    }

    private boolean checkGoal() //检查是否到达目的地
    {

        Point3d currentPos = new Point3d();
        getCoords(currentPos); //当前坐标
        Point3d goalPos = new Point3d(goal3d.x, goal3d.y, goal3d.z);

        if (currentPos.distance(goalPos) <= 0.5) // 如果当前距离目标点小于0.5那么即认为是到达
        {
            return true;
        } else {
            return false;
        }
    }

    public void performBehavior() {
        // 为了防止智能体剧烈晃动，每10帧计算一次受力
        if (getCounter() % 10 == 0) {

            Vector3d velocity = getVelocity(); //获取速度


            Vector2d direct = new Vector2d(velocity.z, velocity.x); //前进的方向向量


            Point3d p = new Point3d();
            getCoords(p);
            Vector2d pos = new Vector2d(p.z, p.x);


            double d0 = sonars.getMeasurement(0);// front声纳，正前方障碍物距离
            double d1 = sonars.getMeasurement(1);// frontleft声纳，左前方障碍物距离
            double d2 = sonars.getMeasurement(8);// frontright声纳，右前方障碍物距离


            double rf0 = repelForce(d0, 4.0); //三个方向的斥力
            double rf1 = repelForce(d1, 4.0);
            double rf2 = repelForce(d2, 4.0);
            System.out.println("d0=" + d0 + "    D1=" + d1 + "    d2=" + d2);

            // 计算斥力的合力
            double k1 = Math.cos(2 * Math.PI / 9);
            double k2 = Math.sin(2 * Math.PI / 9);
            Vector2d vf0 = new Vector2d(0 - rf0, 0);
            Vector2d vf1 = new Vector2d((0 - rf1 * k1), (0 - rf1 * k2));
            Vector2d vf2 = new Vector2d((rf2 * k1), (rf2 * k2));
            Vector2d composition = new Vector2d(vf0.x + vf1.x + vf2.x, vf0.y + vf1.y + vf2.y);
            if (debug) System.out.println("(" + composition.x + "," + composition.y);
            Vector2d repelForceVector = transform(direct, composition);
            Vector2d toGoal = new Vector2d((goal.x - pos.x), (goal.y - pos.y));
            double disGoal = toGoal.length();
            if (debug) System.out.println("distance to goal:" + disGoal);
            double goalForce = attractForce(disGoal);//利用目标的吸引力来引导
            if (debug) System.out.println("attract force from goal:" + goalForce);
            Vector2d goalForceVector = new Vector2d((goalForce * toGoal.x / disGoal), (goalForce * toGoal.y / disGoal));
            Vector2d originForceVector = new Vector2d(origin.x, origin.z);
            double x = repelForceVector.x + goalForceVector.x;
            double y = repelForceVector.y + goalForceVector.y;
            Vector2d allForces = new Vector2d(x, y);//合力
            if (debug) {
                System.out.println("total force(" + allForces.x + "," + allForces.y + ")");
                System.out.println("force direct(" + direct.x + "," + direct.y + ")");
            }
            double angle = getAngle(direct, allForces);
            if (debug) System.out.println("angle:" + angle);
            // 判断转动方向
            if (angle < Math.PI) {
                setRotationalVelocity(angle);
            } else if (angle > Math.PI) {
                setRotationalVelocity((angle - 2 * Math.PI));
            }

            if (checkGoal()) {
                // 到达目标点，停止运动
                setTranslationalVelocity(0);
                setRotationalVelocity(0);
                lamp.setOn(true);
                return;
            } else {
                lamp.setOn(false);
                setTranslationalVelocity(0.5);
            }
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
}