package robot;
import simbad.sim.RangeSensorBelt;
import simbad.sim.RobotFactory;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;
/**
 * Created by 63289 on 2016/12/28.
 */
public class ForceRobot extends RobotBase {
    private static final double repelConstant = 100.0;// 斥力系数
    private static final double rangeConst = 4.0;//斥力作用范围
    private double attractConstant = 30.0;// 引力系数
    private RangeSensorBelt sonars;//9向声纳
    private double speed = 0.5;//速度设定
    public ForceRobot(Vector3d origin, Vector3d goal3d, String name) {
        super(origin, goal3d, name);
        sonars = RobotFactory.addSonarBeltSensor(this);//the sonar sensor
        speed = 1;
    }
    public void initBehavior() {
        setTranslationalVelocity(speed);
    }
    public void performBehavior() {
        if (getCounter() % 10 == 0) {
            Vector3d velocity = linearVelocity;
            Vector2d velocity2d = new Vector2d(velocity.z, velocity.x);
            Point3d posPoint = new Point3d();
            //获取当前位置
            getCoords(posPoint);
            Vector2d pos = new Vector2d(posPoint.z, posPoint.x);
            //测量三个方向的距离
            double frontDistance = sonars.getMeasurement(0);
            double frontLeftDistance = sonars.getMeasurement(1);
            double frontRightDistance = sonars.getMeasurement(8);
            //计算三个方向的排斥力
            double frontRepelForce = getRepelForce(frontDistance, rangeConst);
            double frontLeftRepelForce = getRepelForce(frontLeftDistance, rangeConst);
            double frontRightRepelForce = getRepelForce(frontRightDistance, rangeConst);
            double cosConst = Math.cos(2 * Math.PI / 9);
            double sinConst = Math.sin(2 * Math.PI / 9);
            Vector2d fontRepelVector = new Vector2d(0 - frontRepelForce, 0);
            Vector2d fontLeftRepelVector = new Vector2d((0 - frontLeftRepelForce * cosConst), (0 - frontLeftRepelForce * sinConst));
            Vector2d frontRightRepelVector = new Vector2d((frontRightRepelForce * cosConst), (frontRightRepelForce * sinConst));
            //计算排斥力的合力
            Vector2d repelForce = new Vector2d(fontRepelVector.x + fontLeftRepelVector.x + frontRightRepelVector.x,
                    fontRepelVector.y + fontLeftRepelVector.y + frontRightRepelVector.y);
            Vector2d repelForceVector = forceResolve(velocity2d, repelForce);
            Vector2d toGoal = new Vector2d((goal.x - pos.x), (goal.y - pos.y));
            double disToGoal = toGoal.length();
            double goalForce = getAttractForce(disToGoal);
            Vector2d goalForceVector = new Vector2d((goalForce * toGoal.x / disToGoal), (goalForce * toGoal.y / disToGoal));
            Vector2d allForces = new Vector2d(repelForceVector.x + goalForceVector.x, repelForceVector.y + goalForceVector.y);
            double angle = getAngle(velocity2d, allForces);
            if (angle < Math.PI) {
                setRotationalVelocity(angle);
            } else if (angle > Math.PI) {
                setRotationalVelocity((angle - 2 * Math.PI));
            }
            if (checkGoal()) {
                setTranslationalVelocity(0);
                setRotationalVelocity(0);
                lamp.setOn(true);
                return;
            } else {
                lamp.setOn(false);
                setTranslationalVelocity(speed);
            }
            checkHit();
        }
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
    private Vector2d forceResolve(Vector2d velocity, Vector2d force) {
        Vector2d xAxis = new Vector2d(1, 0);
        double alfa = getAngle(xAxis, velocity);
        double beta = getAngle(force, velocity);
        return new Vector2d( force.x * Math.cos(alfa + beta) / Math.cos(beta), force.y * Math.sin(alfa + beta) / Math.sin(beta));
    }
    private boolean checkHit() {
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
    private double getAttractForce(double distance) {
        return attractConstant * distance;
    }
    //计算斥力
    private double getRepelForce(double distance, double range) {
        return distance <= range ? (1 / distance - 1 / range) * (1 / distance - 1 / range) * repelConstant : 0;
    }
}
