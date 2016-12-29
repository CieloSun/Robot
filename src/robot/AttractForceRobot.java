package robot;

import simbad.sim.RobotFactory;

import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

/**
 * Created by 63289 on 2016/12/28.
 * 使用人工势场法进行运动的agent
 */
public class AttractForceRobot extends RobotBase {
    private static final double repelConstant = 100.0;// 斥力系数
    private static final double attractConstant = 30.0;// 引力系数

    public AttractForceRobot(Vector3d origin, Vector3d goal3d, String name) {
        super(origin, goal3d, name);
    }

    protected double repelForce(double distance, double range) //计算斥力
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

    protected double attractForce(double distance) //计算吸引力
    {
        double force = attractConstant * distance;
        return force;
    }

    public void performBehavior() {
        if (getCounter() % 5 == 0) {
            checkGoal();
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
            Vector2d composition = new Vector2d(vf0.x + vf1.x + vf2.x, vf0.y + vf1.y + vf2.y);//合力
            Vector2d repelForceVector = transform(direct, composition);
            Vector2d toGoal = new Vector2d((goal.x - pos.x), (goal.y - pos.y));
            double disGoal = toGoal.length();
            double goalForce = attractForce(disGoal);//利用目标的吸引力来引导
            Vector2d goalForceVector = new Vector2d((goalForce * toGoal.x / disGoal), (goalForce * toGoal.y / disGoal));
            double x = repelForceVector.x + goalForceVector.x;
            double y = repelForceVector.y + goalForceVector.y;
            Vector2d allForces = new Vector2d(x, y);//合力
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
}
