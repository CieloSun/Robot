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
public class VFHRobot extends RobotBase {
    //置信数
    private Long[][] cv = new Long[21][21];
    //存储向量
    private Double[][] m = new Double[21][21];

    public void initBehavior() {
        setTranslationalVelocity(speed);
    }

    public VFHRobot(Vector3d origin, Vector3d goal3d, String name) {
        super(origin, goal3d, name);
        for (int i = 0; i < 21; ++i) {
            for (int j = 0; j < 21; ++j) {
                cv[i][j] = new Long(0);
                m[i][j] = new Double(0.0);
            }
        }
        speed = 0.5;
    }

    private void setCv(int x, int y, Long val) {
        for (int i = 0; i < 21; ++i) {
            for (int j = 0; j < 21; ++j) {
                System.out.print(getCv(x, y) + " ");
            }
            System.out.println();
        }
        System.out.println(val);
        cv[(int) x + 10][y + 10] = val;
    }

    private final Long getCv(int x, int y) {
        return cv[x + 10][y + 10];
    }

    private void setM(int x, int y, Double dis) {
        m[x + 10][y + 10] = -getCv(x, y) * getCv(x, y) / dis * 0.1;
    }

    private final Double getM(int x, int y) {
        return m[x + 10][y + 10];
    }

    private void calEnvironment(Vector2d now) {
        for (int i = 0; i < 9; ++i) {
            Double dis = new Double(sonars.getMeasurement(i));
            Double deltaX = dis * (Math.cos(i * 40));
            Double deltaY = dis * (Math.sin(i * 40));
            long lnx = Math.round(now.getX() + deltaX);//标记此点的障碍权重
            long lny = Math.round(now.getY() + deltaY);
            int nx = (int) lnx;
            int ny = (int) lny;
            if (nx <= 10 && nx >= -10 && ny <= 10 && ny >= -10) {
                System.out.println("nx:" + nx + " ny:" + ny);
                setCv(nx, ny, new Long(1) + getCv(nx, ny));
                setM(nx, ny, dis);
            }
        }
    }


    public void performBehavior() {
        checkGoal();
        Vector3d velocity = getVelocity(); //获取速度
        Vector2d direct = new Vector2d(velocity.z, velocity.x); //前进的方向向量
        Point3d p = new Point3d();
        getCoords(p);
        Vector2d pos = new Vector2d(p.z, p.x);
        calEnvironment(pos);
        double xForce = 0;
        double yForce = 0;
        for (int i = -10; i <= 10; ++i) {
            for (int j = -10; j <= 10; ++j) {
                xForce += getM(i, j) * (i - pos.getX()) / Math.sqrt((i - pos.getX()) * (i - pos.getX()) + (j - pos.getY()) * (j - pos.getY()));
                yForce += getM(i, j) * (j - pos.getY()) / Math.sqrt((i - pos.getX()) * (i - pos.getX()) + (j - pos.getY()) * (j - pos.getY()));
            }
        }
        Vector2d composition = new Vector2d(xForce + goal.getX() - pos.getX(), yForce + goal.getY() - pos.getY());//合力
        Vector2d forceVector = transform(direct, composition);
        double angle = getAngle(direct, forceVector);
        // 判断转动方向
        if (angle < Math.PI) {
            setRotationalVelocity(angle);
        } else if (angle > Math.PI) {
            setRotationalVelocity((angle - 2 * Math.PI));
        }
        checkHit();
    }
}