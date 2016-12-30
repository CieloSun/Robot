package robot;

/**
 * Created by 63289 on 2016/12/30.
 */

import simbad.sim.Agent;
import simbad.sim.LightSensor;
import simbad.sim.RobotFactory;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

public class LightRobot extends RobotBase {
    //定义物体的最高速度
    private static final double MAX_VELOCITY = 0.4;
    //定义物体的最高角速度
    private static final double MAX_ANGULAR_VELOCITY = Math.PI;
    //定义结构体存储相关变量
    private class Sensor {
        public double angle;
        public double measurement;
        public int number;

        public Sensor() {
            this(0, 0, 0);
        }

        public Sensor(double angle, double measurement, int number) {
            this.angle = angle;
            this.measurement = measurement;
            this.number = number;
        }
    }
    //最小角度
    private Sensor minAngle;
    private LightSensor sensorFrontLeft;
    private LightSensor sensorFrontRight;
    private LightSensor sensorRearLeft;
    private LightSensor sensorRearRight;
    public LightRobot(Vector3d position,Vector3d goal3d, String name) {
        super(position,goal3d, name);
        // 安装传感器，包括24个声纳和4个光敏传感器，振动传感器
        sonars = RobotFactory.addSonarBeltSensor(this, 24);
        sensorFrontLeft = RobotFactory.addLightSensorLeft(this);
        sensorFrontRight = RobotFactory.addLightSensorRight(this);
        Vector3d front = new Vector3d(getRadius() + 0.5, 0, 0);
        sensorRearLeft = addLightSensorRearLeft(this);
        sensorRearRight = addLightSensorRearRight(this);
    }
    static public LightSensor addLightSensorRearRight(Agent agent) {
        Vector3d front = new Vector3d(agent.getRadius() + 0.5, 0, 0);
        Transform3D t3d = new Transform3D();
        t3d.rotY((-Math.PI / 4) * 3);
        Vector3d right = new Vector3d(front);
        t3d.transform(right);
        return RobotFactory.addLightSensor(agent, right, (float) (-Math.PI / 4) * 3,"rear_right");
    }
    static public LightSensor addLightSensorRearLeft(Agent agent) {
        Vector3d front = new Vector3d(agent.getRadius() + 0.5, 0, 0);
        Transform3D t3d = new Transform3D();
        t3d.rotY((Math.PI / 4) * 3);
        Vector3d left = new Vector3d(front);
        t3d.transform(left);
        return RobotFactory.addLightSensor(agent, left, (float) (Math.PI / 4) * 3,"rear_left");
    }
    public void performBehavior() {
        checkGoal();
        //
        Sensor minPositiveAngle = new Sensor();
        Sensor minNegativeAngle = new Sensor();
        //卡死
        if (collisionDetected()) moveToStartPosition();
        //震动测撞击
        if (bumpers.oneHasHit()) {
            setTranslationalVelocity(-0.1);
            setRotationalVelocity(0.1 * Math.random());
        } else {
            for (int i = 0; i < sonars.getNumSensors(); i++) {
                //声纳检测撞击，判断障碍
                if (sonars.hasHit(i)) {
                    if (i <= minPositiveAngle.number + 2) {
                        minPositiveAngle.angle = sonars.getSensorAngle(i) + (2 * Math.PI / sonars.getNumSensors());
                        minPositiveAngle.measurement = sonars.getMeasurement(i);
                    }
                    if ((i - sonars.getNumSensors()) % sonars.getNumSensors() >= minNegativeAngle.number - 2) {
                        minNegativeAngle.angle = (sonars.getSensorAngle(i) - 2 * Math.PI - (2 * Math.PI / sonars.getNumSensors())) % (2 * Math.PI);
                        minNegativeAngle.measurement = sonars.getMeasurement(i);
                    }
                }
            }
            if (Math.abs(minNegativeAngle.angle) >= Math.abs(minPositiveAngle.angle)) {
                minAngle = minNegativeAngle;
            } else {
                minAngle = minPositiveAngle;
            }
            double nextVel = MAX_VELOCITY;
            double nextAngVel = 0;

            //if (minAngle.angle != 0) {
            double bestState = 0;
            for (double velocity = 0.001; velocity < MAX_VELOCITY; velocity += MAX_VELOCITY / 10) {
                for (double angularVelocity = -MAX_ANGULAR_VELOCITY; angularVelocity < MAX_ANGULAR_VELOCITY; angularVelocity += MAX_ANGULAR_VELOCITY / 10) {
                    double heuristic = actionFunction(velocity, angularVelocity, 0.1, 5, 30);
                    if (heuristic > bestState) {
                        bestState = heuristic;
                        nextVel = velocity;
                        nextAngVel = angularVelocity;
                    }
                }
            }
            setRotationalVelocity(nextAngVel);
            setTranslationalVelocity(nextVel);
        }
    }
    private double actionFunction(double velocity, double angularVelocity, double velocityWeight, double obstacleWeight, double aimWeight) {
        return velocityFactor(velocity, angularVelocity) * velocityWeight
                + obstacleFactor(velocity, angularVelocity) * obstacleWeight
                + lightFactor(velocity, angularVelocity) * aimWeight;
    }
    private double lightFactor(double velocity, double angularVelocity) {
        //获取传感器检测值
        double leftLum = sensorFrontLeft.getAverageLuminance();
        double rightLum = sensorFrontRight.getAverageLuminance();
        double rearLeftLum = sensorRearLeft.getAverageLuminance();
        double rearRightLum = sensorRearRight.getAverageLuminance();
        if (Math.abs(rearRightLum - rightLum) + Math.abs(rearRightLum - leftLum) + Math.abs(rearRightLum - rearLeftLum) < 0.1) {
            //检测到终点
            setTranslationalVelocity(0);
            setRotationalVelocity(0);
            lamp.setOn(true);
        }
        //目标角速度
        double desiredRotationalVelocity;
        //判断光源方向
        if ((leftLum > rearLeftLum && leftLum > rearRightLum && rightLum > rearRightLum && rightLum > rearLeftLum)) { //前
            desiredRotationalVelocity = (leftLum - rightLum) * Math.PI / 4;
        } else if (leftLum > rightLum && leftLum > rearRightLum && rearLeftLum > rightLum && rearLeftLum > rearRightLum) { //左
            desiredRotationalVelocity = (rearLeftLum - leftLum) * Math.PI / 4 + Math.PI / 2;
        } else if (rightLum > leftLum && rightLum > rearLeftLum && rearRightLum > leftLum && rearRightLum > rearLeftLum) { //右
            desiredRotationalVelocity = (rightLum - rearRightLum) * Math.PI / 4 - Math.PI / 2;
        } else if (rearRightLum > rightLum && rearRightLum > leftLum && rearLeftLum > leftLum && rearLeftLum > rightLum) { //后
            //判断调头方向
            desiredRotationalVelocity = (rearLeftLum - rearRightLum) * Math.PI / 4;
            if (rearRightLum > rearLeftLum) {
                desiredRotationalVelocity = Math.PI - desiredRotationalVelocity;
            } else {
                desiredRotationalVelocity = -Math.PI - desiredRotationalVelocity;
            }
        } else {
            //无法判断光源方向，则不转动
            desiredRotationalVelocity = 0;
        }
        //利用常数微调
        desiredRotationalVelocity /= 1.5;
        //常数
        double result;
        if (angularVelocity == 0) {
            result = 1 - Math.abs(desiredRotationalVelocity / Math.PI);
        }
        else if (desiredRotationalVelocity == 0) {
            result = 1 - Math.abs(angularVelocity / Math.PI);
        }
        else if (Math.abs(angularVelocity) > Math.abs(desiredRotationalVelocity)) {
            result = desiredRotationalVelocity / angularVelocity;
        }
        else {
            result = angularVelocity / desiredRotationalVelocity;
        }
        return (result > 0 ? result : 0);
    }
    private double obstacleFactor(double velocity, double angularVelocity) {
        //没有障碍
        if (minAngle.angle == 0) {
            return Math.floor(2 - 2 * Math.abs(angularVelocity / Math.PI));
            //判断符号是否相同，若不同
        }
        else if (Math.signum(minAngle.angle) != Math.signum(angularVelocity)) {
            angularVelocity = Math.signum(minAngle.angle) * angularVelocity;
            double angle = Math.abs(Math.sin(minAngle.angle) / minAngle.measurement);
            if (2 * minAngle.angle / angle > angularVelocity / velocity) {
                return 1 - (angularVelocity / velocity) / (2 * minAngle.angle / angle);
            } else {
                return 0;
            }
        } else {
            return 0;
        }
    }
    private double velocityFactor(double velocity, double angularVelocity) {
        return velocity / MAX_VELOCITY;
    }
}

