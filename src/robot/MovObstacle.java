package robot;

import simbad.sim.Agent;
import simbad.sim.LampActuator;
import simbad.sim.RangeSensorBelt;
import simbad.sim.RobotFactory;

import javax.vecmath.Vector3d;

/**
 * Created by 63289 on 2016/12/28.
 */
public class MovObstacle extends Agent {

    RangeSensorBelt sonars,bumpers;
    LampActuator lamp;
    double speed = 0.5;

    public MovObstacle(Vector3d position, String name) {
        super(position,name);
        bumpers = RobotFactory.addBumperBeltSensor(this);
        sonars = RobotFactory.addSonarBeltSensor(this,24);
        lamp = RobotFactory.addLamp(this);
    }

    public void initBehavior() {
        setTranslationalVelocity(speed);
    }

    public void performBehavior() {

        if (bumpers.oneHasHit()) {
            lamp.setBlink(true);
        }else
            lamp.setBlink(false);

        if(getCounter()%100 == 0){
            speed = -speed;
            setTranslationalVelocity(speed);
        }
    }
}