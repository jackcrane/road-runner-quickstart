package teamcode.TestFiles.AccelIntTesting;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.NavUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class TauAccelerationIntegrator implements BNO055IMU.AccelerationIntegrator{
    BNO055IMU.Parameters parameters;
    Position position;
    Velocity velocity;
    Acceleration acceleration;
    Acceleration[] buffer = new Acceleration[10];
    Acceleration avgBuffer;

    TauAccelerationIntegrator(){
        this.parameters = null;
        this.position = new Position();
        this.velocity = new Velocity();
        this.acceleration = null;
        for (int i = 0; i < 10; i++){
            buffer[i] = new Acceleration();
        }
    }

    @Override public void initialize(BNO055IMU.Parameters parameters, Position initialPosition, Velocity initialVelocity)
    {
        this.parameters = parameters;
        this.position = initialPosition != null ? initialPosition : this.position;
        this.velocity = initialVelocity != null ? initialVelocity : this.velocity;
        this.acceleration = null;
    }

    public Acceleration avgAcceleration() {
        return this.avgBuffer;
    }

    @Override
    public Position getPosition() {
        return this.position;
    }

    @Override
    public Velocity getVelocity() {
        return this.velocity;
    }

    @Override
    public Acceleration getAcceleration() {
        return this.acceleration;
    }

    @Override public void update(Acceleration linearAcceleration)
    {
//        //reducing noise
//        for(int i = 0; i < 9; i++){
//            buffer[i] = buffer[i++];
//        }
//        buffer[9] = linearAcceleration;
//
//        avgBuffer = new Acceleration();
//        for (int j = 0; j < 10; j++){
//            avgBuffer = addAcceleration(avgBuffer, buffer[j]);
//        }
//
//        avgBuffer = zeroAcceleration(new Acceleration(avgBuffer.unit, avgBuffer.xAccel/10, avgBuffer.yAccel/10, avgBuffer.zAccel/10, avgBuffer.acquisitionTime));
        avgBuffer = zeroAcceleration(linearAcceleration);

        // We should always be given a timestamp here
        if (avgBuffer.acquisitionTime != 0)
        {
            // We can only integrate if we have a previous acceleration to baseline from
            if (acceleration != null) {

                Acceleration accelPrev    = acceleration;
                Velocity velocityPrev = velocity;

                acceleration = avgBuffer;

                if (accelPrev.acquisitionTime != 0)
                {
                    Velocity deltaVelocity = NavUtil.meanIntegrate(acceleration, accelPrev);
                    velocity = NavUtil.plus(velocity, deltaVelocity);
                }

                if (velocityPrev.acquisitionTime != 0)
                {
                    Position deltaPosition = NavUtil.meanIntegrate(velocity, velocityPrev);
                    position = NavUtil.plus(position, deltaPosition);
                }

                if (parameters != null && parameters.loggingEnabled)
                {
                    RobotLog.vv(parameters.loggingTag, "dt=%.3fs accel=%s vel=%s pos=%s", (acceleration.acquisitionTime - accelPrev.acquisitionTime)*1e-9, acceleration, velocity, position);
                }
            }
            else {
                acceleration = avgBuffer;
            }
        }
    }

    public Acceleration addAcceleration(Acceleration a, Acceleration b){
        Acceleration result = new Acceleration(a.unit, a.xAccel + b.xAccel, a.yAccel + b.yAccel, a.zAccel + b.zAccel, b.acquisitionTime);
        return result;
    }

    public Acceleration zeroAcceleration(Acceleration current){
        double x = current.xAccel;
        double y = current.yAccel;
        double z = current.zAccel;

        if (Math.abs(current.xAccel) < .1){
            x = 0;
        }

        if (Math.abs(current.yAccel) < .1){
            y = 0;
        }

        if (Math.abs(current.zAccel) < .1){
            z = 0;
        }

        return new Acceleration(current.unit, x, y, z, current.acquisitionTime);

    }

}



