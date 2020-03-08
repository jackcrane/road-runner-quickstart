package teamcode.TestFiles.AccelIntTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import teamcode.CompOpModes.TeleopAndRobotFiles.TauBot;

public abstract class tauAutoMode extends LinearOpMode {
    TauBot robot  = new TauBot();

    public void turnDegrees(double degrees, double maxPower, double minPower){
        double power;
        double threshold = 5;
        while (Math.abs(myMod(degrees, 360) - myMod(robot.imu.getAngularOrientation().firstAngle, 360)) > threshold && opModeIsActive()){
            double diff = 1 - Math.abs(robot.imu.getAngularOrientation().firstAngle / degrees);
            power = maxPower * diff;
            power = Range.clip(power, minPower, maxPower);

            double mySign = Math.signum(degrees - robot.imu.getAngularOrientation().firstAngle);

            robot.leftFront.setPower(-power * mySign);
            robot.leftBack.setPower(-power * mySign);
            robot.rightFront.setPower(power * mySign);
            robot.rightBack.setPower(power * mySign);
            idle();
        }
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);
    }
    public double myMod(double a, double b) {
        return (a % b + b) % b;
    }
    public void driveMillImu(double MM){
        robot.imu.startAccelerationIntegration(new Position(),new Velocity(),5);

        while (robot.imu.getPosition().x * 1000 < MM) {
            robot.leftFront.setPower(.1);
            robot.rightBack.setPower(.1);
            robot.leftBack.setPower(.1);
            robot.rightFront.setPower(.1);

            telemetry.addData("x:", robot.imu.getPosition().x);
            telemetry.addData("y:", robot.imu.getPosition().y);
            telemetry.addData("z:", robot.imu.getPosition().z);
            telemetry.update();
            idle();
        }
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);

        robot.imu.stopAccelerationIntegration();

    }

}
