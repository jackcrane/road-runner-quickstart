package teamcode.roadrunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.CompOpModes.TeleopAndRobotFiles.TauBot;
import teamcode.roadrunner.hardware.DashboardUtil;
import teamcode.roadrunner.hardware.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    double imuHeadingAccumulator = 0;
    double imuLastHeading = 0;

    public static double LOCALIZE_TURN = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        TauBot robot = new TauBot();

        robot.initAuto(hardwareMap);

//        drive.turnAsync(Math.toRadians(LOCALIZE_TURN));

        waitForStart();

        while (!isStopRequested()) {
//            Pose2d baseVel = new Pose2d(
//                    -gamepad1.left_stick_y,
//                    -gamepad1.left_stick_x,
//                    -gamepad1.right_stick_x
//            );
//
//            Pose2d vel;
//            if (Math.abs(baseVel.getX()) + Math.abs(baseVel.getY()) + Math.abs(baseVel.getHeading()) > 1) {
//                 re-normalize the powers according to the weights
//                double denom = VX_WEIGHT * Math.abs(baseVel.getX())
//                        + VY_WEIGHT * Math.abs(baseVel.getY())
//                        + OMEGA_WEIGHT * Math.abs(baseVel.getHeading());
//                vel = new Pose2d(
//                        VX_WEIGHT * baseVel.getX(),
//                        VY_WEIGHT * baseVel.getY(),
//                        OMEGA_WEIGHT * baseVel.getHeading()
//                ).div(denom);
//            } else {
//                vel = baseVel;
//            }
//
//            drive.setDrivePower(vel);

            double horizontal = gamepad1.left_stick_x;
            double forwards = (-gamepad1.left_stick_y);
            double turning = gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x);

            double leftFront = forwards + horizontal + turning;
            double leftBack = forwards - horizontal + turning;
            double rightFront = forwards - horizontal - turning;
            double rightBack = forwards + horizontal - turning;

            double[] wheelPowers = {Math.abs(rightFront), Math.abs(leftFront), Math.abs(leftBack), Math.abs(rightBack)};
            double biggestInput = highestValue(wheelPowers);

            leftFront /= biggestInput;
            leftBack /= biggestInput;
            rightFront /= biggestInput;
            rightBack /= biggestInput;

            drive.setMotorPowers(
                    leftFront / 2, leftBack / 2, rightBack / 2, rightFront / 2
            );

            drive.update();

//            Pose2d poseEstimate = drive.getPoseEstimate();
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
//            telemetry.update();
//
            TelemetryPacket packet = new TelemetryPacket();

            Pose2d estimate = drive.getPoseEstimate();
            packet.put("x", estimate.getX());
            packet.put("y", estimate.getY());
            packet.put("heading", Math.toDegrees(estimate.getHeading()));

            double imuHeading = drive.getRawExternalHeading();
            imuHeadingAccumulator += Angle.norm(imuHeading - imuLastHeading);
            imuLastHeading = imuHeading;

            packet.put("rawheading", Math.toDegrees(imuHeadingAccumulator) % 360);

            Canvas fieldOverlay = packet.fieldOverlay();
            fieldOverlay.setStroke("#F44336");
            DashboardUtil.drawRobot(fieldOverlay,
                    new Pose2d(estimate.getX(), estimate.getY(), estimate.getHeading()));

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    public double highestValue(double[] array) {
        double highestVal = array[0];
        for (int i = 1; i < array.length; i++) {
            if (array[i] > highestVal) {
                highestVal = array[i];
            }
        }
        return highestVal;
    }
}