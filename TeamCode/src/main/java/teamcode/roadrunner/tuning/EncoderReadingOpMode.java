package teamcode.roadrunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import teamcode.CompOpModes.TeleopAndRobotFiles.TauBot;

@Autonomous(name="Encoder Reading OpMode")
public class EncoderReadingOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TauBot robot = new TauBot();

        FtcDashboard dashboard = FtcDashboard.getInstance();

        robot.init(hardwareMap);

        waitForStart();

        while(!isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();

            packet.put("leftBackWheelEncoder", robot.leftBack.getCurrentPosition());
            packet.put("leftFrontWheelEncoder", robot.leftFront.getCurrentPosition());
            packet.put("rightBackWheelEncoder", robot.rightBack.getCurrentPosition());
            packet.put("rightFrontWheelEncoder", robot.rightFront.getCurrentPosition());
            packet.put("rightDeadWheel", robot.rightIntake.getCurrentPosition());
            packet.put("leftDeadWheel", robot.leftIntake.getCurrentPosition());
            packet.put("middleDeadWheel", robot.leftLift.getCurrentPosition());
            packet.put("liftEncoder", robot.rightLift.getCurrentPosition());

            dashboard.sendTelemetryPacket(packet);
        }
    }
}
