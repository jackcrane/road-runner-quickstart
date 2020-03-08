package teamcode.CompOpModes.TeleopAndRobotFiles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="RunToPositionTest", group="tauBot")
public class RunToPositionTest extends LinearOpMode {
    @Override
    public void runOpMode() {


        Servo blockServo = null;
        DcMotor testMotor = null;
        DcMotor newMotor = null;
        testMotor = hardwareMap.get(DcMotor.class, "leftLift");
        newMotor = hardwareMap.get(DcMotor.class, "rightLift");
        blockServo = hardwareMap.get(Servo.class, "blockServo");
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setPower(0);
        newMotor.setPower(0);
        newMotor.setDirection(DcMotor.Direction.REVERSE);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

            blockServo.setPosition(.75);

            if (gamepad1.b) {
                testMotor.setTargetPosition(-60);
                newMotor.setTargetPosition(-60);

                testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                newMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                testMotor.setPower(.15);
                newMotor.setPower(.15);
            } else if (gamepad1.x) {
                testMotor.setTargetPosition(0);
                newMotor.setTargetPosition(0);


                testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                newMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                testMotor.setPower(.15);
                newMotor.setPower(.15);
            }


            telemetry.addData("left pos:", testMotor.getCurrentPosition());
            telemetry.addData("new pos:", newMotor.getCurrentPosition());

            telemetry.addData("Run Mode", testMotor.getMode());
            telemetry.addData("Run Mode", newMotor.getMode());
            telemetry.addData("Targert Pos: ", testMotor.getTargetPosition());
            telemetry.addData("Targert Pos: ", newMotor.getTargetPosition());

            telemetry.addData("Right Lift Motor Power", testMotor.getPower());
            telemetry.addData("Right Lift Motor Power", newMotor.getPower());

            telemetry.addData("leftBusy",testMotor.isBusy());
            telemetry.addData("right busy", newMotor.isBusy());

            telemetry.update();
        }
        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        testMotor.setPower(0);
    }
}
