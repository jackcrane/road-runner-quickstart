package teamcode.TestFiles.AccelIntTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Loading Auto", group="tauBot")
public class BlueLoadingAuto extends tauAutoMode {

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        telemetry.addData("Ready To Go: ", true);
        telemetry.update();

        waitForStart();

        if (!isStopRequested()) {
            Autonomous();
        }
    }

    private void Autonomous() throws InterruptedException {

    }
}
