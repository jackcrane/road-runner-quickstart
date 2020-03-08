//package teamcode.CompOpModes;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import teamcode.CompOpModes.TeleopAndRobotFiles.TauBot;
//import teamcode.Vision.camera;
//
//@Autonomous(name = "VisionTestAutoMode")
//
//
//
//public class VisionTestAutoMode extends LinearOpMode {
//
//    private SampleMecanumDriveREVOptimized r = new SampleMecanumDriveREVOptimized(hardwareMap);
//    private TauBot robot = new TauBot();
//    private boolean allianceColorisRed = true;
//    private int skyStonePos;
//
//
//    public void runOpMode() throws InterruptedException {
//
//        robot.initAuto(hardwareMap);
//        camera camera = new camera(this);
//        skyStonePos = camera.getSkyPos(allianceColorisRed);
//        telemetry.addData("SkyStone Pos:", skyStonePos);
//        telemetry.update();
//        sleep(10000);
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//
//    }
//
//
//}
