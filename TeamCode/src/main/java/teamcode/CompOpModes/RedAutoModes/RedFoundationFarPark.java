//package teamcode.CompOpModes.RedAutoModes;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import teamcode.CompOpModes.TeleopAndRobotFiles.TauBot;
//
//@Autonomous(name = "RedFoundationFarPark")
//
//public class RedFoundationFarPark extends LinearOpMode {
//
//    private SampleMecanumDriveREVOptimized r;
//    private TauBot robot;
//    double   rightHookUp    = 0.37;
//    double   rightHookDown  = 0;
//    double   leftHookUp     = 0.34;
//    double   leftHookDown   = 0.69;
//
//
//    public void runOpMode() throws InterruptedException {
//        r = new SampleMecanumDriveREVOptimized(hardwareMap);
//        robot = new TauBot();
//
//        Trajectory trajectory = r.trajectoryBuilder().forward(35).build();
//        robot.initAuto(hardwareMap);
//        waitForStart();
//
//
//        if (isStopRequested()) return;
//
//        hooksDown();
//        r.followTrajectorySync(trajectory);
//        r.setPoseEstimate(new Pose2d());
//       // r.followTrajectorySync(r.trajectoryBuilder().reverse().splineTo(new Pose2d(-20,-10,Math.toRadians(90))).build());
//        hooksUp();
//        //sleep(2000);
//        r.followTrajectorySync(r.trajectoryBuilder().splineTo(new Pose2d(-20,8,Math.toRadians(90))).build());
//       // r.followTrajectorySync(r.trajectoryBuilder().reverse().splineTo(new Pose2d(-4,-40,Math.toRadians(90))).build());
//
//    }
//
//    public void hooksDown(){
//        robot.leftHook.setPosition(leftHookDown);
//        robot.rightHook.setPosition(rightHookDown);
//    }
//
//    public void hooksUp(){
//        robot.leftHook.setPosition(leftHookUp);
//        robot.rightHook.setPosition(rightHookUp);
//    }
//}
