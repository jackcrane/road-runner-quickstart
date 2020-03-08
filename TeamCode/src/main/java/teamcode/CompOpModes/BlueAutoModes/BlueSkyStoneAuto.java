//package teamcode.CompOpModes.BlueAutoModes;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import teamcode.CompOpModes.TeleopAndRobotFiles.TauBot;
//
//
//@Autonomous(name = "BlueSkyStoneAuto")
//
//public class BlueSkyStoneAuto extends LinearOpMode {
//
//    private SampleMecanumDriveREVOptimized r;
//    private TauBot robot;
//    double   rightHookUp    = 0.37;
//    double   rightHookDown  = 0;
//    double   leftHookUp     = 0.34;
//    double   leftHookDown   = 0.69;
//    boolean  colelctJam     = false;
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
//        intake("in", 0.65);
//       // r.followTrajectorySync(r.trajectoryBuilder().reverse().splineTo(new Pose2d(-36,-8,Math.toRadians(45))).build());
//        r.followTrajectorySync(r.trajectoryBuilder().splineTo(new Pose2d(-26,5,Math.toRadians(90))).build());
//        intake("out", 0.65);
//        sleep(220);
//        intake("in", 0.65);
//        sleep(500);
//        intake("out",0.65);
//        sleep(500);
//        intake( "off", 0);
//        //r.followTrajectorySync(r.trajectoryBuilder().splineTo(new Pose2d(-26,-20,Math.toRadians(-90))).build());
//
//        r.followTrajectorySync(r.trajectoryBuilder().splineTo(new Pose2d(-26,30,Math.toRadians(90))).build());
//
//        robot.blockServo.setPosition(robot.stoneGrab);
//        hooksDown();
//        r.followTrajectorySync(r.trajectoryBuilder().splineTo(new Pose2d(-30,56,Math.toRadians(180))).build());
//        r.followTrajectorySync(r.trajectoryBuilder().forward(5).build());
//        robot.leftLift.setTargetPosition(-59);
//        robot.rightLift.setTargetPosition(-59);
//
//        robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.leftLift.setPower(.9);
//        robot.rightLift.setPower(.9);
//
//
//        r.setPoseEstimate(new Pose2d());
//
//       // r.followTrajectorySync(r.trajectoryBuilder().reverse().splineTo(new Pose2d(-20,10,Math.toRadians(-90))).build());
//
//        robot.xSlide393.setPower(0.9);
//        sleep(1700);
//        robot.blockServo.setPosition(robot.stoneUp);
//        sleep(1200);
//        robot.leftLift.setTargetPosition(-159);
//        robot.rightLift.setTargetPosition(-159);
//
//        robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.leftLift.setPower(.9);
//        robot.rightLift.setPower(.9);
//
//        robot.xSlide393.setPower(-0.9);
//
//        hooksUp();
//        //sleep(2000);
//        r.followTrajectorySync(r.trajectoryBuilder().splineTo(new Pose2d(-20,-8,Math.toRadians(-90))).build());
//
//        robot.leftLift.setTargetPosition(0);
//        robot.rightLift.setTargetPosition(0);
//
//        robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.leftLift.setPower(.9);
//        robot.rightLift.setPower(.9);
//       // r.followTrajectorySync(r.trajectoryBuilder().reverse().splineTo(new Pose2d(-26,5,Math.toRadians(-90))).build());
//        //r.followTrajectorySync(r.trajectoryBuilder().reverse().splineTo(new Pose2d(-20,10,Math.toRadians(-90))).build());
//
//
//    }
//
//    public void intake(String direction, double speed){
//        if (direction == "in") {
//            robot.leftIntake.setPower(speed);
//            robot.rightIntake.setPower(speed);
//        }
//        if (direction == "out"){
//            robot.leftIntake.setPower(speed * -1);
//            robot.rightIntake.setPower(speed * -1);
//        }
//        if (direction == "off"){
//            robot.leftIntake.setPower(0);
//            robot.rightIntake.setPower(0);
//        }
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
