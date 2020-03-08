//package teamcode.CompOpModes.RedAutoModes;
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
//@Autonomous(name = "RedWalnutSkyStoneAuto")
//
//public class RedWalnutSkyStoneAuto extends LinearOpMode {
//
//    private SampleMecanumDriveREVOptimized r;
//    private TauBot robot;
//    boolean  colelctJam     = false;
//    private int stonePos;
//
//    public void runOpMode() throws InterruptedException {
//        r = new SampleMecanumDriveREVOptimized(hardwareMap);
//        robot = new TauBot();
//
//        //run vision somewhere here
//        stonePos = 3;
//
//        Trajectory stoneThreeTrajectory = r.trajectoryBuilder()
//                //.reverse().splineTo(new Pose2d(-35,-14,Math.toRadians(-45)))//to block
//                /*forward*/  //.reverse().splineTo(new Pose2d(-26,-33,Math.toRadians(-90)))//under bridge
//                .splineTo(new Pose2d(-26,-70,Math.toRadians(-90)))//to foundation
//                //.reverse().splineTo(new Pose2d(-26,0,Math.toRadians(-90)))//to bridge
//                .splineTo(new Pose2d(-35,2,Math.toRadians(-90)))//to second block
//
//                .build();
//        Trajectory stoneThreePartOneTwoSplit = r.trajectoryBuilder()
//                //.reverse().splineTo(new Pose2d(-26,-33,Math.toRadians(-90)))//under bridge
//                .splineTo(new Pose2d(-26,-70,Math.toRadians(-90)))//to foundation
//                .build();
//        Trajectory stoneThreePartTwoTrajectory = r.trajectoryBuilder()
//                //.reverse().splineTo(new Pose2d(-26,-60,Math.toRadians(-90)))//to bridge
//                // .splineTo(new Pose2d(-35,2,Math.toRadians(-60)))//to second block
//                .build();
//
//        robot.initAuto(hardwareMap);
//        waitForStart();
//
//
//        if (isStopRequested()) return;
//
//        intake("in", 0.65);
//        if (stonePos == 3) {
//            //r.followTrajectorySync(stoneThreeTrajectory);
//            r.followTrajectorySync(
//                    r.trajectoryBuilder()
//                            //.reverse().splineTo(new Pose2d(-35,-16,Math.toRadians(-45)))//to block
//                            .build()
//            );
//            sleep(100);
//            r.followTrajectorySync(
//                    r.trajectoryBuilder()
//                            .splineTo(new Pose2d(-26,-33,Math.toRadians(-90)))//to bridge
//                            .splineTo(new Pose2d(-26,-55,Math.toRadians(-90)))//to ofundation
//                            .build()
//            );
//
//            intake("off", 0.65);
//
//            robot.blockServo.setPosition(robot.stoneGrab);
//            robot.xSlide393.setPower(-0.9);
//            sleep(2200);
//            robot.xSlide393.setPower(0);
//            raiseLiftClicks(-80);
//
//            robot.xSlide393.setPower(0.9);
//
//            r.followTrajectorySync(
//                    r.trajectoryBuilder()
//                            .splineTo(new Pose2d(-26,-88,Math.toRadians(-90)))//back towards wall
//                            .build()
//            );
//
//            robot.xSlide393.setPower(0);
//            robot.blockServo.setPosition(robot.stoneUp);
//            raiseLiftClicks(-90);
//            sleep(200);
//            robot.xSlide393.setPower(-0.9);
//            sleep(1200);
//            robot.xSlide393.setPower(0);
//
//            robot.leftLift.setTargetPosition(0);
//            robot.rightLift.setTargetPosition(0);
//
//            robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            robot.leftLift.setPower(.85);
//            robot.rightLift.setPower(.85);
//
//            intake("in", 0.65);
//
//            r.followTrajectorySync(
//                    r.trajectoryBuilder()
//                            //.reverse()
//                            .splineTo(new Pose2d(-26,-55,Math.toRadians(-90)))//to ofundation
//                            .splineTo(new Pose2d(-26,-33,Math.toRadians(-90)))//to bridge
//                            .splineTo(new Pose2d(-38, -13, Math.toRadians(-60))) //towards 2nd block
//                            .build()
//            );
//            sleep(150);
//            r.followTrajectorySync(
//                    r.trajectoryBuilder()
//                            .splineTo(new Pose2d(-28,-33,Math.toRadians(-90)))//to bridge
//                            .splineTo(new Pose2d(-28,-55,Math.toRadians(-90)))//to ofundation
//                            .build()
//            );
//
//            intake("off", 0.65);
//
//            robot.blockServo.setPosition(robot.stoneGrab);
//            robot.xSlide393.setPower(-0.9);
//            sleep(1200);
//            robot.xSlide393.setPower(0);
//            raiseLiftClicks(-320);
//
//            robot.xSlide393.setPower(0.9);
//
//            r.followTrajectorySync(
//                    r.trajectoryBuilder()
//                            .splineTo(new Pose2d(-28,-88,Math.toRadians(-90)))//back towards wall
//                            .build()
//            );
//
//            robot.xSlide393.setPower(0);
//            robot.blockServo.setPosition(robot.stoneUp);
//            //sleep(200);
//            robot.xSlide393.setPower(-0.9);
//            sleep(1200);
//            robot.xSlide393.setPower(0);
//
//            robot.leftLift.setTargetPosition(0);
//            robot.rightLift.setTargetPosition(0);
//
//            robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            robot.leftLift.setPower(.85);
//            robot.rightLift.setPower(.85);
//
//            r.followTrajectorySync(
//                    r.trajectoryBuilder()
//                           // .reverse()
//                            .splineTo(new Pose2d(-26,-40,Math.toRadians(-90)))//to bridge
//                            .build()
//            );
//
//
//            /*r.followTrajectorySync(
//                    r.trajectoryBuilder()
//                            .reverse().splineTo(new Pose2d(-26,-7,Math.toRadians(-90)))
//                            .strafeLeft(22)
//                            .build()
//            );
//            r.followTrajectorySync(
//                    r.trajectoryBuilder()
//                            .back(7)
//                            .build()
//            );
//            r.followTrajectorySync(
//                    r.trajectoryBuilder()
//                            .strafeRight(22)
//
//                            .build()
//            );
//            r.setPoseEstimate(new Pose2d(-26,0,Math.toRadians(-90)));
//            r.followTrajectorySync(
//                    r.trajectoryBuilder()
//                    .splineTo(new Pose2d(-26,-33,Math.toRadians(-90)))
//                    .splineTo(new Pose2d(-26,-70,Math.toRadians(-90)))
//                    .build()
//            );*/
//        }
//        /*if (stonePos == 2) {
//            r.followTrajectorySync(
//                    r.trajectoryBuilder()
//                            .reverse()
//                            .splineTo(new Pose2d(-2,0,Math.toRadians(0)))
//                            .splineTo(new Pose2d(-26,-12,Math.toRadians(-90)))//to block
//                            .build()
//            );
//            r.followTrajectorySync(
//                    r.trajectoryBuilder()
//                            .strafeLeft(22)
//                            .build()
//            );
//            r.followTrajectorySync(
//                    r.trajectoryBuilder()
//                            .back(4)
//                            .build()
//            );
//            r.followTrajectorySync(
//                    r.trajectoryBuilder()
//                            .strafeRight(25)
//
//                            .build()
//            );
//            r.setPoseEstimate(new Pose2d(-26,0,Math.toRadians(-90)));
//            r.followTrajectorySync(
//                    r.trajectoryBuilder()
//                            .splineTo(new Pose2d(-26,-33,Math.toRadians(-90)))
//                            .splineTo(new Pose2d(-26,-70,Math.toRadians(-90)))
//                            .build()
//            );
//
//            //second block
//            r.followTrajectorySync(
//                    r.trajectoryBuilder()
//                            .reverse()
//                            .splineTo(new Pose2d(-26,-12,Math.toRadians(-90)))//to block
//                            .build()
//            );
//            r.followTrajectorySync(
//                    r.trajectoryBuilder()
//                            .strafeLeft(22)
//                            .build()
//            );
//            r.followTrajectorySync(
//                    r.trajectoryBuilder()
//                            .back(4)
//                            .build()
//            );
//            r.followTrajectorySync(
//                    r.trajectoryBuilder()
//                            .strafeRight(25)
//
//                            .build()
//            );
//            r.setPoseEstimate(new Pose2d(-26,0,Math.toRadians(-90)));
//            r.followTrajectorySync(
//                    r.trajectoryBuilder()
//                            .splineTo(new Pose2d(-26,-33,Math.toRadians(-90)))
//                            .splineTo(new Pose2d(-26,-70,Math.toRadians(-90)))
//                            .build()
//            );
//
//        }*/
//
//        //r.followTrajectorySync(r.trajectoryBuilder().splineTo(new Pose2d(-26,-20,Math.toRadians(-90))).build());
//
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
//        robot.leftHook.setPosition(robot.leftHookDown);
//        robot.rightHook.setPosition(robot.rightHookDown);
//    }
//
//    public void hooksUp(){
//        robot.leftHook.setPosition(robot.leftHookUp);
//        robot.rightHook.setPosition(robot.rightHookUp);
//    }
//    public void raiseLiftClicks(int targetClicks){
////        if (targetClicks < maxLiftPos){
////            targetClicks = maxLiftPos;
////        }
//        robot.leftLift.setTargetPosition(targetClicks);
//        robot.rightLift.setTargetPosition(targetClicks);
//
//        robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.leftLift.setPower(.9);
//        robot.rightLift.setPower(.9);
//    }
//}
