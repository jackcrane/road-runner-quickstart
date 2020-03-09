/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package teamcode.CompOpModes.TeleopAndRobotFiles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.openftc.revextensions2.ExpansionHubEx;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 * <p>
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Teleop", group = "testing")
@Config
public class Teleop extends LinearOpMode {

    /* Declare OpMode members. */
    TauBot robot = new TauBot();


    FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double kP = 0.006;
    public static double kG = -0.0028;

    int depositOffset = -61;
    //int[] liftPos = {-64, -242, -412, -589, -745, -920, -1078, -1225, -1399, -1578, -1739, -1800};
    int[] liftPos = {
            -57 + depositOffset,
            -226 + depositOffset,
            -402 + depositOffset,
            -562 + depositOffset,
            -737 + depositOffset,
            -902 + depositOffset,
            -1068 + depositOffset,
            -1223 + depositOffset,
            -1389 + depositOffset,
            -1555 + depositOffset,
            -1721 + depositOffset,
            -1800 + depositOffset
    };
    int maxLiftPos = -1780;
    public ElapsedTime vexServoTime = new ElapsedTime(5);
    public ElapsedTime delayVexServoTime = new ElapsedTime(5);
    public ElapsedTime collectJamTime = new ElapsedTime(10);
    String collecting = "off";
    boolean collectJam = false;
    boolean xAxisDelayedBoolean = false;
    int cycleCounter = 1;
    int capLiftConstant = -7;

    public static double liftP = 0.01;
    public static double liftI = 0;
    public static double liftD = 0.002;
    public static double liftG = 0;

    private PIDControllerJava liftController = new PIDControllerJava(liftP, liftI, liftD, liftG);
    public static int liftTargetPosition = 0;
    private int liftZeroOffset = 0;

    public boolean retractingLifts = false;
    public double vexMotorPower = 0.9;
    public boolean cappingMode = false;

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        //hook constants
        double rightHookUp = 0.37;
        double rightHookDown = 0;
        double leftHookUp = 0.34;
        double leftHookDown = 0.69;
        boolean hookReset = true;
        boolean hookUp = true;

        //movement
        double horizontal;
        double forwards;
        double turning;
        boolean slowMode = false;
        boolean reset = true;
        double slowModeTurn = 0.65;
        double slowModeForward = 0.6;
        double slowModehorizontal = 1.0;

        //cycle & collection
        double intakeSpeed = 0.8;

        int rightBumpCounter = 0;
        boolean holding = false;
        boolean cycleReset = true;
        boolean leftBumperReset = true;

        //lift
        boolean goingDown = false;
        boolean goingUp = false;
        int liftCorrectionCounter = 0;
        double stoneGrab = robot.stoneGrab;
        double stoneUp = robot.stoneUp;
        double stoneDispense = robot.stoneDispense;
        ElapsedTime stoneHoldingTime = new ElapsedTime();

        boolean liftAllTheWayDown = true;
        boolean xAxisLiftIsMoveable = false;


        //Cap Stone
        boolean capStoneBoolean = true;
        boolean capReset = true;
        double capStoneHold = 0.17;
        double capStoneRelease = 0.38;

        boolean xReset = true;
        boolean yReset = true;


        // run until the end of the match (driver presses STOP)
        while (!isStopRequested()) {

            //hook control
            if (gamepad1.a && hookReset) {
                hookReset = false;
                if (hookUp) {
                    robot.rightHook.setPosition(rightHookDown);
                    robot.leftHook.setPosition(leftHookDown);
                    hookUp = false;
                } else {
                    robot.rightHook.setPosition(rightHookUp);
                    robot.leftHook.setPosition(leftHookUp);
                    hookUp = true;
                }
            } else if (!gamepad1.a && !hookReset) {
                hookReset = true;

            }

            //decreasing cycle count value
            if (gamepad1.x && xReset) {
                xReset = false;
                if (cycleCounter > 1) {
                    cycleCounter--;
                }
            } else if (!gamepad1.x && !xReset) {
                xReset = true;

            }

            if (gamepad1.y && yReset) {
                yReset = false;
                if (cycleCounter < 12) {
                    cycleCounter++;
                }
            } else if (!gamepad1.y && !yReset) {
                yReset = true;

            }

            //capstone controls
            if (gamepad1.b && capReset) {
                capReset = false;
                if (capStoneBoolean) {
//                    robot.capStone.setPosition(capStoneRelease);
                    capStoneBoolean = false;

                } else {
//                    robot.capStone.setPosition(capStoneHold);
                    capStoneBoolean = true;
                }
            } else if (!gamepad1.b && !capReset) {
                capReset = true;

            }


            // collection&cycle
            if (gamepad1.right_bumper && cycleReset) { //prevents multiple inputs
                cycleReset = false;              //every time the right bumper is pressed
                if (rightBumpCounter == 6) {
                    rightBumpCounter = 1;
                } else if (rightBumpCounter == 4) {
                    if (cycleCounter < 12) {
                        cycleCounter++;
                    }
                    rightBumpCounter++;
                } else {
                    rightBumpCounter++;
                }
            } else if (!gamepad1.right_bumper && !cycleReset) {
                cycleReset = true;
            }

//            if (gamepad1.left_bumper && leftBumperReset) { //prevents multiple inputs
//                leftBumperReset = false;              //every time the right bumper is pressed
//                if (rightBumpCounter == 1) {
//                    rightBumpCounter = 6;
//                } else {
//                    rightBumpCounter--;
//                }
//            } else if (!gamepad1.left_bumper && !leftBumperReset) {
//                leftBumperReset = true;
//            }
//
//            if (gamepad2.x) {
//                runLiftToZero();
//            }

            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                if (rightBumpCounter == 1) { //collect and raise lift
                    //raiseLiftClicks(-30);
                    collectJam = true;
                    robot.frontGrab.setPosition(robot.frontGrabDown);
                    robot.backGrab.setPosition(robot.backGrabOpen);
                    robot.turnServo.setPosition(robot.turnTableStraight);
                } else if (rightBumpCounter == 2) { //grab block and lower lift
                    //vexMotorPower = Math.abs(vexMotorPower) * -1;
                    //vexServoTime.reset();
                    //collapseLifts();
                    collectJam = false;
                    collecting = "off";
                    robot.frontGrab.setPosition(robot.frontGrabClosed);
                    robot.backGrab.setPosition(robot.backGrabClosed);
                    stoneHoldingTime.reset();
                    holding = true;
                    liftAllTheWayDown = true;
                } else if (rightBumpCounter == 3) { //extend both lifts
                    extendLifts(cycleCounter);
                } else if (rightBumpCounter == 4) { //release blocks
                    //score
                    robot.frontGrab.setPosition(robot.frontGrabOpen);
                    robot.backGrab.setPosition(robot.backGrabOpen);
                    holding = false;
                } else if (rightBumpCounter == 5) {
                    robot.linkageServo.setPosition(robot.linkageCollapsed);
                } else if (rightBumpCounter == 6) { //lower lift\
                    runLiftToZero();
//                    robot.blockServo.setPosition(stoneUp);
                }
            }


//            if (holding && stoneHoldingTime.seconds() > 1 && liftAllTheWayDown){
////                robot.leftLift.setTargetPosition(-100);
////                robot.rightLift.setTargetPosition(-100);
//                liftAllTheWayDown = false;
//                xAxisLiftIsMoveable = true;
//            }
//
//            if (holding && stoneHoldingTime.seconds() > 1.5 && xAxisLiftIsMoveable){
//                //code to move x axis lift out
//                xAxisLiftIsMoveable = false;
//            }


//            if (robot.blockBeam.getState() && rightBumpCounter == 1){
//                rightBumpCounter = 2;
//                runLiftToZero();
//                collecting = "off";
//                robot.blockServo.setPosition(stoneGrab);
//                stoneHoldingTime.reset();
//                holding = true;
//                liftAllTheWayDown = true;
//            }

            intake(collecting, intakeSpeed);

//            //x-axis lift controls
//            if (gamepad1.left_bumper && holding) {
////                robot.xSlide393.setPower(0.9);
//            } else if (gamepad1.left_bumper && !holding) {
////                robot.xSlide393.setPower(-0.9);
//            } else if (gamepad1.dpad_left) {
////                robot.xSlide393.setPower(0.9);
//            } else if (gamepad1.dpad_right) {
////                robot.xSlide393.setPower(-0.9);
//            } else if (cycleCounter == 1 && vexServoTime.seconds() > .35 && vexServoTime.seconds() < 2.35) { // delays the x-lift when its the first cycle
////                robot.xSlide393.setPower(vexMotorPower);
//            } else if (vexServoTime.seconds() < 2 && cycleCounter != 1) {
////                robot.xSlide393.setPower(vexMotorPower);
//            } else {
////                robot.xSlide393.setPower(0);
//            }

//            if (xAxisDelayedBoolean){
//                xAxisDelayed();
//            }
//
//            if (delayVexServoTime.seconds() > 2.15){
//                xAxisDelayedBoolean = false;
//            }

            //collector jam code
            if (collectJam) {
                if ((robot.rightIntake.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS) > 6000 || robot.leftIntake.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS) > 6000) && collectJamTime.seconds() > .4) {
                    collectJamTime.reset();
                } else if (collectJamTime.seconds() > .15 && collectJamTime.seconds() < .27) {
                    collecting = "out";
                } else {
                    collecting = "in";
                }
            }

            //manual y-axis lift controls

            if (gamepad1.right_trigger > 0.2) {
                raiseLiftClicks(robot.leftLift.getTargetPosition() + capLiftConstant);
            }
            if (gamepad1.left_trigger > 0.2) {
                raiseLiftClicks(robot.leftLift.getTargetPosition() - capLiftConstant);
            }
            /*
//            if (gamepad1.right_trigger > 0.2) {
//
//                robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//                robot.leftLift.setPower(gamepad1.right_trigger);
//                robot.rightLift.setPower(gamepad1.right_trigger);
//
//                goingUp = true;
//                goingDown = false;
//
//            } else if (gamepad1.left_trigger > 0.1) {
//
//                robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//                robot.leftLift.setPower(-0.4);
//                robot.rightLift.setPower(-0.4);
//
//                liftCorrectionCounter = 0;
//                goingUp = false;
//                goingDown = true;
//
//            } else {
//
//                if (goingDown) {
//                    robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    robot.leftLift.setPower(-0.4);
//                    robot.rightLift.setPower(-0.4);
//                } else if (goingUp) {
//                    robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    robot.leftLift.setPower(0.1);
//                    robot.rightLift.setPower(0.1);
//                } else {
//                    robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    robot.leftLift.setPower(0);
//                    robot.rightLift.setPower(0);
//                    /*if (liftCorrectionCounter < 8) {
//                        robot.leftLift.setPower(0.1);
//                        robot.rightLift.setPower(0.1);
//                        liftCorrectionCounter++;
//                    } else {
//                        robot.leftLift.setPower(0);
//                        robot.rightLift.setPower(0);
//                    }
//                }
//            }


            if (!robot.liftBeam.getState()) {
                goingDown = false;
            }
            */

            //Movement
            if (gamepad1.left_stick_button && gamepad1.right_stick_button && reset) {
                slowMode = !slowMode;
                reset = false;
            } else if (!(gamepad1.left_stick_button || gamepad1.right_stick_button) && !reset) {
                reset = true;
            }

            if (slowMode) {
                horizontal = gamepad1.left_stick_x * slowModehorizontal;
                forwards = (-gamepad1.left_stick_y) * slowModeForward;
                turning = gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x) * slowModeTurn;
            } else {
                horizontal = gamepad1.left_stick_x;
                forwards = (-gamepad1.left_stick_y);
                turning = gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x);
            }

            drive(forwards, horizontal, turning);


            // run to position with lift test buttons
/*
            if (gamepad1.x){
                robot.leftLift.setTargetPosition(-800);
                robot.rightLift.setTargetPosition(-800);

                robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.leftLift.setPower(.85);
                robot.rightLift.setPower(.85);
//                liftTargetPosition = -200;

            } else if (gamepad1.b){
                robot.leftLift.setTargetPosition(0);
                robot.rightLift.setTargetPosition(0);

                robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.leftLift.setPower(.65);
                robot.rightLift.setPower(.65);
//                liftTargetPosition = -200;

            } else if (!robot.leftLift.isBusy()){
                robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.leftLift.setPower(-0.1);
                robot.rightLift.setPower(-.1);
            }

            robot.leftLift.setTargetPosition(liftTargetPosition);
            robot.rightLift.setTargetPosition(liftTargetPosition);

            robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftLift.setPower(.15);
            robot.rightLift.setPower(.15);

             lift PID
            int currentPosition = robot.leftLift.getCurrentPosition();
            double power = liftPIDController.update(liftTargetPosition - currentPosition);

            if(currentPosition >= 0 && liftTargetPosition >= 0) {
                robot.leftLift.setPower(0);
                robot.rightLift.setPower(0);
            } else {
                robot.leftLift.setPower(power);
                robot.rightLift.setPower(power);
            } */
            //misconlanious test code for old lift pid
//            telemetry.addData("Lift power", power);

            liftController.setKP(liftP);
            liftController.setKI(liftI);
            liftController.setKD(liftD);
            liftController.setKG(liftG);

            int currentPositionRaw = robot.rightLift.getCurrentPosition();
            int currentPosition = currentPositionRaw - liftZeroOffset;

            double liftPower = liftController.update(liftTargetPosition - currentPosition);
            if (!robot.liftBeam.getState() && liftTargetPosition - currentPosition > 0) {
                liftZeroOffset = currentPositionRaw;

                if (liftTargetPosition - currentPosition > 0) {
                    robot.rightLift.setPower(0);
                    robot.leftLift.setPower(0);
                }
            } else {
                robot.rightLift.setPower(liftPower);
                robot.leftLift.setPower(liftPower);
            }

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("currentPos", currentPosition);
            packet.put("targetPos", liftTargetPosition);
            packet.put("error", liftTargetPosition - currentPosition);
            packet.put("motorPower", liftPower);

            dashboard.sendTelemetryPacket(packet);

            idle();

            telemetry.addData("Lift Run Mode", robot.leftLift.getMode());
            telemetry.addData("Target Pos: ", robot.leftLift.getTargetPosition());
            telemetry.addData("left lift current Pos:", robot.leftLift.getCurrentPosition());
            telemetry.addData("right lift current Pos:", robot.rightLift.getCurrentPosition());

            // lift motor power telem
            /*
            telemetry.addData("Left Lift Motor Power", robot.leftLift.getPower());
            telemetry.addData("Right Lift Motor Power", robot.rightLift.getPower());
            */

            //lift busy telemetry
            /*
//            telemetry.addData("leftBusy",robot.leftLift.isBusy());
//            telemetry.addData("right busy", robot.rightLift.isBusy());

             */

            telemetry.addData("\nliftBeam", robot.liftBeam.getState());
            telemetry.addData("block beam", robot.blockBeam.getState());

            telemetry.addData("\ncycle counter", cycleCounter);
            telemetry.addData("right bump counter", rightBumpCounter);

            telemetry.addData("\n vex servo power: ", vexMotorPower);

//            intake voltange telemetry
            /*
//            telemetry.addData("left Intake voltage ",robot.leftIntake.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS));
//            telemetry.addData("right Intake voltage ",robot.rightIntake.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS));
*/

            telemetry.update();
        }
    }


    public void intake(String direction, double speed) {
        if (direction == "in") {
            robot.leftIntake.setPower(-speed);
            robot.rightIntake.setPower(-speed);
        }
        if (direction == "out") {
            robot.leftIntake.setPower(speed * 1);
            robot.rightIntake.setPower(speed * 1);
        }
        if (direction == "off") {
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
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

    public void drive(double forwards, double horizontal, double turning) {
        double leftFront = forwards + horizontal + turning;
        double leftBack = forwards - horizontal + turning;
        double rightFront = forwards - horizontal - turning;
        double rightBack = forwards + horizontal - turning;

        double[] wheelPowers = {Math.abs(rightFront), Math.abs(leftFront), Math.abs(leftBack), Math.abs(rightBack)};
        double biggestInput = highestValue(wheelPowers);

//        if (biggestInput > 1) {
        leftFront /= biggestInput;
        leftBack /= biggestInput;
        rightFront /= biggestInput;
        rightBack /= biggestInput;
//        }

        robot.leftBack.setPower(leftBack);
        robot.rightFront.setPower(rightFront);
        robot.leftFront.setPower(leftFront);
        robot.rightBack.setPower(rightBack);

    }

    public void raiseLiftCycle(int cycleCounter) {
        raiseLiftClicks(liftPos[cycleCounter - 1]);
    }

    public void raiseLiftClicks(int targetClicks) {
//        if (targetClicks < maxLiftPos){
//            targetClicks = maxLiftPos;
//        }

//        robot.leftLift.setTargetPosition(targetClicks);
//        robot.rightLift.setTargetPosition(targetClicks);
//
//        robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.leftLift.setPower(.9);
//        robot.rightLift.setPower(.9);
        targetClicks = Range.clip(targetClicks, maxLiftPos, 0);
        liftTargetPosition = targetClicks;
    }

    public void extendLifts(int cycleCounter) {
        //vexMotorPower = Math.abs(vexMotorPower);
        //vexServoTime.reset();
        robot.linkageServo.setPosition(robot.linkage1Block);
        raiseLiftCycle(cycleCounter);
    }

    public void runLiftToZero() {
//        if (robot.leftLift.getTargetPosition() < -240) { //if the lift is low send it down quick (prevent it form not going down all the way) else slow (preventing overshooting
//            robot.leftLift.setTargetPosition(0);
//            robot.rightLift.setTargetPosition(0);
//
//            robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            robot.leftLift.setPower(.85);
//            robot.rightLift.setPower(.85);
//        } else {
//            robot.leftLift.setTargetPosition(0);
//            robot.rightLift.setTargetPosition(0);
//
//            robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            robot.leftLift.setPower(.85);
//            robot.rightLift.setPower(.85);
//        }
        liftTargetPosition = 0;
    }

    public void collapseLifts() {
//        vexMotorPower = Math.abs(vexMotorPower) * -1;
//        vexServoTime.reset();
        robot.linkageServo.setPosition(robot.linkageCollapsed);
        runLiftToZero();
    }

//    public void xAxisDelayed(){
//        if(delayVexServoTime.seconds() > .15 && delayVexServoTime.seconds() < 2.15){
//            robot.xSlide393.setPower(Math.abs(vexMotorPower));
//        }
//    }


}
