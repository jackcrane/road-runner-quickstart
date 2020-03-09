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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.revextensions2.ExpansionHubMotor;
//import org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms.DoubleMotorLift;
//import org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms.SimpleLift;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class TauBot
{
    /* Public OpMode members. */

    public DcMotor leftFront      = null;
    public DcMotor leftBack       = null;
    public DcMotor rightFront     = null;
    public DcMotor rightBack      = null;

    public  ExpansionHubMotor   leftIntake     = null;
    public  ExpansionHubMotor   rightIntake    = null;

    public DcMotorEx leftLift       = null;
    public DcMotorEx rightLift      = null;

    public Servo leftHook       = null;
    public Servo rightHook      = null;
//    public Servo capStone       = null;
//    public Servo blockServo     = null;
    public Servo middleDeadWheelServo = null;
//    public CRServo xSlide393      = null;
    public Servo frontGrab = null;
    public Servo backGrab = null;
    public Servo turnServo = null;
    public Servo linkageServo = null;

    public double   rightHookUp    = 0.37;
    public double   rightHookDown  = 0;
    public double   leftHookUp     = 0.34;
    public double   leftHookDown   = 0.69;

    public double   stoneGrab = 0.49;
    public double   stoneUp = 0;
    public double   stoneDispense = 0.35;

    public double frontGrabClosed = 0.21;
    public double frontGrabOpen = 0.8;
    public double frontGrabCap = 0.95;
    public double frontGrabDown = 0.27;

    public double backGrabClosed = 0.33;
    public double backGrabOpen = 0.92;


    public double linkageCollapsed = 0.21;
    public double linkage1Block = 0.54;
    public double linkageMax = 0.59;

    public double turnTableStraight = .07;
    public double turnTableNinety = .59;

    double   middleDeadWheelServoPositionUp = 0.25;
    double   middleDeadWheelServoPositionDown = 0.34;


    public Rev2mDistanceSensor liftDistanceSensor;
    double capStoneHold = 0.2;
    public BNO055IMU imu = null;

//    public DoubleMotorLift lift;
//    public SimpleLift pidLift;

    public DigitalChannel liftBeam = null;
    public DigitalChannel blockBeam = null;

//    public VoltageSensor leftIntakeVoltage = null;
//    public VoltageSensor rightIntakeVoltage = null;
    teamcode.Vision.camera camera;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public TauBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        PIDFCoefficients newPIDF = new PIDFCoefficients(10.0,  3.0,   0.0,  12.0);

        // Define and Initialize Motors
        leftBack  = hwMap.get(DcMotor.class, "leftBack");
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        rightBack  = hwMap.get(DcMotor.class, "rightBack");
        rightFront = hwMap.get(DcMotor.class, "rightFront");

        leftIntake = hwMap.get(ExpansionHubMotor.class, "leftIntake");
        rightIntake = hwMap.get(ExpansionHubMotor.class, "rightIntake");

        leftLift = hwMap.get(DcMotorEx.class, "leftLift");
        rightLift = hwMap.get(DcMotorEx.class, "rightLift");

        liftBeam = hwMap.get(DigitalChannel.class, "liftBeam");
        blockBeam = hwMap.get(DigitalChannel.class, "blockBeam");

        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);

        leftLift.setDirection(DcMotorEx.Direction.FORWARD);
        rightLift.setDirection(DcMotorEx.Direction.REVERSE);

        liftDistanceSensor = hwMap.get(Rev2mDistanceSensor.class, "liftDistanceSensor");

//        leftIntakeVoltage = hwMap.get(VoltageSensor.class, "leftIntake");
//        rightIntakeVoltage = hwMap.get(VoltageSensor.class, "rightIntake");

        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
        IMUParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMUParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //IMUParameters.accelerationIntegrationAlgorithm = new TauAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMUParameters);

        // Set all motors to zero power
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        leftLift.setPower(0);
        rightLift.setPower(0);
        leftLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

//        leftLift.setTargetPosition(0);
//        rightLift.setTargetPosition(0);
//        leftLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        rightLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);



        // Define and initialize ALL installed servos.

        leftHook = hwMap.get(Servo.class, "leftHook");
        rightHook = hwMap.get(Servo.class, "rightHook");
//        capStone = hwMap.get(Servo.class, "capStone");
//        blockServo = hwMap.get(Servo.class, "blockServo");
//        xSlide393 = hwMap.get(CRServo.class, "xSlide393");
//        xSlide393.setDirection(CRServo.Direction.REVERSE);

        leftHook.setPosition(leftHookUp);
        rightHook.setPosition(rightHookUp);
//        blockServo.setPosition(capStoneHold);
//        capStone.setPosition(.17);

        frontGrab = hwMap.get(Servo.class, "frontGripServo");
        backGrab = hwMap.get(Servo.class, "backGripServo");
        turnServo = hwMap.get(Servo.class, "turnServo");
        linkageServo = hwMap.get(Servo.class, "linkageServo");

//        leftLift.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);
//        rightLift.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);

        middleDeadWheelServo = hwMap.get(Servo.class, "middleDeadWheelServo");
        middleDeadWheelServo.setPosition(middleDeadWheelServoPositionUp);


    }
    /* Initialize standard Hardware interfaces */
    public void initAuto(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        double   stoneGrab = 0.49;
        double   stoneUp = 0;
        double   stoneDispense = 0.35;

//        PIDFCoefficients newPIDF = new PIDFCoefficients(10.0,  3.0,   0.0,  12.0);

        // Define and Initialize Motors
        leftIntake = hwMap.get(ExpansionHubMotor.class, "leftIntake");
        rightIntake = hwMap.get(ExpansionHubMotor.class, "rightIntake");

        leftLift = hwMap.get(DcMotorEx.class, "leftLift");
        rightLift = hwMap.get(DcMotorEx.class, "rightLift");

        liftBeam = hwMap.get(DigitalChannel.class, "liftBeam");
        blockBeam = hwMap.get(DigitalChannel.class, "blockBeam");

//        rightIntake.setDirection(DcMotor.Direction.REVERSE);

//        leftLift.setDirection(DcMotorEx.Direction.FORWARD);
        leftLift.setDirection(DcMotorEx.Direction.REVERSE);
        rightLift.setDirection(DcMotorEx.Direction.REVERSE);

        liftDistanceSensor = hwMap.get(Rev2mDistanceSensor.class, "liftDistanceSensor");

//        leftIntakeVoltage = hwMap.get(VoltageSensor.class, "leftIntake");
//        rightIntakeVoltage = hwMap.get(VoltageSensor.class, "rightIntake");

//        camera = new camera(this);

//        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
//        IMUParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        IMUParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //IMUParameters.accelerationIntegrationAlgorithm = new TauAccelerationIntegrator();
//        imu = hwMap.get(BNO055IMU.class, "imu");
//        imu.initialize(IMUParameters);

        // Set all motors to zero power

        leftIntake.setPower(0);
        rightIntake.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        leftLift.setPower(0);
        rightLift.setPower(0);
//        leftLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        rightLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        leftLift.setTargetPosition(0);
//        rightLift.setTargetPosition(0);
//        leftLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        rightLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);



        // Define and initialize ALL installed servos.

        leftHook = hwMap.get(Servo.class, "leftHook");
        rightHook = hwMap.get(Servo.class, "rightHook");
//        capStone = hwMap.get(Servo.class, "capStone");
//        blockServo = hwMap.get(Servo.class, "blockServo");
        middleDeadWheelServo = hwMap.get(Servo.class, "middleDeadWheelServo");
//        xSlide393 = hwMap.get(CRServo.class, "xSlide393");
//        xSlide393.setDirection(CRServo.Direction.REVERSE);

        leftHook.setPosition(leftHookUp);
        rightHook.setPosition(rightHookUp);
//        blockServo.setPosition(stoneUp);
        middleDeadWheelServo.setPosition(middleDeadWheelServoPositionDown);

        frontGrab = hwMap.get(Servo.class, "frontGripServo");
        backGrab = hwMap.get(Servo.class, "backGripServo");
        turnServo = hwMap.get(Servo.class, "turnServo");
        linkageServo = hwMap.get(Servo.class, "linkageServo");

//        leftLift.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);
//        rightLift.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);

    }
 }

