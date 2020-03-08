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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a pushbot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "testerTeleop", group = "TauBot")

public class testerTeleop extends LinearOpMode {
    TauBot robot           = new TauBot();


    // Define class members


    @Override
    public void runOpMode() {
        robot.initAuto(hardwareMap);
        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.


        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()){

            double position = robot.linkageServo.getPosition();

            if (gamepad1.left_bumper) {
                position += 0.001;
            } else if (gamepad1.right_bumper) {
                position -= 0.001;
            }

            robot.linkageServo.setPosition(Range.clip(position, 0, 0.99));
            telemetry.addData("Servo Position", "%5.2f", robot.linkageServo.getPosition());

            double positionb = robot.turnServo.getPosition();

            if (gamepad1.a) {
                positionb += 0.001;
            } else if (gamepad1.b) {
                positionb -= 0.001;
            }

            robot.turnServo.setPosition(Range.clip(positionb, 0, 0.99));
            telemetry.addData("Servo Position", "%5.2f", robot.turnServo.getPosition());
//
//
//            double positionc = robot.rightHook.getPosition();
//
//            if (gamepad1.x) {
//                positionc += 0.001;
//            } else if (gamepad1.y) {
//                positionc -= 0.001;
//            }
//
//            robot.rightHook.setPosition(Range.clip(positionc, 0, 0.99));
//            telemetry.addData("Servo Position", "%5.2f", robot.rightHook.getPosition());

            if(gamepad1.x) {
                robot.linkageServo.setPosition(robot.linkageMax);
            }

            if(gamepad1.y) {
                robot.linkageServo.setPosition(robot.linkageCollapsed);
            }


            telemetry.update();


            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
