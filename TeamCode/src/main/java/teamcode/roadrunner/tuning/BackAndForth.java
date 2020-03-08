package teamcode.roadrunner.tuning;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import teamcode.roadrunner.hardware.DriveConstants;
import teamcode.roadrunner.hardware.SampleMecanumDrive;

@Autonomous(name="Tuning - Back and Forth", group="tuning")
public class BackAndForth extends LinearOpMode {

    public static double DISTANCE = 50;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive driveBaseMecanumOld = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectoryForward = driveBaseMecanumOld.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        Trajectory trajectoryBackward = new TrajectoryBuilder(trajectoryForward.end(),trajectoryForward.end().getHeading(),
                DriveConstants.BASE_CONSTRAINTS)
                .back(DISTANCE)
                .build();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            driveBaseMecanumOld.followTrajectory(trajectoryForward);
            driveBaseMecanumOld.followTrajectory(trajectoryBackward);
        }
    }
}
