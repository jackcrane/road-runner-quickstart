package teamcode.roadrunner.hardware;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TwoWheelLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1.88976 / 2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 8.15272500716; // in; distance between the left and right wheels
    public static double MIDDLE_X_OFFSET = -6; // in; offset of the lateral wheel
    public static double MIDDLE_Y_OFFSET = 0; // in; offset of the lateral wheel

    private MecanumDrive drive;

    private DcMotor rightEncoder, middleEncoder;

    private double leftLast = 0;
    private double rightLast = 0;
    private double middleLast = 0;

    private double PARALLEL_MULTIPLIER = 1;
    private double MIDDLE_MULTIPLIER = (55.5 / 54.8) * (50 / 49.87992200231521) * (43.125 / 43.75);

    public TwoWheelLocalizer(HardwareMap hardwareMap, MecanumDrive drive) {
        super(Arrays.asList(
//                new Pose2d(-5.875, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(-5.875, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(MIDDLE_X_OFFSET, MIDDLE_Y_OFFSET, Math.toRadians(90)) // front))
//                new Pose2d(MIDDLE_X_OFFSET, MIDDLE_Y_OFFSET, Math.toRadians(90)) // front))
        ));


        rightEncoder = hardwareMap.dcMotor.get("rightIntake");
        middleEncoder = hardwareMap.dcMotor.get("leftLift");

        this.drive = drive;
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }


    @Override
    public double getHeading() {
        return drive.getExternalHeading();
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        try {
            rightLast = encoderTicksToInches(rightEncoder.getCurrentPosition()) * PARALLEL_MULTIPLIER;
            middleLast = encoderTicksToInches(middleEncoder.getCurrentPosition()) * MIDDLE_MULTIPLIER;

            TelemetryPacket packet = new TelemetryPacket();

            packet.put("right", rightEncoder.getCurrentPosition());
            packet.put("middle", middleEncoder.getCurrentPosition());

//            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        } catch(Exception e) {
            Log.i("bulkRead", "null thing");
        }

        return Arrays.asList(rightLast, middleLast);
    }
}
