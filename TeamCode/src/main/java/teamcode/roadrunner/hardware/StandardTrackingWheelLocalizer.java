package teamcode.roadrunner.hardware;

import android.support.annotation.NonNull;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1.88976 / 2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 8.15272500716; // in; distance between the left and right wheels
//    public static double LATERAL_DISTANCE = 7.92; // in; distance between the left and right wheels
//    public static double FORWARD_OFFSET = -12; // in; offset of the lateral wheel
//    public static double MIDDLE_X_OFFSET = (1/8); // in; offset of the lateral wheel
//    public static double MIDDLE_Y_OFFSET = -12; // in; offset of the lateral wheel
     public static double MIDDLE_X_OFFSET = -6; // in; offset of the lateral wheel
    public static double MIDDLE_Y_OFFSET = 0; // in; offset of the lateral wheel

    private DcMotor leftEncoder, rightEncoder, middleEncoder;

    private double leftLast = 0;
    private double rightLast = 0;
    private double middleLast = 0;
    
    private double PARALLEL_MULTIPLIER = 1;
    private double MIDDLE_MULTIPLIER = (55.5 / 54.8) * (50 / 49.87992200231521) * (43.125 / 43.75);

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(-5.875, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(-5.875, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(MIDDLE_X_OFFSET, MIDDLE_Y_OFFSET, Math.toRadians(90)) // front
        ));

        leftEncoder = hardwareMap.dcMotor.get("leftIntake");
        rightEncoder = hardwareMap.dcMotor.get("rightIntake");
        middleEncoder = hardwareMap.dcMotor.get("leftLift");

//        middleEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        try {
            leftLast = encoderTicksToInches(leftEncoder.getCurrentPosition()) * PARALLEL_MULTIPLIER;
            rightLast = encoderTicksToInches(rightEncoder.getCurrentPosition()) * PARALLEL_MULTIPLIER;
            middleLast = encoderTicksToInches(middleEncoder.getCurrentPosition()) * MIDDLE_MULTIPLIER;

            TelemetryPacket packet = new TelemetryPacket();

            packet.put("left", leftEncoder.getCurrentPosition());
            packet.put("right", rightEncoder.getCurrentPosition());
            packet.put("middle", middleEncoder.getCurrentPosition());

//            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        } catch(Exception e) {
            Log.i("bulkRead", "null thing");
        }

        return Arrays.asList(leftLast, rightLast, middleLast);
    }
}