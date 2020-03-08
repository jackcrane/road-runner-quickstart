package teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

//import org.firstinspires.ftc.teamcode.RobotLibs;


public class camera {
    public OpenCvCamera webcam;
    SkystoneDetectorPipeline pipeline;
    OpMode opMode;

    public camera(OpMode mode) {
        opMode = mode;
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDetectorPipeline();
        webcam.openCameraDevice();
        webcam.setPipeline(pipeline);
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    public int getSkyPos(boolean allianceColorisRed) {
        int skyPos = pipeline.getSkyPos();
        if (!allianceColorisRed) {
            return (skyPos == 0 ? 2 : (skyPos == 2) ? 0 : 1);
        }
        return skyPos;
    }
}