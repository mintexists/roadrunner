package org.firstinspires.ftc.teamcode.auto;

import android.opengl.Matrix;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "CVVVVV")
public class ConceptOpenCV extends LinearOpMode {

    int spike;
    int cam;

    class conceptPipeline extends OpenCvPipeline {

        Mat mat = new Mat();

        @Override
        public Mat processFrame(Mat input) {

            Mat mask = new Mat();

            Imgproc.filter2D(input, mat, 1, new Mat());

            return mat;
        }
    }

    OpenCvCamera camera;
    conceptPipeline pipeline = new conceptPipeline();

    public void cameraInit() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");

        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

                camera.setPipeline(pipeline);
            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addAction(() -> {
                    telemetry.addData("CAMERA ERROR", "");
                });
                telemetry.update();
            }
        });
    }
    @Override
    public void runOpMode() throws InterruptedException {

        cameraInit();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("HIII", "hiii");
                telemetry.update();
            }
        }

    }
}
