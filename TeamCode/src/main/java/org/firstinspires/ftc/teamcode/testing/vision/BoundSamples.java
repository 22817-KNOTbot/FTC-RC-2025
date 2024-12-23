package org.firstinspires.ftc.teamcode.testing.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@Autonomous(name = "Bound Samples", group = "Testing")
public class BoundSamples extends LinearOpMode {
    @Override
    public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onError(int error) {}
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

		BoundSamplesPipeline pipeline = new BoundSamplesPipeline();

        camera.setPipeline(pipeline);

        waitForStart();
        
        while (opModeIsActive()) {
            telemetry.addData("Y", pipeline.get_y());
            telemetry.addData("Cr", pipeline.get_Cr());
            telemetry.addData("Cb", pipeline.get_Cb());
            telemetry.update();
            sleep(50);
        }
    }
}