package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;

@Disabled
@Config
// @TeleOp(name="Camera Setting", group="Debug")
public class CameraSetting extends LinearOpMode {
	public static int EXPOSURE = 15;
	public static int GAIN = 100;
	public static int ZOOM = 100;

	private VisionPortal visionPortal;

	@Override
	public void runOpMode() {
		visionPortal = new VisionPortal.Builder()
			.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
			.build();

		while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING && !isStopRequested()) {
			sleep(20);
		}

		waitForStart();

		while (opModeIsActive()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
			exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure((long) EXPOSURE, TimeUnit.MILLISECONDS);

			GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(GAIN);

			PtzControl ptzControl = visionPortal.getCameraControl(PtzControl.class);
			ptzControl.setZoom(ZOOM);

			telemetry.addData("MIN", "%d | %d | %d", 
				exposureControl.getMinExposure(TimeUnit.MILLISECONDS),
				gainControl.getMinGain(),
				ptzControl.getMinZoom());
			telemetry.addData("MAX", "%d | %d | %d", 
				exposureControl.getMaxExposure(TimeUnit.MILLISECONDS),
				gainControl.getMaxGain(),
				ptzControl.getMaxZoom());
			telemetry.update();

			sleep(1000);
		}
	}
}
