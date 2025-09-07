package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Vision {
	public static boolean DEBUG = false;

	private VisionPortal visionPortal;
	private AprilTagProcessor aprilTagProcessor;

	private AutoAlign autoAlignProcessor;

	public Vision(HardwareMap hardwareMap) {
		WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

		autoAlignProcessor = new AutoAlign(webcam, null, DEBUG);

		visionPortal = new VisionPortal.Builder()
				.setCamera(webcam)
				.enableLiveView(DEBUG)
				.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
				.addProcessor(autoAlignProcessor.getProcessor())
				.build();
	}

	public AutoAlign.AlignmentDirection getAlignmentDirection() {
		return autoAlignProcessor.getAlignmentDirection();
	}

	public void setAprilTagId(Integer aprilTagId) {
		autoAlignProcessor.setAprilTagId(aprilTagId);
	}

	public void setAprilTagProcessorEnabled(boolean enabled) {
		visionPortal.setProcessorEnabled(aprilTagProcessor, enabled);
	}

	public void setAllProcessorsEnabled(boolean enabled) {
		setAprilTagProcessorEnabled(enabled);
	}
	
	public void close() {
		visionPortal.close();
	}

	public void showTelemetry(Telemetry telemetry) {
		autoAlignProcessor.showTelemetry(telemetry);
	}
}
