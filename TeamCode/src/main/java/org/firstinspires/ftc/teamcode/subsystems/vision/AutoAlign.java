package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.bylazar.configurables.annotations.Configurable;

import java.util.List;

@Configurable
public class AutoAlign {
	public static int decimation = 3;
	public static int targetX = 160;
	public static int targetY = 120;
	public static int rangeX = 160;
	public static int rangeY = 120;

	private boolean DEBUG = false;

	private Integer aprilTagId;

	private WebcamName webcam;
	private AprilTagProcessor aprilTagProcessor;

	public class AlignmentDirection {
		public boolean directionKnown;
		public Float x;
		public Float y;

		public AlignmentDirection(boolean directionKnown, Float x, Float y) {
			this.directionKnown = directionKnown;
			this.x = x;
			this.y = y;
		}

		@Override
		public String toString() {
			return "AlignmentDirection [directionKnown=" + directionKnown + ", x=" + x + ", y=" + y + "]";
		}
	}

	public AutoAlign(WebcamName webcam, Integer aprilTagId) {
		this(webcam, aprilTagId, false);
	}

	public AutoAlign(WebcamName webcam, Integer aprilTagId, boolean DEBUG) {
		this.webcam = webcam;
		this.aprilTagId = aprilTagId;
		this.DEBUG = DEBUG;

		aprilTagProcessor = new AprilTagProcessor.Builder()
				.setDrawAxes(DEBUG)
				.setDrawCubeProjection(DEBUG)
				.build();

		aprilTagProcessor.setDecimation(decimation);
	}

	public AprilTagProcessor getProcessor() {
		return aprilTagProcessor;
	}

	public AlignmentDirection getAlignmentDirection() {
		AprilTagDetection targetedAprilTag = null;

		List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
		for (AprilTagDetection detection : detections) {
			if (aprilTagId == null) {
				targetedAprilTag = detection;
				break;
			}
			if (detection.id == aprilTagId) {
				targetedAprilTag = detection;
				break;
			}
		}

		if (targetedAprilTag == null) {
			return new AlignmentDirection(false, null, null);
		}

		float distX = (float) targetedAprilTag.center.x - targetX;
		float distY = targetY - (float) targetedAprilTag.center.y;

		float speedX = Math.min(distX / rangeX, 1);
		float speedY = Math.min(distY / rangeY, 1);

		return new AlignmentDirection(true, speedX, speedY);
	}

	public void setAprilTagId(Integer aprilTagId) {
		this.aprilTagId = aprilTagId;
	}

	public void showTelemetry(Telemetry telemetry) {
		List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
		telemetry.addData("# AprilTags Detected", currentDetections.size());

		for (AprilTagDetection detection : currentDetections) {
			if (detection.metadata != null) {
				telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
				telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x,
						detection.ftcPose.y, detection.ftcPose.z));
				telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch,
						detection.ftcPose.roll, detection.ftcPose.yaw));
				telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range,
						detection.ftcPose.bearing, detection.ftcPose.elevation));
			} else {
				telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
				telemetry.addLine(
						String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
			}
		}

		telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
		telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
		telemetry.addLine("RBE = Range, Bearing & Elevation");
	}
}