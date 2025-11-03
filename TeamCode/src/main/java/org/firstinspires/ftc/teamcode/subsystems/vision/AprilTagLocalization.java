package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.scoring.Artifact.Pattern;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

import java.lang.Math;

@Configurable
public class AprilTagLocalization {
	public static boolean DEBUG = false;
	public static int decimation = 3;
	public static Pose turretPos; // determined after measuring?
	public static Pose robotCentre; // determined after measuring?

	private VisionPortal visionPortal;
	private AprilTagProcessor aprilTagProcessor;
	private AprilTagDetection detection;
	private Pose pose;

	private AutoAlign autoAlignProcessor;
	private MotifDecode motifDecodeProcessor;

	private WebcamName webcam;

	private double myX = detection.ftcPose.x;
	private double myY = detection.ftcPose.y;
	private double myZ = detection.ftcPose.z;
	private double myYaw = detection.ftcPose.yaw;

	public AprilTagLocalization(HardwareMap hardwareMap) {
		this(hardwareMap, null);
	}

	public AprilTagLocalization(HardwareMap hardwareMap, Integer targetAprilTagId) {
		WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

		aprilTagProcessor = new AprilTagProcessor.Builder()
				.setDrawAxes(DEBUG)
				.setDrawCubeProjection(DEBUG)
				.build();

		aprilTagProcessor.setDecimation(decimation);

		autoAlignProcessor = new AutoAlign(aprilTagProcessor, targetAprilTagId, DEBUG);
		motifDecodeProcessor = new MotifDecode(aprilTagProcessor, DEBUG);

		visionPortal = new VisionPortal.Builder()
				.setCamera(webcam)
				.enableLiveView(DEBUG)
				.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
				.addProcessor(aprilTagProcessor)
				.build();
		
		pose = new Pose(myX, myY, myYaw);
	}

	public AutoAlign.AlignmentDirection getAlignmentDirection() {
		return autoAlignProcessor.getAlignmentDirection();
	}

	public Pattern updateMotifPattern() {
		return motifDecodeProcessor.updatePattern();
	}

	public Pattern getLastMotifPattern() {
		return motifDecodeProcessor.getLastPattern();
	}

	public void setTargetAprilTagId(Integer aprilTagId) {
		autoAlignProcessor.setAprilTagId(aprilTagId);
	}

	public Pose getPose() {
		myX = detection.ftcPose.x;
		myY = detection.ftcPose.y;
		myYaw = detection.ftcPose.yaw;
		pose = new Pose(myX, myY, myYaw);
		// change pose to center of turret
		// change pose to center of robot
		return pose;
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
		List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
		telemetry.addData("# AprilTags Detected", currentDetections.size());

		for (AprilTagDetection detection : currentDetections) {
			if (detection.metadata != null) {
				telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
				telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x,
						detection.ftcPose.y, detection.ftcPose.z));
				telemetry.addLine(String.format("XYZ %6.1f %6.1f (center px)", detection.center.x,
						detection.center.y));
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
