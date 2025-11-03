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
	public static int decimation = 3;
	public static Pose turretPos; // determined after measuring?
	public static Pose robotCentre; // determined after measuring?

	private VisionPortal visionPortal;
	private AprilTagProcessor aprilTagProcessor;
	private AprilTagDetection detection;
	private Pose pose;
	public boolean DEBUG = false;

	private AutoAlign autoAlignProcessor;
	private MotifDecode motifDecodeProcessor;

	private WebcamName webcam;

	public AprilTagLocalization(AprilTagProcessor aprilTagProcessor, Integer aprilTagId, boolean DEBUG) {
		if (!detection.metadata.name.contains("Obelisk")) {
			this.aprilTagProcessor = aprilTagProcessor;
			this.aprilTagId = aprilTagId;
			this.DEBUG = DEBUG;
		}
	}

	public Pose getPose() {
		pose = new Pose(detection.robotPose.getPosition().x,
			detection.robotPose.getPosition().y,
			detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)
		);
		// change pose to center of turret
		// change pose to center of robot
		return pose;
	}
}
