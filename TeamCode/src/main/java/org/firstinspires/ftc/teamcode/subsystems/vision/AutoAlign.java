package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.bylazar.configurables.annotations.Configurable;

import java.util.List;

import com.acmerobotics.dashboard.config.Config;

@Configurable
@Config
public class AutoAlign {
	public static int targetX = 320;
	public static int targetY = 240;
	public static int rangeX = 160;
	public static int rangeY = 120;
	public static int toleranceX = 10;
	public static int toleranceY = 10;

	private boolean DEBUG = false;

	private AprilTagProcessor aprilTagProcessor;
	private Integer aprilTagId;

	public class AlignmentDirection {
		public boolean directionKnown;
		public Float x;
		public Float bearing;
		public Float y;

		public AlignmentDirection(boolean directionKnown, Float x, Float bearing, Float y) {
			this.directionKnown = directionKnown;
			this.x = x;
			this.bearing = bearing;
			this.y = y;
		}

		@Override
		public String toString() {
			return "AlignmentDirection [directionKnown=" + directionKnown + ", x=" + x + ", bearing=" + bearing + ", y="
					+ y + "]";
		}
	}

	public AutoAlign(AprilTagProcessor aprilTagProcessor, Integer aprilTagId) {
		this(aprilTagProcessor, aprilTagId, false);
	}

	public AutoAlign(AprilTagProcessor aprilTagProcessor, Integer aprilTagId, boolean DEBUG) {
		this.aprilTagProcessor = aprilTagProcessor;
		this.aprilTagId = aprilTagId;
		this.DEBUG = DEBUG;
	}

	public AlignmentDirection getAlignmentDirection() {
		AprilTagDetection targetedAprilTag = null;

		List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
		for (AprilTagDetection detection : detections) {
			if (aprilTagId == null || detection.id == aprilTagId) {
				targetedAprilTag = detection;
				break;
			}
		}

		if (targetedAprilTag == null) {
			return new AlignmentDirection(false, null, null, null);
		}

		float distX = (float) targetedAprilTag.center.x - targetX;
		float distY = targetY - (float) targetedAprilTag.center.y;

		distX = Math.abs(distX) < toleranceX ? 0 : distX;
		distY = Math.abs(distY) < toleranceY ? 0 : distY;

		float speedX = Math.max(-1, Math.min(distX / rangeX, 1));
		float speedY = Math.max(-1, Math.min(distY / rangeY, 1));

		float bearing = (float) targetedAprilTag.ftcPose.bearing;

		return new AlignmentDirection(true, speedX, bearing, speedY);
	}

	public void setAprilTagId(Integer aprilTagId) {
		this.aprilTagId = aprilTagId;
	}
}