package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.teamcode.scoring.Artifact.Pattern;

import java.util.List;

public class MotifDecode {
	private boolean DEBUG = false;

	private AprilTagProcessor aprilTagProcessor;

	private Pattern pattern = null;
	private Integer prioritySide = null;

	public MotifDecode(AprilTagProcessor aprilTagProcessor) {
		this(aprilTagProcessor, false);
	}

	public MotifDecode(AprilTagProcessor aprilTagProcessor, boolean DEBUG) {
		this.aprilTagProcessor = aprilTagProcessor;
		this.DEBUG = DEBUG;
	}

	// Only returns pattern it currently sees
	// Use getLastPattern() to get stored pattern
	public Pattern updatePattern() {
		Pattern pattern = null;
		Double obtainedPatternPosition = null;

		List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
		for (AprilTagDetection detection : detections) {
			switch (detection.id) {
				case 21:
					pattern = Pattern.GPP;
					break;
				case 22:
					pattern = Pattern.PGP;
					break;
				case 23:
					pattern = Pattern.PPG;
					break;
			}
			if (pattern != null) {
				if (prioritySide == null) {
					break;
				}
				if (obtainedPatternPosition == null || Math.signum(detection.center.x - obtainedPatternPosition) == Math.signum(prioritySide)) {
					obtainedPatternPosition = detection.center.x;
				}
			}
		}

		if (pattern != null) {
			this.pattern = pattern;
		}
		return pattern;
	}

	public Pattern getLastPattern() {
		return pattern;
	}

	// If multiple tags visible, this determines priority
	// Negative = left, positive = right
	public void setPrioritySide(Integer side) {
		this.prioritySide = side;
	}
}
