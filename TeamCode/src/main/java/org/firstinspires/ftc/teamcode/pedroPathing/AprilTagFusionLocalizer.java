package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;

import android.util.Log;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.vision.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.vision.Limelight.PoseUpdate;

@Config
@Configurable
public class AprilTagFusionLocalizer {
	public static Pose initialCovariance = new Pose(0.25, 0.25, Math.toRadians(2));
	public static Pose processVariance = new Pose(1, 1, Math.toRadians(0.5) / 60);
	public static Pose measurementVariance = new Pose(2.1561, 2.6065, 0.0248);

	private FusionLocalizer fusion;
	private Limelight limelight;

	public AprilTagFusionLocalizer(Localizer localizer, Limelight limelight) {
		fusion = new FusionLocalizer(
				localizer,
				initialCovariance,
				processVariance,
				measurementVariance,
				100);

		this.limelight = limelight;
	}

	public Localizer getLocalizer() {
		return fusion;
	}

	public void update() {
		update(false);
	}

	public void update(boolean log) {
		PoseUpdate limelightPoseUpdate = limelight.getPoseUpdateMT2(Math.toDegrees(fusion.getPose().getHeading()));
		if (limelightPoseUpdate != null) {
			Pose limelightPose = limelightPoseUpdate.pose;
			long timestampMs = limelightPoseUpdate.timestamp;
			long latencyMs = System.currentTimeMillis() - timestampMs;
			long timestampNs = System.nanoTime() - (latencyMs * 1_000_000);
	
			fusion.addMeasurement(limelightPose, timestampNs);
			if (log) {
				Log.v("AprilTagFusionLocalizer", "Added measurement: " + limelightPose.toString());
				Pose ftcPose = limelightPose.getAsCoordinateSystem(InvertedFTCCoordinates.INSTANCE);
				Pose2d wpiPose = new Pose2d(DistanceUnit.INCH.toMeters(ftcPose.getX()),
						DistanceUnit.INCH.toMeters(ftcPose.getY()), Rotation2d.fromRadians(ftcPose.getHeading()));
				Logger.recordOutput("AprilTagLocalizer Measurement", wpiPose);
			}
		}
	}
}