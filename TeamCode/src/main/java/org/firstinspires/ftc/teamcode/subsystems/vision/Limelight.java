package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.scoring.Artifact.Pattern;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
@Configurable
public class Limelight {
	public static int PIPELINE = 1;
	public static int maxFps = 30;

	private Limelight3A limelight;
	private Integer targetAprilTagId;

	private Pattern pattern = null;

	public Limelight(HardwareMap hardwareMap) {
		this(hardwareMap, null);
	}

	public Limelight(HardwareMap hardwareMap, Integer targetAprilTagId) {
		this(hardwareMap, targetAprilTagId, false);
	}

	public Limelight(HardwareMap hardwareMap, Integer targetAprilTagId, boolean debug) {
		limelight = hardwareMap.get(Limelight3A.class, "Ethernet Device");
		limelight.setPollRateHz(100);
		limelight.pipelineSwitch(PIPELINE);
		limelight.start();

		this.targetAprilTagId = targetAprilTagId;

		if (debug) {
			FtcDashboard.getInstance().startCameraStream(limelight, maxFps);
		}
	}

	public class AlignmentDirection {
		public boolean directionKnown;
		public Double bearing;
		public Double y;

		public AlignmentDirection(boolean directionKnown, Double bearing, Double y) {
			this.directionKnown = directionKnown;
			this.bearing = bearing;
			this.y = y;
		}

		@Override
		public String toString() {
			return "AlignmentDirection [directionKnown=" + directionKnown + ", bearing=" + bearing + ", y="
					+ y + "]";
		}
	}

	public AlignmentDirection getAlignmentDirection() {
		LLResult result = limelight.getLatestResult();
		if (result != null && result.isValid()) {
			for (FiducialResult fiducialResult : result.getFiducialResults()) {
				if (fiducialResult.getFiducialId() == targetAprilTagId || targetAprilTagId == null) {
					return new AlignmentDirection(true, fiducialResult.getTargetXDegrees(),
							fiducialResult.getTargetYDegrees());
				}
			}
		}
		return new AlignmentDirection(false, null, null);
	}

	public void setTargetAprilTagId(Integer targetAprilTagId) {
		this.targetAprilTagId = targetAprilTagId;
	}

	public Pattern updateMotifPattern() {
		Pattern pattern = null;
		LLResult result = limelight.getLatestResult();
		if (result != null && result.isValid()) {
			for (FiducialResult fiducialResult : result.getFiducialResults()) {
				switch (fiducialResult.getFiducialId()) {
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
			}
		}

		if (pattern != null) {
			this.pattern = pattern;
		}
		return pattern;
	}

	public Pattern getLastMotifPattern() {
		return pattern;
	}

	public Pose getPoseMT1(double heading) {
		limelight.updateRobotOrientation(heading);

		LLResult result = limelight.getLatestResult();
		if (result != null && result.isValid()) {
			Pose3D botpose_mt1 = result.getBotpose();
			if (botpose_mt1 == null) {
				return null;
			}
			double x = botpose_mt1.getPosition().x / DistanceUnit.mPerInch;
			double y = botpose_mt1.getPosition().y / DistanceUnit.mPerInch;
			Pose pedroPose = new Pose(x + (72 - -100), y + (144 - 57),
					botpose_mt1.getOrientation().getYaw(AngleUnit.RADIANS));
			return pedroPose;
		}
		return null;
	}

	public Pose getPoseMT2(double heading) {
		limelight.updateRobotOrientation(heading);

		LLResult result = limelight.getLatestResult();
		if (result != null && result.isValid()) {
			Pose3D botpose_mt2 = result.getBotpose_MT2();
			if (botpose_mt2 == null) {
				return null;
			}
			double x = botpose_mt2.getPosition().x / DistanceUnit.mPerInch;
			double y = botpose_mt2.getPosition().y / DistanceUnit.mPerInch;
			Pose pedroPose = new Pose(x + (72 - -100), y + (144 - 57),
					botpose_mt2.getOrientation().getYaw(AngleUnit.RADIANS));
			return pedroPose;
		}
		return null;
	}
}
