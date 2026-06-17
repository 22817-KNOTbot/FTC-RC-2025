package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.BlueAlliance;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.RedAlliance;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

import com.acmerobotics.dashboard.FtcDashboard;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.configurables.annotations.Configurable;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.acmerobotics.dashboard.config.Config;

@Configurable
@Config
@Disabled
public class AutoAlignTesting extends LinearOpMode {
	public static double manualRobotPoseX = 0;
	public static double manualRobotPoseY = 0;
	public static double manualRobotPoseHeading = 0;
	public static Localization localization = Localization.ODOMETRY;
	public static Target target = Target.RED;
	public static double manualTargetPoseX = 0;
	public static double manualTargetPoseY = 0;
	public static double manualTargetPoseHeading = 0;

	private static enum Localization {
		ODOMETRY,
		MANUAL
	}

	private static enum Target {
		RED,
		BLUE,
		MANUAL
	}

	private static Alliance RED_ALLIANCE = new RedAlliance();
	private static Alliance BLUE_ALLIANCE = new BlueAlliance();

	private Pose manualRobotPose;
	private Pose manualTargetPose;

	private Turret turret;
	private Follower follower;

	@Override
	public void runOpMode() {
		final Localization LOCALIZATION = localization;
		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcTelemetry(telemetry);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		turret = new Turret(hardwareMap);

		manualRobotPose = new Pose(manualRobotPoseX, manualRobotPoseY, Math.toRadians(manualRobotPoseHeading));
		manualTargetPose = new Pose(manualTargetPoseX, manualTargetPoseY, Math.toRadians(manualTargetPoseHeading));
		if (LOCALIZATION == Localization.ODOMETRY) {
			follower = Constants.createFollower(hardwareMap);
			follower.setStartingPose(manualRobotPose);
			// follower.setPose(manualRobotPose);
		}

		Drawing.init();

		waitForStart();

		if (LOCALIZATION == Localization.ODOMETRY) {
			follower.startTeleopDrive();
			follower.update();
		}

		while (opModeIsActive()) {
			manualRobotPose = new Pose(manualRobotPoseX, manualRobotPoseY, Math.toRadians(manualRobotPoseHeading));
			manualTargetPose = new Pose(manualTargetPoseX, manualTargetPoseY, Math.toRadians(manualTargetPoseHeading));

			if (LOCALIZATION == Localization.ODOMETRY) {
				follower.update();
				follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
			}
			Pose robotPose;
			switch (LOCALIZATION) {
				case ODOMETRY:
					robotPose = follower.getPose();
					break;
				case MANUAL:
				default:
					robotPose = manualRobotPose;
					break;
			}

			Pose targetPose;
			switch (target) {
				case RED:
					targetPose = RED_ALLIANCE.getGoalPose();
					break;
				case BLUE:
					targetPose = BLUE_ALLIANCE.getGoalPose();
					break;
				case MANUAL:
				default:
					targetPose = manualTargetPose;
					break;
			}

			Pose poseDifference = targetPose.minus(robotPose);

			// Converting to normal coordinate system where
			// 0 = up, increases clockwise; In radians
			double robotAngle = (0.5 * Math.PI) - robotPose.getHeading();
			robotAngle = robotAngle % (2 * Math.PI);
			double targetAngle = Math.atan2(poseDifference.getX(), poseDifference.getY());

			double angleDifference = targetAngle - robotAngle;
			double normalizedAngle = angleDifference - (Math.ceil((angleDifference + Math.PI) / (2 * Math.PI)) - 1)
					* 2 * Math.PI;

			double targetRotation = Turret.BASE_ROTATION + Math.toDegrees(normalizedAngle) * Turret.rotation_per_deg;
			turret.setRotation(targetRotation);

			Drawing.drawRobot(robotPose);
			Drawing.sendPacket();

			telemetryManager.addData("Pose", robotPose);
			telemetryManager.addData("Pose Difference", poseDifference);
			telemetryManager.addData("Robot angle", Math.toDegrees(robotAngle));
			telemetryManager.addData("Target angle", Math.toDegrees(targetAngle));
			telemetryManager.addData("Angle difference", Math.toDegrees(angleDifference));
			telemetryManager.addData("Target rotation", targetRotation);
			telemetryManager.update();
		}

	}
}
