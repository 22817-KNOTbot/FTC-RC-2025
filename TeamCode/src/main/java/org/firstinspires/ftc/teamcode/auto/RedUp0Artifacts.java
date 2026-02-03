package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.RedAlliance;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

@Configurable
@Autonomous(name = "Upper Red Leave", group = "Autonomous")
public class RedUp0Artifacts extends LinearOpMode {
	public static boolean doActions = true;
	public static boolean DEBUG = false;

	private int pathState = 0;

	private Alliance alliance = new RedAlliance();
	private Follower follower;
	private Pose startPose = new Pose(123.000, 124.000, Math.toRadians(125));

	private PathChain leaveline;

	@Override
	public void runOpMode() {
		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcTelemetry(telemetry);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		blackboard.put("alliance", alliance);
		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(startPose);
		buildPaths();

		waitForStart();

		while (opModeIsActive()) {
			follower.update();
			pathUpdate();

			if (DEBUG) {
				Drawing.drawRobot(follower.getPose(), telemetryManager.getDashboardCanvas());
				Drawing.sendPacket();

				telemetryManager.addData("T value", follower.getCurrentTValue());
				telemetryManager.addData("Path completion", follower.getPathCompletion());

				String[] debugLines = null;
				try {
					debugLines = follower.debug();
				} catch (Exception e) {
					telemetryManager.addLine("Failed to retrieve debug string!");
				}
				if (debugLines != null) {
					for (String line : debugLines) {
						telemetryManager.addLine(line);
					}
				}
			}
			telemetryManager.update();
		}
		blackboard.put("pose", follower.getPose());
	}

	public void buildPaths() {
		leaveline = follower.pathBuilder()
				.addPath(new BezierLine(startPose, new Pose(107.000, 130.000)))
				.setConstantHeadingInterpolation(Math.toRadians(125))
				.build();
	}

	public void setPathState(int state) {
		pathState = state;
	}

	public void pathUpdate() {
		switch (pathState) {
			case 0:
				if (!follower.isBusy()) {
					follower.followPath(leaveline, true);
					setPathState(-1);
				}
		}
	}
}