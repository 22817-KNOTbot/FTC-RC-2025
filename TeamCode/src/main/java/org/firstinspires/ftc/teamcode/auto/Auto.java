package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RedAlliance;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.AutoAction;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.AutoState;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.StartAutoState;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

@Config
@Configurable
@Autonomous
public class Auto extends LinearOpMode {
	public static Alliance alliance = new RedAlliance(); // TODO: make a separate configuration input for this
	public static StartAutoState startingState;

	private AutoManager autoManager;
	private TelemetryManager telemetryManager;

	@Override
	public void runOpMode() {
		telemetryManager = new TelemetryManager();
		telemetryManager.setFtcTelemetry(telemetry);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		autoManager = new AutoManager(alliance);
		startingState = autoManager.getStartingStates()[0]; // TODO: make a separate configuration input for this

		boolean configuring = true;
		int selectedIndex = 0;
		AutoState currentState = startingState;
		List<AutoAction> autoActions = new ArrayList<>();

		while (configuring) {
			if (isStopRequested())
				return;

			AutoAction[] actionOptions = currentState.getAutoActions();

			if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
				selectedIndex = Math.min(selectedIndex + 1, actionOptions.length - 1);
			} else if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
				selectedIndex = Math.max(selectedIndex - 1, 0);
			} else if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
				autoActions.add(actionOptions[selectedIndex]);
				currentState = actionOptions[selectedIndex].getResultingState();
				if (currentState == null) {
					configuring = false;
					break;
				}
				actionOptions = currentState.getAutoActions();
			}

			String[] breadcrumbsList = autoActions.stream()
					.map(AutoAction::getNameString).toArray(String[]::new);
			String breadcrumbsString = String.join(" > ", breadcrumbsList);
			telemetryManager.addLine(breadcrumbsString);

			telemetryManager.addLine();
			telemetryManager.addLine("====================");
			telemetryManager.addLine();

			for (int i = 0; i < actionOptions.length; i++) {
				telemetryManager.addLine((selectedIndex == i ? "> " : "") + actionOptions[i].getNameString());
			}
			telemetryManager.update();
		}

		autoManager.initialize(hardwareMap, startingState, autoActions);

		while (opModeInInit()) {
			telemetryManager.addLine("Auto configuration complete");
			telemetryManager.addLine("====================");

			String[] breadcrumbsList = autoActions.stream()
					.map(AutoAction::getNameString).toArray(String[]::new);
			String breadcrumbsString = String.join(" > ", breadcrumbsList);
			telemetryManager.addLine(breadcrumbsString);
		}

		while (opModeIsActive()) {
			autoManager.update();
		}
	}
}
