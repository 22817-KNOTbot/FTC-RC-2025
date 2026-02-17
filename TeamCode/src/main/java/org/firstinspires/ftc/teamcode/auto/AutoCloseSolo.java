package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.BlueAlliance;
import org.firstinspires.ftc.teamcode.util.RedAlliance;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.AutoAction;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.AutoState;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.StartAutoState;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

@Config
@Configurable
@Autonomous
public class AutoCloseSolo extends LinearOpMode {
	private Alliance alliance;
	private StartAutoState startingState;

	private final Alliance[] ALLIANCES = new Alliance[] {
			new RedAlliance(),
			new BlueAlliance(),
	};

	private AutoManager autoManager;
	private TelemetryManager telemetryManager;

	@Override
	public void runOpMode() {
		telemetryManager = new TelemetryManager();
		telemetryManager.setFtcTelemetry(telemetry);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		startingState = autoManager.getStartingStates()[1];

		boolean configuring = true;
		int selectedIndex = 0;
		while (configuring) {
			if (isStopRequested())
				return;

			String[] options;
			options = Stream.of(ALLIANCES)
					.map(Alliance::getColourString).toArray(String[]::new);

			if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
				selectedIndex = Math.min(selectedIndex + 1, ALLIANCES.length - 1);
			} else if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
				selectedIndex = Math.max(selectedIndex - 1, 0);
			} else if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
				alliance = ALLIANCES[selectedIndex];
				autoManager = new AutoManager(alliance);
				selectedIndex = 0;
				configuring = false;
				break;
			}

			telemetryManager.addData("Alliance", alliance != null ? alliance.getColourString() : "None");
			telemetryManager.addData("Starting State", startingState != null ? startingState.getNameString() : "None");

			telemetryManager.addLine();
			telemetryManager.addLine("====================");
			telemetryManager.addLine();

			for (int i = 0; i < options.length; i++) {
				telemetryManager.addLine((selectedIndex == i ? "> " : "") + options[i]);
			}
			telemetryManager.update();

		}

		selectedIndex = 0;
		AutoState currentState = startingState;
		List<AutoAction> autoActions = new ArrayList<>();

		if (isStopRequested())
			return;

		AutoAction[] actionOptions = currentState.getAutoActions();

		for (int i = 0; i < actionOptions.length; i++) {
			autoActions.add(actionOptions[i]);
			if (i == 2) {
				autoActions.add(actionOptions[2]);
			}
			currentState = actionOptions[i].getResultingState();
			if (currentState == null) {
				break;
			}
			actionOptions = currentState.getAutoActions();
		}

		autoManager.initialize(hardwareMap, startingState, autoActions);

		while (opModeInInit()) {
			telemetryManager.addLine("Auto configuration complete");
			telemetryManager.addLine("====================");

			String[] breadcrumbsList = autoActions.stream()
					.map(AutoAction::getNameString).toArray(String[]::new);
			String breadcrumbsString = String.join(" > ", breadcrumbsList);
			telemetryManager.addLine(breadcrumbsString);
			telemetryManager.update();
		}

		autoManager.start();

		while (opModeIsActive()) {
			autoManager.update();

			autoManager.showTelemetry(telemetryManager);
			// autoManager.showAutomationsTelemetry(telemetryManager);
			autoManager.showFollowerTelemetry(telemetryManager);
			telemetryManager.update();
		}

		autoManager.end();
	}
}
