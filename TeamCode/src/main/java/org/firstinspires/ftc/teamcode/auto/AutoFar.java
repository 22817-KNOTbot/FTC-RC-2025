package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.BlueAlliance;
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
public class AutoFar extends LinearOpMode {
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

		boolean configuring = true;
		int selectedIndex = 0;
		int configuringLevel = 0;
		while (configuring) {
			if (isStopRequested())
				return;

			String[] options;
			switch (configuringLevel) {
				case 0:
					options = Stream.of(ALLIANCES)
							.map(Alliance::getColourString).toArray(String[]::new);
					break;
				case 1:
					options = Stream.of(autoManager.getStartingStates())
							.map(AutoState::getNameString).toArray(String[]::new);
					break;
				default:
					// This should never happen. Simply ending the OpMode is safest if it somehow does
					return;
			}

			if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
				selectedIndex = Math.min(selectedIndex + 1, ALLIANCES.length - 1);
			} else if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
				selectedIndex = Math.max(selectedIndex - 1, 0);
			} else if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
				if (configuringLevel == 0) {
					alliance = ALLIANCES[selectedIndex];
					autoManager = new AutoManager(alliance);
				} else if (configuringLevel == 1) {
					startingState = autoManager.getStartingStates()[selectedIndex];
					configuring = false;
				} else {
					// This should never happen. Simply ending the OpMode is safest if it somehow does
					return; 
				}

				configuringLevel++;
				selectedIndex = 0;
				continue;
			} else if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
				configuringLevel = 0;
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

		configuring = true;
		selectedIndex = 0;
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
				selectedIndex = 0;
			} else if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
				autoActions.remove(autoActions.size() - 1);
				currentState = autoActions.get(autoActions.size() - 1).getResultingState();
				actionOptions = currentState.getAutoActions();
				selectedIndex = 0;
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
