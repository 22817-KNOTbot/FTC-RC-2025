package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.gamepad.PanelsGamepad;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.BlueAlliance;
import org.firstinspires.ftc.teamcode.util.GamepadManager;
import org.firstinspires.ftc.teamcode.util.RedAlliance;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.AutoAction;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.DirectIntakeMiddleAction;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.DirectIntakeTopAction;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.GateIntakeAction;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.IntakeBottomSidespikeAction;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.LeaveUpAction;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.ShootCloseAction;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.ShootCloseFromStartAction;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.ShootUnsortedAction;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.StartAutoState;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

@Config
@Configurable
@Autonomous
public class AutoCloseSolo extends LinearOpMode {
	public static final List<Class<? extends AutoAction>> AUTO_ACTIONS = List.of(
		ShootCloseFromStartAction.class,
		ShootUnsortedAction.class,

		DirectIntakeMiddleAction.class,
		ShootCloseAction.class,
		ShootUnsortedAction.class,

		GateIntakeAction.class,
		ShootCloseAction.class,
		ShootUnsortedAction.class,

		GateIntakeAction.class,
		ShootCloseAction.class,
		ShootUnsortedAction.class,

		GateIntakeAction.class,
		ShootCloseAction.class,
		ShootUnsortedAction.class,

		DirectIntakeTopAction.class,
		ShootCloseAction.class,
		ShootUnsortedAction.class,

		IntakeBottomSidespikeAction.class,
		ShootCloseAction.class,
		ShootUnsortedAction.class,

		LeaveUpAction.class
	);
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

		GamepadManager gamepadManager = new GamepadManager(gamepad1, gamepad2,
			PanelsGamepad.INSTANCE.getFirstManager()::getAsFTCGamepad,
			PanelsGamepad.INSTANCE.getSecondManager()::getAsFTCGamepad);
		gamepadManager.updateGamepads();
		Gamepad customGamepad1 = gamepadManager.getGamepad1();
		Gamepad customGamepad2 = gamepadManager.getGamepad2();

		boolean configuring = true;
		int selectedIndex = 0;
		while (configuring) {
			if (isStopRequested())
				return;

			String[] options;
			options = Stream.of(ALLIANCES)
					.map(Alliance::getColourString).toArray(String[]::new);

			if (customGamepad1.dpadDownWasPressed() || customGamepad2.dpadDownWasPressed()) {
				selectedIndex = Math.min(selectedIndex + 1, ALLIANCES.length - 1);
			} else if (customGamepad1.dpadUpWasPressed() || customGamepad2.dpadUpWasPressed()) {
				selectedIndex = Math.max(selectedIndex - 1, 0);
			} else if (customGamepad1.aWasPressed() || customGamepad2.aWasPressed()) {
				alliance = ALLIANCES[selectedIndex];
				autoManager = new AutoManager(alliance);
				startingState = autoManager.getStartingStates()[1];
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

		List<Class<? extends AutoAction>> desiredAutoActions = new ArrayList<>(AUTO_ACTIONS);
		List<AutoAction> autoActions = AutoManager.getAutoActionsByClasses(startingState, desiredAutoActions);

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
			autoManager.showAutomationsTelemetry(telemetryManager);
			autoManager.showFollowerTelemetry(telemetryManager);
			telemetryManager.update();
		}

		autoManager.end();
		
	}
}
