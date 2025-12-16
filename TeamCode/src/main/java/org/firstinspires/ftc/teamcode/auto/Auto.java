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

		while (opModeInInit() && configuring) {
			AutoAction[] actionOptions = currentState.getAutoActions();

			if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
				selectedIndex = Math.min(selectedIndex + 1, actionOptions.length - 1);
			} else if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
				selectedIndex = Math.max(selectedIndex - 1, 0);
			}

			String[] parentStringList = autoActions.stream()
					.map(AutoAction::getNameString).toArray(String[]::new);
			String parentString = String.join(" > ", parentStringList);
			telemetryManager.addLine(parentString);

			telemetry.addLine();
			telemetry.addLine("====================");
			telemetry.addLine();

			for (int i = 0; i < actionOptions.length; i++) {
				telemetry.addLine(selectedIndex == i ? "> " : "" + actionOptions[i].getNameString());
			}
			telemetryManager.update();
		}

		Follower follower = Constants.createFollower(hardwareMap);
		autoActions.add(startingState.getAutoActions()[0]);
		autoActions.add(autoActions.get(autoActions.size() - 1).getResultingState().getAutoActions()[0]);
		autoActions.add(autoActions.get(autoActions.size() - 1).getResultingState().getAutoActions()[0]);
		autoActions.add(autoActions.get(autoActions.size() - 1).getResultingState().getAutoActions()[3]);

		autoManager.initialize(hardwareMap, follower, startingState, autoActions);


	}
}
