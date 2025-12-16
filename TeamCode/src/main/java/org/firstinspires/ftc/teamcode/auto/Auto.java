package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RedAlliance;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.AutoAction;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.AutoState;

import java.util.stream.Stream;

@Config
@Configurable
@Autonomous
public class Auto extends LinearOpMode {
	public static Alliance alliance = new RedAlliance(); // TODO: make a separate configuration input for this
	public static AutoState startingState;

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

		while (opModeInInit() && configuring) {
			String[] parentStringList = Stream.of(currentState.getParents())
					.map(AutoState::getNameString).toArray(String[]::new);
			String parentString = String.join(" > ", parentStringList);
			telemetryManager.addLine(parentString);

			telemetry.addLine();
			telemetry.addLine("====================");
			telemetry.addLine();

			AutoAction[] actionOptions = currentState.getAutoActions();
			for (int i = 0; i < actionOptions.length; i++) {
				telemetry.addLine(selectedIndex == i ? "> " : "" + actionOptions[i].getNameString());
			}
			telemetryManager.update();
		}
	}
}
