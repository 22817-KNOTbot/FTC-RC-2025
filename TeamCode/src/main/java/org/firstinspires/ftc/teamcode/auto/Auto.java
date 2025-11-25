package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.TelemetryManager;
import org.firstinspires.ftc.teamcode.auto.AutoConfiguration.AutoAction;
import org.firstinspires.ftc.teamcode.auto.AutoConfiguration.AutoState;

import java.util.stream.Stream;

@Config
@Configurable
@Autonomous
public class Auto extends LinearOpMode {
	public static AutoState startingState = new AutoConfiguration.StartState();

	private TelemetryManager telemetryManager;

	@Override
	public void runOpMode() {
		telemetryManager = new TelemetryManager();
		telemetryManager.setFtcTelemetry(telemetry);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		AutoState currentState = startingState;

		while (opModeInInit()) {
			String[] parentStringList = Stream.of(currentState.getParents())
					.map(AutoState::getNameString).toArray(String[]::new);
			String parentString = String.join(" > ", parentStringList);
			telemetryManager.addLine(parentString);

			telemetry.addLine();
			telemetry.addLine("====================");
			telemetry.addLine();

			for (AutoAction action : currentState.getAutoActions()) {
				telemetry.addLine(action.getNameString());
			}
			telemetryManager.update();
		}
	}
}
