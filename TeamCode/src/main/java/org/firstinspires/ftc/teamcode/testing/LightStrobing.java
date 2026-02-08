package org.firstinspires.ftc.teamcode.testing;

import org.firstinspires.ftc.teamcode.util.TelemetryManager;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@Config
public class LightStrobing extends LinearOpMode {
	public static String lightName = "light";
	public static double min = 0.25;
	public static double max = 0.75;
	public static double increment = 0.003;
	public static int delayMs = 10;

	private double current = min;
	private int direction = 1;

	@Override
	public void runOpMode() {
		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcTelemetry(telemetry);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		Servo light = hardwareMap.get(Servo.class, lightName);

		waitForStart();

		while (opModeIsActive()) {
			light.setPosition(current);
			current = current + (increment * direction);
			if (current > max) {
				current = max;
				direction = -1;
			} else if (current < min) {
				current = min;
				direction = 1;
			}
			
			telemetryManager.addData("Current", current);
			telemetryManager.update();
			
			sleep(delayMs);
		}
	}
}
