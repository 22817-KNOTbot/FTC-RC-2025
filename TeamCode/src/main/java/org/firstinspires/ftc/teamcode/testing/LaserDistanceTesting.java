package org.firstinspires.ftc.teamcode.testing;

import org.firstinspires.ftc.teamcode.util.TelemetryManager;

import com.acmerobotics.dashboard.FtcDashboard;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class LaserDistanceTesting extends LinearOpMode {
	public static String sensor = "laserSensor";

	@Override
	public void runOpMode() {
		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcFastTelemetry(this);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		DigitalChannel laserSensor = hardwareMap.get(DigitalChannel.class, sensor);
        laserSensor.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

		while (opModeIsActive()) {
			telemetryManager.addData("Detected", laserSensor.getState());
			telemetryManager.update();
		}
	}
}
