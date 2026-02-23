package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Configurable
@Config
public class ColourTesting extends LinearOpMode {
	public static String COLOUR_SENSOR = "colourSensor";
	public static double DISTANCE = 10;

	@Override
	public void runOpMode() {
		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcFastTelemetry(this);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());
		ColorRangeSensor colourRangeSensor = hardwareMap.get(ColorRangeSensor.class, COLOUR_SENSOR);
		
		waitForStart();

		while (opModeIsActive()) {
			NormalizedRGBA colours = colourRangeSensor.getNormalizedColors();
			double red = colours.red;
			double green = colours.green;
			double blue = colours.blue;

			telemetryManager.addData("Distance", colourRangeSensor.getDistance(DistanceUnit.MM));
			telemetryManager.addData("Red", red);
			telemetryManager.addData("Green", green);
			telemetryManager.addData("Blue", blue);
			telemetryManager.addData("==================","");
			telemetryManager.addData("Loaded", colourRangeSensor.getDistance(DistanceUnit.MM) < DISTANCE);
			telemetryManager.addData("Artifact Purple", red < green && green < blue && blue > red);
			telemetryManager.addData("Artifact Green", red < green && green > blue && blue > red && green > 0.008 && blue > 0.008);
			telemetryManager.update();
		}
	}
}