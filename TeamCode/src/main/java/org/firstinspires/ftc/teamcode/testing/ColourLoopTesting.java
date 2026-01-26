package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

// @Disabled
@Configurable
@Config
// @TeleOp(name="Color loop testing", group="Debug")
public class ColourLoopTesting extends LinearOpMode {
	public static int TELEMETRY_COUNT = 100;
	public static int colourCacheTimeMs = 100;

	private ColorRangeSensor colourSensor;

	private NormalizedRGBA cachedColours;
	private long lastColourUpdateTime;

	@Override
	public void runOpMode() {
		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcTelemetry(telemetry);
		// telemetryManager.setFtcFastTelemetry(this);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		ElapsedTime loopTimer = new ElapsedTime();

		colourSensor = hardwareMap.get(ColorRangeSensor.class, "colourSensor");
		
		waitForStart();

		while (opModeIsActive()) {
			double loopTime = loopTimer.milliseconds();
			loopTimer.reset();

			telemetryManager.addData("Loop Time (ms)", loopTime);
			telemetryManager.addData("Loop Speed (hz)", 1000 / loopTime);

			telemetryManager.addData("Red", getRed());
			telemetryManager.addData("Blue", getBlue());
			telemetryManager.addData("Green", getGreen());

			telemetryManager.update();
		}
	}

	public NormalizedRGBA getColours() {
		long currentTime = System.currentTimeMillis();
		if (currentTime - lastColourUpdateTime > colourCacheTimeMs) {
			lastColourUpdateTime = currentTime;
			cachedColours = colourSensor.getNormalizedColors();		
		}
		return cachedColours;
	}

	public float getRed() {
		return getColours().red;
	}

	public float getGreen() {
		return getColours().green;
	}

	public float getBlue() {
		return getColours().blue;
	}
}