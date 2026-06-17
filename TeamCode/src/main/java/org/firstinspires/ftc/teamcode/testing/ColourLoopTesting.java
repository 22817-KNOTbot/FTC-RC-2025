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

import java.util.LinkedList;
import java.util.Queue;

// @Disabled
@Configurable
@Config
// @TeleOp(name="Color loop testing", group="Debug")
@Disabled
public class ColourLoopTesting extends LinearOpMode {
	public static int AVERAGE_WINDOW_SIZE = 1000;
	public static int colourCacheTimeMs = 10;

	private ColorRangeSensor colourSensor;

	private NormalizedRGBA cachedColours;
	private long lastColourUpdateTime;

	@Override
	public void runOpMode() {
		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcFastTelemetry(this);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		ElapsedTime loopTimer = new ElapsedTime();
		RollingAverageDouble loopTimeAverage = new RollingAverageDouble(AVERAGE_WINDOW_SIZE);

		colourSensor = hardwareMap.get(ColorRangeSensor.class, "colourSensor");

		waitForStart();

		while (opModeIsActive()) {
			double loopTime = loopTimer.milliseconds();
			loopTimer.reset();
			loopTimeAverage.addNumber(loopTime);

			telemetryManager.addData("Loop Time (ms)", loopTime);
			telemetryManager.addData("Loop Speed (hz)", 1000 / loopTime);
			double averageLoopTime = loopTimeAverage.getAverage();
			telemetryManager.addData("Average Loop Time (ms)", averageLoopTime);
			telemetryManager.addData("Average Loop Speed (hz)", 1000 / averageLoopTime);

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

	// Copied from FTC SDK and modified to accept double
	private static class RollingAverageDouble {
		private final Queue<Double> queue = new LinkedList<>();
		private double total;
		private int size;

		public RollingAverageDouble(int size) {
			this.resize(size);
		}

		public int size() {
			return this.size;
		}

		public void resize(int size) {
			this.size = size;
			this.queue.clear();
		}

		public void addNumber(double number) {
			if (this.queue.size() >= this.size) {
				double last = (Double) this.queue.remove();
				this.total -= (double) last;
			}

			this.queue.add(number);
			this.total += (double) number;
		}

		public double getAverage() {
			return this.queue.isEmpty() ? 0 : (double) (this.total / (double) this.queue.size());
		}

		public void reset() {
			this.queue.clear();
		}
	}
}