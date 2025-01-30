package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;

// @Disabled
@Config
// @TeleOp(name="Color testing", group="Debug")
public class ColourTesting extends LinearOpMode {
    public static double COLOUR_THRESHOLD_1 = 1;
    public static double COLOUR_THRESHOLD_2 = 1.6;
    public static double COLOUR_THRESHOLD_3 = 0.7;
    public static double COLOUR_THRESHOLD_4 = 0.9;
    public static double COLOUR_THRESHOLD_5 = 1.8;
    public static double COLOUR_THRESHOLD_6 = 0.8;

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
		ColorRangeSensor colourRangeSensor = hardwareMap.get(ColorRangeSensor.class, "colorSensor");
		
		waitForStart();

		while (opModeIsActive()) {
			double red = colourRangeSensor.red();
			double green = colourRangeSensor.green();
			double blue = colourRangeSensor.blue();

			telemetry.addData("Distance", colourRangeSensor.getDistance(DistanceUnit.MM)); // 30
			telemetry.addData("Red", red);
			telemetry.addData("Green", green);
			telemetry.addData("Blue", blue);
			telemetry.addData("DIVIDER", "");
			telemetry.addData("Sample Yellow", (red / blue >= COLOUR_THRESHOLD_1) && (green / blue >= COLOUR_THRESHOLD_2));
			telemetry.addData("Sample Red", (red / green >= COLOUR_THRESHOLD_3) && (red / blue >= COLOUR_THRESHOLD_4));
			telemetry.addData("Sample Blue", (blue / red >= COLOUR_THRESHOLD_5) && (blue / green >= COLOUR_THRESHOLD_6));
			telemetry.addData("2 - Sample Yellow", (green > 150) && (green > red) && (red > blue));
			telemetry.addData("2 - Sample Red", (red > green) && (green > blue));
			telemetry.addData("2 - Sample Blue", (blue > green) && (green > red));
			telemetry.update();
		}
	}
}