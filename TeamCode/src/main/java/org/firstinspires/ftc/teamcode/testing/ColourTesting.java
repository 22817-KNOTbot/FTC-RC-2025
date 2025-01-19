package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

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
			telemetry.addData("Yellow", (red / blue >= 1) && (green / blue >= 1.6));
			telemetry.addData("Red", (red / green >= 0.7) && (red / blue >= 0.9));
			telemetry.addData("Blue", (blue / red >= 1.8) && (blue / green >= 0.8));
			telemetry.update();
		}
	}
}