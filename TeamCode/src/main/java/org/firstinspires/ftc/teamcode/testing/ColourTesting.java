package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

@Config
@TeleOp(name="Color testing", group="Debug")
public class ColourTesting extends LinearOpMode {
    public static double COLOUR_THRESHOLD_1 = 2.5;
    public static double COLOUR_THRESHOLD_2 = 3;
    public static double COLOUR_THRESHOLD_3 = 1.6;
    public static double COLOUR_THRESHOLD_4 = 2;
    public static double COLOUR_THRESHOLD_5 = 3.5;
    public static double COLOUR_THRESHOLD_6 = 1.2;

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
			telemetry.addData("Yellow", (red/blue>COLOUR_THRESHOLD_1) && (green/blue>COLOUR_THRESHOLD_2));
			telemetry.addData("Red", (red/green>COLOUR_THRESHOLD_3) && (red/blue>COLOUR_THRESHOLD_4));
			telemetry.addData("Blue", (blue/red>COLOUR_THRESHOLD_5) && (blue/green>COLOUR_THRESHOLD_6));
			telemetry.update();
		}
	}
}