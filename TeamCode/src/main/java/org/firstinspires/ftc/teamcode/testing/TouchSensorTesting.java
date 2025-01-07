package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name="Touch sensor testing", group="Debug")
public class TouchSensorTesting extends LinearOpMode {
	public static String TOUCH_SENSOR_NAME = "intakeTouch";

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		TouchSensor touch = hardwareMap.get(TouchSensor.class, TOUCH_SENSOR_NAME);
		DigitalChannel digital = hardwareMap.get(DigitalChannel.class, TOUCH_SENSOR_NAME);
		waitForStart();

		while (opModeIsActive()) {
			telemetry.addData("Touch", touch.isPressed());
			telemetry.addData("Digital", !digital.getState());
			telemetry.update();

			sleep(50);
		}
	}
}