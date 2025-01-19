package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
@Config
// @TeleOp(name="Touch sensor testing", group="Debug")
public class TouchSensorTesting extends LinearOpMode {
	public static String TOUCH_SENSOR_NAME = "intakeTouch";
	public static String DIGITAL_CHANNEL_NAME = "intakeTouch";
	public static Sensors SENSOR = Sensors.TOUCH_SENSOR;

	public enum Sensors {
		TOUCH_SENSOR,
		DIGITAL_CHANNEL
	}

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		TouchSensor touch = null;
		DigitalChannel digital = null;

		switch (SENSOR) {
			case TOUCH_SENSOR:
				touch = hardwareMap.get(TouchSensor.class, TOUCH_SENSOR_NAME);
				break;
			case DIGITAL_CHANNEL:
				digital = hardwareMap.get(DigitalChannel.class, TOUCH_SENSOR_NAME);
				break;
		}
		waitForStart();

		while (opModeIsActive()) {
			telemetry.addData("Touch", (touch != null) ? touch.isPressed() : "None");
			telemetry.addData("Digital", (digital != null) ? !digital.getState() : "None");
			telemetry.update();

			sleep(50);
		}
	}
}