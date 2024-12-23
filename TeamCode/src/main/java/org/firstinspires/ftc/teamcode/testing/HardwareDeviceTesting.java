package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareDevice;

@Disabled
@Config
@TeleOp(name="Hardare Device Testing", group="Debug")
public class HardwareDeviceTesting extends LinearOpMode {
	public static String DEVICE_NAME = "testDevice"; 

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		HardwareDevice testDevice = hardwareMap.get(HardwareDevice.class, DEVICE_NAME);

		waitForStart();

		while (opModeIsActive()) {
			telemetry.addData("Output", testDevice.getConnectionInfo());
			telemetry.update();
			sleep(50);
		}
	}
}