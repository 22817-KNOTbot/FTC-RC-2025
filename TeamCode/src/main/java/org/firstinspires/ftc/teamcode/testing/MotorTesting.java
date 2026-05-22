package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ManualHardwareManager;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.acmerobotics.dashboard.config.Config;

@Configurable
@Config
// @TeleOp(name="Motor testing", group="Debug")
public class MotorTesting extends LinearOpMode {
	public static double POWER = 0;
	public static int MOTOR_PORT = -1; 
	public static String MOTOR_NAME = "testMotor"; 

	@Override
	public void runOpMode() {
		DcMotor motor;
		if (MOTOR_PORT < 0) {
			motor = hardwareMap.get(DcMotor.class, MOTOR_NAME);
		} else {
			int address = MOTOR_PORT >= 4 ? 2 : 173;
			motor = ManualHardwareManager.getMotor(hardwareMap, MOTOR_PORT % 4, address);
		}

		waitForStart();

		while (opModeIsActive()) {
			motor.setPower(POWER);

			telemetry.addData("Power", POWER);
			telemetry.update();

		}
	}
}