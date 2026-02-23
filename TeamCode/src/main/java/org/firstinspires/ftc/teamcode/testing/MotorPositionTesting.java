package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.TelemetryManager;

@Configurable
@Config
// @TeleOp(name="Motor position testing", group="Debug")
public class MotorPositionTesting extends LinearOpMode {
	public static String MOTOR_NAME = "testMotor";
	public static String MOTOR_NAME_2 = "testMotor2";
	public static double POWER = 0;
	public static double POWER_2 = 0;
	public static double VELOCITY = 0;
	public static int TARGET = 0;
	public static boolean resetEncoder = true;
	public static DcMotor.RunMode MODE = DcMotor.RunMode.RUN_TO_POSITION;
	public static DcMotor.ZeroPowerBehavior ZERO_POWER = DcMotor.ZeroPowerBehavior.BRAKE;

	public static boolean PID_SET = false;
	public static double PID_P = 10;
	public static double PID_I = 0;
	public static double PID_D = 0;

	@Override
	public void runOpMode() {
		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcTelemetry(telemetry);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		DcMotorEx motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
		// DcMotor motor2 = hardwareMap.get(DcMotor.class, MOTOR_NAME_2);
		if (resetEncoder) {
			motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		}
		motor.setTargetPosition(TARGET);
		motor.setMode(MODE);
		motor.setZeroPowerBehavior(ZERO_POWER);

		PIDCoefficients originalPID = motor.getPIDCoefficients(MODE);

		waitForStart();

		while (opModeIsActive()) {
			if (PID_SET) {
				motor.setPIDCoefficients(MODE, new PIDCoefficients(PID_P, PID_I, PID_D));
			}
			motor.setPower(POWER);
			motor.setTargetPosition(TARGET);

			// motor2.setPower(POWER_2);

			if (motor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
				motor.setVelocity(VELOCITY);	
			}

			telemetryManager.addData("Power", motor.getPower());
			telemetryManager.addData("Position", motor.getCurrentPosition());
			telemetryManager.addData("Position Error", motor.getCurrentPosition() - motor.getTargetPosition());
			telemetryManager.addData("Busy", motor.isBusy());
			telemetryManager.addData("Mode", motor.getMode());
			telemetryManager.addData("Zero Power Behaviour", motor.getZeroPowerBehavior());

			telemetryManager.addData("PID Original", originalPID.toString());
			telemetryManager.addData("PID Current", motor.getPIDCoefficients(MODE).toString());

			telemetryManager.addData("Velocity", motor.getVelocity());
			telemetryManager.addData("Velocity Error", motor.getVelocity() - VELOCITY);
			telemetryManager.update();
		}
	}
}