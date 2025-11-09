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
// @TeleOp(name="Manual Individual Control", group="Debug")
public class ManualIndividualControl extends LinearOpMode {
	public static String MOTOR_NAME_SPINDEXER = "testMotor";
	public static String MOTOR_NAME_INTAKE = "testMotor2";
	public static String MOTOR_NAME_SHOOTER_1 = "testMotor3";
	public static String MOTOR_NAME_SHOOTER_2 = "testMotor4";
	public static double POWER_SPINDEXER = 0;
	public static double POWER_INTAKE = 0;
	public static double POWER_SHOOTER = 0;
	public static double SHOOTER_VELOCITY = 0;
	public static boolean SHOOTER_USE_VELOCITY = false;
	public static int SPINDEXER_TARGET = 0;
	public static boolean SPINDEXER_RESET_ENCODER = true;

	public static double SPINDEXER_PID_P = 10;
	public static double SPINDEXER_PID_I = 0;
	public static double SPINDEXER_PID_D = 0;

	@Override
	public void runOpMode() {
		final boolean USING_VELOCITY = SHOOTER_USE_VELOCITY;

		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcTelemetry(telemetry);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		DcMotorEx storageMotor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_SPINDEXER);
		DcMotor intakeMotor = hardwareMap.get(DcMotor.class, MOTOR_NAME_INTAKE);
		DcMotorEx shooterMotorLeft = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_SHOOTER_1);
		DcMotorEx shooterMotorRight = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_SHOOTER_2);

		if (SPINDEXER_RESET_ENCODER) {
			storageMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		}
		storageMotor.setTargetPosition(SPINDEXER_TARGET);
		storageMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		storageMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		shooterMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		shooterMotorLeft.setDirection(DcMotor.Direction.REVERSE);
		shooterMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		if (USING_VELOCITY) {
			shooterMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		} else {
			shooterMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		}
		shooterMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		waitForStart();

		while (opModeIsActive()) {
			storageMotor.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
					new PIDCoefficients(SPINDEXER_PID_P, SPINDEXER_PID_I, SPINDEXER_PID_D));
			storageMotor.setPower(POWER_SPINDEXER);
			storageMotor.setTargetPosition(SPINDEXER_TARGET);

			intakeMotor.setPower(POWER_INTAKE);

			if (USING_VELOCITY) {
				shooterMotorLeft.setVelocity(SHOOTER_VELOCITY);
			} else {
				shooterMotorLeft.setPower(POWER_SHOOTER);				
			}
			shooterMotorRight.setPower(shooterMotorLeft.getPower());

			telemetryManager.addData("Spindexer - Position", storageMotor.getCurrentPosition());
			telemetryManager.addData("Spindexer - Position Error", storageMotor.getCurrentPosition() - storageMotor.getTargetPosition());
			telemetryManager.addData("Spindexer - Busy", storageMotor.isBusy());

			telemetryManager.addData("Shooter - Power", shooterMotorLeft.getPower());
			telemetryManager.addData("Shooter - Velocity", shooterMotorLeft.getVelocity());
			telemetryManager.addData("Shooter - Velocity Error", shooterMotorLeft.getVelocity() - SHOOTER_VELOCITY);
			telemetryManager.update();
		}
	}
}