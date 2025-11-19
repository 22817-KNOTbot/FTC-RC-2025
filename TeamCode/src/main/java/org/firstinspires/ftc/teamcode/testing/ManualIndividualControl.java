package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.TelemetryManager;

@Configurable
@Config
// @TeleOp(name="Manual Individual Control", group="Debug")
public class ManualIndividualControl extends LinearOpMode {
	public static final String MOTOR_NAME_SPINDEXER = "storageMotor";
	public static final String MOTOR_NAME_INTAKE = "intakeMotor";
	public static final String MOTOR_NAME_SHOOTER_1 = "shooterMotorLeft";
	public static final String MOTOR_NAME_SHOOTER_2 = "shooterMotorRight";
	public static final String SERVO_NAME_TRANSFER_RIGHT = "transferRampServo";
	public static final String SERVO_NAME_GATE = "gateServo";
	
	public static double POWER_SPINDEXER = 0;
	public static double POWER_INTAKE = 0;
	public static double POWER_SHOOTER = 0;
	public static int SPINDEXER_TARGET = 0;
	public static boolean SPINDEXER_RESET_ENCODER = true;
	public static double TRANSFER_RIGHT_POSITION = 0.47;
	public static double GATE_POSITION = 0.318;
	public static double FLIPPER_POSITION = 0.1;

	public static double SPINDEXER_PID_P = 10;
	public static double SPINDEXER_PID_I = 0;
	public static double SPINDEXER_PID_D = 0;

	@Override
	public void runOpMode() {
		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcTelemetry(telemetry);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		DcMotorEx storageMotor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_SPINDEXER);
		DcMotor intakeMotor = hardwareMap.get(DcMotor.class, MOTOR_NAME_INTAKE);
		DcMotorEx shooterMotorLeft = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_SHOOTER_1);
		DcMotorEx shooterMotorRight = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_SHOOTER_2);

		Servo transferRightServo = hardwareMap.get(Servo.class, SERVO_NAME_TRANSFER_RIGHT);
		Servo gateServo = hardwareMap.get(Servo.class, SERVO_NAME_GATE);

		if (SPINDEXER_RESET_ENCODER) {
			storageMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		}
		storageMotor.setTargetPosition(SPINDEXER_TARGET);
		storageMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		// storageMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		storageMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		storageMotor.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
				new PIDCoefficients(SPINDEXER_PID_P, SPINDEXER_PID_I, SPINDEXER_PID_D));

		intakeMotor.setDirection(DcMotor.Direction.REVERSE);
		intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		shooterMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		shooterMotorLeft.setDirection(DcMotor.Direction.REVERSE);
		shooterMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		shooterMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		shooterMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		waitForStart();

		while (opModeIsActive()) {
			// storageMotor.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
			// 		new PIDCoefficients(SPINDEXER_PID_P, SPINDEXER_PID_I, SPINDEXER_PID_D));
			storageMotor.setPower(POWER_SPINDEXER);
			storageMotor.setTargetPosition(SPINDEXER_TARGET);

			intakeMotor.setPower(POWER_INTAKE);

			shooterMotorLeft.setPower(POWER_SHOOTER);				
			shooterMotorRight.setPower(shooterMotorLeft.getPower());

			transferRightServo.setPosition(TRANSFER_RIGHT_POSITION);
			gateServo.setPosition(GATE_POSITION);

			telemetryManager.addData("Spindexer - Position", storageMotor.getCurrentPosition());
			telemetryManager.addData("Spindexer - Position Error", storageMotor.getCurrentPosition() - storageMotor.getTargetPosition());
			telemetryManager.addData("Spindexer - Busy", storageMotor.isBusy());

			telemetryManager.addData("Shooter - Power", shooterMotorLeft.getPower());
			telemetryManager.addData("Shooter - Velocity", shooterMotorLeft.getVelocity());
			telemetryManager.update();
		}
	}
}