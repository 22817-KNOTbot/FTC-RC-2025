package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.TelemetryManager;

@Configurable
@Config
// @TeleOp(name="Transfer Sequence Testing", group="Debug")
public class TransferSequenceTesting extends LinearOpMode {
	public static final String MOTOR_NAME_SPINDEXER = "storageMotor";
	public static final String MOTOR_NAME_INTAKE = "intakeMotor";
	public static final String MOTOR_NAME_SHOOTER_1 = "shooterMotorLeft";
	public static final String MOTOR_NAME_SHOOTER_2 = "shooterMotorRight";
	public static final String SERVO_NAME_TRANSFER_RIGHT = "transferRampServo";
	public static final String SERVO_NAME_GATE = "gateServo";
	public static final String SERVO_NAME_FLIPPER = "miniFlipperServo";

	public static double POWER_SPINDEXER = 0.4;
	public static double POWER_SHOOTER = 0.85;

	public static boolean RUN = false;
	public static int WAIT_POST_RAMP = 400;
	public static int WAIT_POST_SPIN = 1400;
	public static int WAIT_POST_FLIP = 800;
	public static int WAIT_POST_UNFLIP = 500;
	public static int spindexerIncrement = 128;
	public static int shooterVelocity = 2000;
	public static double transferRampPosition = 0.535;
	public static double gatePosition = 0.318;
	public static double flipperInPosition = 0.1;
	public static double flipperOutPosition = 0.87;

	public static double SPINDEXER_PID_P = 9.72;
	public static final double SPINDEXER_PID_I = 0;
	public static final double SPINDEXER_PID_D = 0;

	private TelemetryManager telemetryManager = new TelemetryManager();
	private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
	private DcMotorEx storageMotor;
	private DcMotorEx shooterMotorLeft;
	private DcMotorEx shooterMotorRight;
	private Servo transferRightServo;
	private Servo gateServo;
	private Servo miniFlipperServo;

	@Override
	public void runOpMode() {
		telemetryManager.setFtcTelemetry(telemetry);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		storageMotor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_SPINDEXER);
		shooterMotorLeft = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_SHOOTER_1);
		shooterMotorRight = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_SHOOTER_2);

		transferRightServo = hardwareMap.get(Servo.class, SERVO_NAME_TRANSFER_RIGHT);
		gateServo = hardwareMap.get(Servo.class, SERVO_NAME_GATE);
		miniFlipperServo = hardwareMap.get(Servo.class, SERVO_NAME_FLIPPER);
		
		storageMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		storageMotor.setTargetPosition(0);
		storageMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		storageMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		storageMotor.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
				new PIDCoefficients(SPINDEXER_PID_P, SPINDEXER_PID_I, SPINDEXER_PID_D));

		shooterMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		shooterMotorLeft.setDirection(DcMotor.Direction.REVERSE);
		shooterMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		shooterMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		shooterMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		waitForStart();

		storageMotor.setPower(POWER_SPINDEXER);
		miniFlipperServo.setPosition(flipperOutPosition);
		gateServo.setPosition(gatePosition);

		// shooterMotorLeft.setVelocity(shooterVelocity);
		shooterMotorLeft.setPower(POWER_SHOOTER);
		shooterMotorRight.setPower(shooterMotorLeft.getPower());

		while (opModeIsActive() && !RUN && !gamepad1.aWasPressed()) {
			sleep(50);
		}

		transferRightServo.setPosition(transferRampPosition);

		sleep(WAIT_POST_RAMP);

		for (int artifactsShot = 0; artifactsShot < 3; artifactsShot++) {
			transfer(spindexerIncrement * (artifactsShot + 1));
		}

		requestOpModeStop();
	}

	private void transfer(int spindexerTarget) {
		storageMotor.setTargetPosition(spindexerTarget);
		timer.reset();
		while (opModeIsActive() && timer.milliseconds() < WAIT_POST_SPIN) {
			telemetryManager.addData("Spindexer Target", storageMotor.getTargetPosition());
			telemetryManager.addData("Spindexer Position", storageMotor.getCurrentPosition());
			telemetryManager.addData("Spindexer PID", storageMotor.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
			telemetryManager.update();
			sleep(50);
		}

		while (opModeIsActive() && shooterMotorLeft.getVelocity() < shooterVelocity) {
			telemetryManager.addData("Shooter - Velocity", shooterMotorLeft.getVelocity());
			telemetryManager.update();
			sleep(50);
		}
		telemetryManager.addData("Shooter - Velocity", shooterMotorLeft.getVelocity());
		telemetryManager.update();

		miniFlipperServo.setPosition(flipperInPosition);
		timer.reset();
		while (opModeIsActive() && timer.milliseconds() < WAIT_POST_FLIP) {
			sleep(50);
		}

		storageMotor.setPower(0);
		miniFlipperServo.setPosition(flipperOutPosition);
		timer.reset();
		while (opModeIsActive() && timer.milliseconds() < WAIT_POST_UNFLIP) {
			sleep(50);
		}
		storageMotor.setPower(POWER_SPINDEXER);
	}
}