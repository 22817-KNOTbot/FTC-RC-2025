package org.firstinspires.ftc.teamcode.teleop;

import java.util.List;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Automations;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.CV4B;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.ControlTheory;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;
import org.firstinspires.ftc.teamcode.util.GamepadStorage;

@Config
@TeleOp(name = "Field Centric Teleop: Red", group = "AAA Field Centric")
public class fieldCentricRed extends LinearOpMode {
	public static boolean DEBUG = false;
	public static boolean USE_PID = false;
	public static boolean USE_ODO = true;
	public static int TARGET_SPEED = 50;
	public static double Kp = 0.008;
	public static double Ki = 0;
	public static double Kd = 0;
	private ElapsedTime runtime = new ElapsedTime();
	private DcMotor frontLeftDrive;
	private DcMotor backLeftDrive;
	private DcMotor frontRightDrive;
	private DcMotor backRightDrive;
	private Automations automationHandler;
	private Automations.Basket targetBasket;

	private Localizer localizer;
	private Pose2d pose;

	private Vector2d oldReferenceVel = new Vector2d(0, 0);

	@Override
	public void runOpMode() {
		final boolean USE_PID = this.USE_PID;
		final boolean USE_ODO = this.USE_ODO;
		boolean buttonPressed = false;
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
		GamepadStorage.gamepad1 = gamepad1;
		GamepadStorage.gamepad2 = gamepad2;
		automationHandler = new Automations(hardwareMap,
				OpModeStorage.mode == null ? Automations.Modes.SPECIMEN : OpModeStorage.mode, DEBUG);
		ControlTheory.PID xVelocityController = new ControlTheory.PID(Kp, Ki, Kd, true);
		ControlTheory.PID yVelocityController = new ControlTheory.PID(Kp, Ki, Kd, false);
		targetBasket = Automations.Basket.HIGH;

		frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
		backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
		frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
		backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

		frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
		backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
		frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
		backRightDrive.setDirection(DcMotor.Direction.FORWARD);
		frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		IMU imu = null;
		if (USE_ODO) {
			localizer = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick);
			if (OpModeStorage.pose != null) {
				Pose2d storedPose = OpModeStorage.pose;
				pose = new Pose2d(storedPose.position, -storedPose.heading.plus(Math.PI / 2).log()); // Rotate so
																										// forwards = 0
			} else {
				pose = new Pose2d(0, 0, 0);
			}
		} else {
			imu = hardwareMap.get(IMU.class, "imu");
			IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
					RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
					RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
			imu.initialize(parameters);
			imu.resetYaw();
		}

		// Bulk read
		List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
		for (LynxModule hub : allHubs) {
			hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
		}

		if (!automationHandler.colourSensorResponding()) {
			automationHandler.vibrateControllers(1000);
			telemetry.addLine("********************");
			telemetry.addLine("WARNING: COLOUR SENSOR");
			telemetry.addLine("IS NOT RESPONDING");
			telemetry.addLine("********************");
		}

		waitForStart();
		runtime.reset();

		while (opModeIsActive()) {
			// IMPORTANT: Cache must be cleared every loop to prevent stale data
			for (LynxModule hub : allHubs) {
				hub.clearBulkCache();
			}

			// Update pose for velocity and heading used in field centric rotation
			double heading;
			Vector2d velocity = new Vector2d(0, 0);
			if (USE_ODO) {
				velocity = updatePose();
				heading = pose.heading.log();
			} else {
				heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
			}

			double xOutput;
			double yOutput;

			if (USE_PID && USE_ODO) {
				// Sets a target velocity (field-centric)
				Vector2d referenceVel = new Vector2d(
						gamepad1.left_stick_x * TARGET_SPEED,
						-gamepad1.left_stick_y * TARGET_SPEED);

				// If reference changes -> reset integral sum
				// Quite important for drivetrain movement to prevent delay in movement
				if (oldReferenceVel.x != referenceVel.x) {
					xVelocityController.resetIntegral();
				}
				if (oldReferenceVel.y != referenceVel.y) {
					yVelocityController.resetIntegral();
				}

				// Use PID calculations from ControlTheory.java
				xOutput = xVelocityController.calculate(referenceVel.x, velocity.x);
				yOutput = yVelocityController.calculate(referenceVel.y, velocity.y);
			} else {
				// Switch to regular input if PID doesn't work
				xOutput = gamepad1.left_stick_x;
				yOutput = -gamepad1.left_stick_y;

			}
			double rx = gamepad1.right_stick_x;

			// Rotate vector to be relative to robot
			double xRotated = xOutput * Math.cos(-heading) - yOutput * Math.sin(-heading);
			double yRotated = xOutput * Math.sin(-heading) + yOutput * Math.cos(-heading);

			double denominator = Math.max(Math.abs(xRotated) + Math.abs(yRotated) + Math.abs(rx), 1);
			double frontLeftPower = (yRotated + xRotated + rx) / denominator;
			double backLeftPower = (yRotated - xRotated + rx) / denominator;
			double frontRightPower = (yRotated - xRotated - rx) / denominator;
			double backRightPower = (yRotated + xRotated - rx) / denominator;

			frontLeftDrive.setPower(frontLeftPower);
			backLeftDrive.setPower(backLeftPower);
			frontRightDrive.setPower(frontRightPower);
			backRightDrive.setPower(backRightPower);

			switch (automationHandler.automationState) {
				case ABORT:
					automationHandler.abort();
					break;
				case IDLE:
					// Sample mode
					if (automationHandler.getMode() == Automations.Modes.SAMPLE) {
						if (gamepad2.a) {
							automationHandler.intakeInit(Automations.SamplePurpose.SAMPLE);
						} else if (gamepad2.b) {
							automationHandler.depositInit(targetBasket);
						} else if (gamepad2.x) {
							automationHandler.intakeInit(Automations.SamplePurpose.SPECIMEN);
						}
						automationHandler.setIntakePower(gamepad2.left_trigger > 0.9 ? -0.5 : 0);

						// Specimen mode
					} else if (automationHandler.getMode() == Automations.Modes.SPECIMEN) {
						if (gamepad2.y) {
							automationHandler.specimenInit();
						}
					}

					// Switch modes
					if (gamepad2.dpad_left) {
						automationHandler.setMode(Automations.Modes.SAMPLE);
					} else if (gamepad2.dpad_right) {
						automationHandler.setMode(Automations.Modes.SPECIMEN);
					}

					if (gamepad1.left_trigger > 0.9 && (runtime.time() > 90 || DEBUG)) {
						automationHandler.ascendInit();
					} else if (gamepad1.right_bumper) {
						automationHandler.retract();
					} else {
						if (automationHandler.getSlideLeftPosition() < 5
								&& automationHandler.getSlideRightPosition() < 5) {
							automationHandler.setSlidesPower(0);
						}
					}
					break;

				// Sample intake
				case INTAKE_WAIT:
					if (gamepad2.right_bumper) {
						automationHandler.manualIntakePosition(Intake.BUCKET_SUB_BARRIER, gamepad2.dpad_up, gamepad2.dpad_down);
					} else {
						automationHandler.intakePosition(gamepad2.right_trigger, gamepad2.dpad_up, gamepad2.dpad_down);
					}
					automationHandler.setIntakePower(gamepad2.left_trigger > 0.9 ? -0.5 : 0.6);
					automationHandler.intakeWait();
					break;
				case INTAKE_FILLED:
					automationHandler.intakeFilled(Automations.Alliance.RED, true);
					break;
				case INTAKE_DUMPING:
					automationHandler.intakeDumping();
					break;
				// Transfer
				case TRANSFER:
					automationHandler.transferInit();
					break;
				case TRANSFER_WAIT:
					automationHandler.transferWait();
					break;
				case TRANSFERRING:
					automationHandler.transferring();
					break;
				// Deposit
				case TRANSFERRED:
					if (gamepad2.b) {
						automationHandler.depositInit(targetBasket);
					}
					break;
				case DEPOSIT_EXTENDING:
					automationHandler.depositExtending();
					break;
				case DEPOSIT_EXTENDED:
					if (gamepad2.b && !buttonPressed) {
						automationHandler.depositSample();
					}
					break;
				case DEPOSITED:
					if (gamepad1.right_bumper) {
						automationHandler.resetDeposit();
					}
					break;

				// Specimens
				// Give to HP
				case NO_TRANSFER:
					automationHandler.noTransfer();
					break;
				case SAMPLE_LOADED:
					if (gamepad2.x) {
						automationHandler.sampleEjectInit();
					}
					break;
				case SAMPLE_EJECT_WAIT:
					automationHandler.sampleEject();
					break;
				// Grabbing specimen
				case SPECIMEN_INIT_WAIT:
					automationHandler.specimenInitWait();
					break;
				case SPECIMEN_GRAB_READY:
					if (gamepad2.y && !buttonPressed) {
						automationHandler.grabSpecimen();
					}
					break;
				case SPECIMEN_GRABBING:
					automationHandler.grabSpecimenWait();
					break;
				case SPECIMEN_GRABBED:
					if (gamepad2.y && !buttonPressed) {
						automationHandler.hangSpecimen();
					}
					break;
				case SPECIMEN_HANGING:
					automationHandler.hangSpecimenWait();
					break;
				case SPECIMEN_HUNG:
					if (gamepad1.right_bumper) {
						automationHandler.resetSpecimen();
					}
					break;
				// Ascent
				case ASCEND_LOW_EXTENDING:
					automationHandler.ascendLowExtending();
					break;
				case ASCEND_LOW_EXTENDED:
					if (gamepad1.left_trigger > 0.9 && !buttonPressed) {
						automationHandler.ascendLowRetract();
					}
					break;

				// Mode switching
				case SPECIMEN_TO_SAMPLE:
					automationHandler.specimenToSample();
					break;
				case SAMPLE_TO_SPECIMEN:
					automationHandler.sampleToSpecimen();
					break;
				default:
					break;
			}

			if (gamepad1.back || gamepad2.back) {
				automationHandler.automationState = Automations.State.ABORT;
			} else if (gamepad1.start || gamepad2.start) {
				if (USE_ODO) {
					pose = new Pose2d(pose.position, 0);
				} else {
					imu.resetYaw();
				}
			}
			if (gamepad1.a) {
				targetBasket = targetBasket == Automations.Basket.HIGH ? Automations.Basket.LOW
						: Automations.Basket.HIGH;
			}

			if (gamepad1.dpad_up) {
				CV4B.offset_drive += 0.05;
			} else if (gamepad1.dpad_down) {
				CV4B.offset_drive -= 0.05;
			}

			buttonPressed = gamepad2.y || gamepad2.b || gamepad1.left_trigger > 0.9;

			telemetry.addData("Time", runtime.time());
			telemetry.addData("State", automationHandler.automationState);
			telemetry.addData("Mode", automationHandler.getMode());
			if (!automationHandler.colourSensorResponding()) {
				telemetry.addLine("********************");
				telemetry.addLine("WARNING: COLOUR SENSOR");
				telemetry.addLine("IS NOT RESPONDING");
				telemetry.addLine("********************");
			}
			telemetry.addData("Basket", targetBasket);
			if (DEBUG) {
				telemetry.addData("Heading", Math.toDegrees(heading));
				telemetry.addData("frontLeftPower", frontLeftPower);
				telemetry.addData("frontRightPower", frontRightPower);
				telemetry.addData("backLeftPower", backLeftPower);
				telemetry.addData("backRightPower", backRightPower);

				Canvas canvas = automationHandler.telemetryPacket.fieldOverlay();
				if (USE_ODO) {
					Drawing.drawRobot(canvas, pose);
				} else {
					Drawing.drawRobot(canvas, new Pose2d(0, 0, heading));
				}
				automationHandler.updateDashboardTelemetry();
			}
			telemetry.update();
		}
	}

	public Vector2d updatePose() {
		Twist2dDual<Time> twist = localizer.update();
		pose = pose.plus(twist.value());

		return twist.value().line;
	}
}