// fieldCentricRed will remain deprecated until necessary
// package org.firstinspires.ftc.teamcode.teleop;

// import java.util.List;

// import com.qualcomm.hardware.lynx.LynxModule;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.util.ElapsedTime;

// import com.acmerobotics.dashboard.canvas.Canvas;
// import com.acmerobotics.dashboard.config.Config;
// import com.acmerobotics.dashboard.FtcDashboard;
// import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

// import com.acmerobotics.roadrunner.Pose2d;
// import com.acmerobotics.roadrunner.PoseVelocity2d;
// import com.acmerobotics.roadrunner.Time;
// import com.acmerobotics.roadrunner.Twist2d;
// import com.acmerobotics.roadrunner.Twist2dDual;
// import com.acmerobotics.roadrunner.Vector2d;

// import org.firstinspires.ftc.teamcode.Automations;
// import org.firstinspires.ftc.teamcode.Drawing;
// import org.firstinspires.ftc.teamcode.Localizer;
// import org.firstinspires.ftc.teamcode.MecanumDrive;
// import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
// import org.firstinspires.ftc.teamcode.util.ControlTheory;
// import org.firstinspires.ftc.teamcode.util.OpModeStorage;

// @Config
// @TeleOp(name="Field Centric Teleop: Red", group="Field Centric")
// public class fieldCentricRed extends LinearOpMode {
// 	public static boolean DEBUG = true;
// 	public static boolean USE_PID = true;
// 	public static int TARGET_SPEED = 50;
// 	public static double Kp = 0.008;
// 	public static double Ki = 0;
// 	public static double Kd = 0;
// 	private ElapsedTime runtime = new ElapsedTime();
// 	private DcMotor frontLeftDrive;
// 	private DcMotor backLeftDrive;
// 	private DcMotor frontRightDrive;
// 	private DcMotor backRightDrive;
// 	private Automations automationHandler; 
// 	private Automations.Basket targetBasket;

// 	private Localizer localizer;
// 	private Pose2d pose;

// 	private Vector2d oldReferenceVel = new Vector2d(0, 0);
	
// 	@Override
// 	public void runOpMode() {       
// 		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); 
// 		automationHandler = new Automations(hardwareMap, DEBUG);
// 		ControlTheory.PID xVelocityController = new ControlTheory.PID(Kp, Ki, Kd, true);
// 		ControlTheory.PID yVelocityController = new ControlTheory.PID(Kp, Ki, Kd, false);
// 		targetBasket = Automations.Basket.HIGH;

// 		frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
// 		backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
// 		frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
// 		backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

// 		frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
// 		backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
// 		frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
// 		backRightDrive.setDirection(DcMotor.Direction.FORWARD);
// 		frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// 		backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// 		frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// 		backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// 		frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
// 		backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
// 		frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
// 		backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

// 		localizer = new ThreeDeadWheelLocalizer(hardwareMap, new MecanumDrive.Params().inPerTick);
// 		if (OpModeStorage.pose != null) {
// 			pose = OpModeStorage.pose;
// 		} else {
// 			pose = new Pose2d(0, 0, Math.PI*1.5);
// 		}
		
// 		// Bulk read
// 		List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
// 		for (LynxModule hub : allHubs) {
// 			hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
// 		}

// 		waitForStart();
// 		runtime.reset();

// 		while (opModeIsActive()) {
// 			// IMPORTANT: Cache must be cleared every loop to prevent stale data
// 			for (LynxModule hub : allHubs) {
// 				hub.clearBulkCache();
// 			}

// 			// Update pose for velocity and heading used in field centric rotation
// 			Vector2d velocity = updatePose();
// 			// First line for red, second line for b\lue ('\' is to prevent find and replace)
// 			double heading = Math.PI/2 - pose.heading.log();
// 			// double heading = Math.PI*1.5 - pose.heading.log();

// 			double xOutput;
// 			double yOutput;

// 			if (USE_PID) {
// 				// Sets a target velocity (field-centric)
// 				Vector2d referenceVel = new Vector2d(
// 					gamepad1.left_stick_x * TARGET_SPEED, 
// 					-gamepad1.left_stick_y * TARGET_SPEED);

// 				// If reference changes -> reset integral sum
// 				// Quite important for drivetrain movement to prevent delay in movement
// 				if (oldReferenceVel.x != referenceVel.x) {
// 					xVelocityController.resetIntegral();
// 				}
// 				if (oldReferenceVel.y != referenceVel.y) {
// 					yVelocityController.resetIntegral();
// 				}
				
// 				// Use PID calculations from ControlTheory.java
// 				xOutput = xVelocityController.calculate(referenceVel.x, velocity.x);
// 				yOutput = yVelocityController.calculate(referenceVel.y, velocity.y);
// 			} else {
// 				// Switch to regular input if PID doesn't work
// 				xOutput = gamepad1.left_stick_x;
// 				yOutput = -gamepad1.left_stick_y;

// 			}
// 			double rx = gamepad1.right_stick_x;

// 			// Rotate vector to be relative to robot
// 			double xRotated = xOutput * Math.cos(-heading) - yOutput * Math.sin(-heading);
// 			double yRotated = xOutput * Math.sin(-heading) + yOutput * Math.cos(heading);

// 			double denominator = Math.max(Math.abs(xRotated) + Math.abs(yRotated) + Math.abs(rx), 1);
// 			double frontLeftPower = (yRotated + xRotated + rx) / denominator;
// 			double backLeftPower = (yRotated - xRotated + rx) / denominator;
// 			double frontRightPower = (yRotated - xRotated - rx) / denominator;
// 			double backRightPower = (yRotated + xRotated - rx) / denominator;

// 			frontLeftDrive.setPower(frontLeftPower);
// 			backLeftDrive.setPower(backLeftPower);
// 			frontRightDrive.setPower(frontRightPower);
// 			backRightDrive.setPower(backRightPower);

// 			switch (automationHandler.automationState) {
// 				case ABORT:
// 					automationHandler.abort();
// 					break;
// 				case IDLE:
// 					if (gamepad1.a) {
// 						automationHandler.intakeInit(Automations.SamplePurpose.SAMPLE);
// 					} else if (gamepad1.x) {
// 						automationHandler.intakeInit(Automations.SamplePurpose.SPECIMEN);
// 					} else if (gamepad1.y) {
// 						automationHandler.specimenInit();
// 					} else if (gamepad1.left_trigger > 0.9 /* && runtime.time() > 90 */) {
// 						automationHandler.ascendInit();
// 					} else if (gamepad1.right_bumper) {
// 						automationHandler.setSlidePosition(0);
// 					} else {
// 						if (automationHandler.slideMotorLeft.getCurrentPosition() > -5) {
// 							automationHandler.slideMotorLeft.setPower(0);
// 						}
// 						if (automationHandler.slideMotorRight.getCurrentPosition() < 5) {
// 							automationHandler.slideMotorRight.setPower(0);
// 						}
// 					}

// 					break;

// 				// Sample intake
// 				case INTAKE_WAIT:
// 					automationHandler.intakeWait();
// 					break;
// 				case INTAKE_FILLED:
// 					automationHandler.intakeFilled(Automations.Alliance.RED, true);
// 					break;
// 				case INTAKE_DUMPING:
// 					automationHandler.intakeDumping();
// 					break;
// 				// Transfer
// 				case TRANSFER:
// 					automationHandler.transferInit();
// 					break;
// 				case TRANSFER_WAIT:
// 					automationHandler.transferWait();
// 					break;
// 				case TRANSFERRED:
// 					if (gamepad1.b) {
// 						automationHandler.depositInit(targetBasket);
// 					}
// 				// Deposit
// 				case DEPOSIT_EXTENDING:
// 					automationHandler.depositExtending();
// 					break;
// 				case DEPOSIT_EXTENDED:
// 					if (gamepad1.b) {
// 						automationHandler.depositSample();
// 					}
// 					break;
// 				case DEPOSITED:
// 					if (gamepad1.right_bumper) {
// 						automationHandler.resetDeposit();
// 					}
// 					break;

// 				// Specimens
// 				// Give to HP
// 				case NO_TRANSFER:
// 					automationHandler.noTransfer();
// 					break;
// 				case SAMPLE_LOADED:
// 					if (gamepad1.x) {
// 						automationHandler.sampleEjectInit();
// 					}
// 					break;		
// 				case SAMPLE_EJECT_WAIT:
// 					automationHandler.sampleEject();
// 					break;
// 				// Grabbing specimen
// 				case SPECIMEN_GRAB_READY:
// 					if (gamepad1.y) {
// 						automationHandler.grabSpecimen();
// 					}
// 					break;
// 				case SPECIMEN_GRABBED:
// 					if (gamepad1.y) {
// 						automationHandler.hangSpecimen();
// 					}
// 					break;
// 				case SPECIMEN_HUNG:
// 					if (gamepad1.right_bumper) {
// 						automationHandler.resetSpecimen();
// 					}

// 				// Ascent
// 				case ASCEND_LOW_EXTENDED:
// 					if (gamepad1.left_trigger > 0.9) {
// 						automationHandler.ascendLowRetract();
// 					}
// 					break;
// 			}

// 			if (gamepad1.back || gamepad2.back) {
// 				automationHandler.automationState = Automations.State.ABORT;
// 			} else if (gamepad2.start) {
// 				pose = new Pose2d(pose.position, 0);
// 			} else if (gamepad2.a) {
// 				targetBasket = targetBasket == Automations.Basket.HIGH ? Automations.Basket.LOW : Automations.Basket.HIGH;
// 			}

// 			telemetry.addData("State", automationHandler.automationState);
// 			telemetry.addData("Basket", targetBasket);
// 			if (DEBUG) {
// 				telemetry.addData("Heading", Math.toDegrees(heading));
// 				telemetry.addData("frontLeftPower", frontLeftPower);
// 				telemetry.addData("frontRightPower", frontRightPower);
// 				telemetry.addData("backLeftPower", backLeftPower);
// 				telemetry.addData("backRightPower", backRightPower);

// 				Canvas canvas = automationHandler.telemetryPacket.fieldOverlay().setRotation(Math.toRadians(-90));
// 				Drawing.drawRobot(canvas, pose);
// 				automationHandler.updateDashboardTelemetry();
// 			}
// 			telemetry.update();
// 		}
// 	}

// 	public Vector2d updatePose() {
//         Twist2dDual<Time> twist = localizer.update();
// 		pose = pose.plus(twist.value());

// 		return twist.value().line;
// 	}
// }