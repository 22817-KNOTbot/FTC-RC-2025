package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
// import org.firstinspires.ftc.teamode.AprilTagDrive;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;

@Config
@Autonomous(name = "Samples Testing Auto", group = "Testing")
public class Samples extends LinearOpMode {
	public static double initialPoseX = -23;
	public static double initialPoseY = -62;
	public static double initialPoseHeading = 0;
	private Intake intakeControl; 
	private Slides slideControl;
	private CV4B cv4bControl;	

	private enum Cv4bPosition {
		BASE,
		TRANSFER,
		PRE_DEPOSIT,
		DUMP,
		SPECIMEN_GRAB
	}
	
	@Override
	public void runOpMode() {
		Pose2d initialPose = new Pose2d(initialPoseX, initialPoseY, Math.toRadians(initialPoseHeading));
		MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
		// AprilTagDrive drive = new AprilTagDrive(hardwareMap, initialPose, aprilTagProcessor, true);
		intakeControl = new Intake(hardwareMap);
		slideControl = new Slides(hardwareMap);
		cv4bControl = new CV4B(hardwareMap);

		Action preDeposit = drive.actionBuilder(initialPose)
			.setReversed(true)
			.splineToConstantHeading(new Vector2d(-40, -55), Math.toRadians(180))
			.splineToSplineHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180))
			.build();

		Action intakeFirst = drive.actionBuilder(new Pose2d(-55, -55, Math.toRadians(45)))
			.splineToLinearHeading(new Pose2d(-49, -38, Math.toRadians(90)), Math.toRadians(90))
			.build();

		Action depositFirst = drive.actionBuilder(new Pose2d(-49, -38, Math.toRadians(90)))
			.setReversed(true)
			.splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(270))
			.build();

		Action intakeSecond = drive.actionBuilder(new Pose2d(-55, -55, Math.toRadians(45)))
			.splineToLinearHeading(new Pose2d(-59, -38, Math.toRadians(90)), Math.toRadians(90))
			.build();

		Action depositSecond = drive.actionBuilder(new Pose2d(-59, -38, Math.toRadians(90)))
			.setReversed(true)
			.splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(270))
			.build();

		Action intakeThird = drive.actionBuilder(new Pose2d(-55, -55, Math.toRadians(45)))
			.splineToLinearHeading(new Pose2d(-59, -25, Math.toRadians(180)), Math.toRadians(180))
			.build();

		Action depositThird = drive.actionBuilder(new Pose2d(-59, -25, Math.toRadians(180)))
			.setReversed(true)
			.splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(270))
			.build();

		Action park = drive.actionBuilder(new Pose2d(-55, -55, Math.toRadians(45)))
			.splineToLinearHeading(new Pose2d(-25, -10, Math.toRadians(180)), Math.toRadians(0))
			.build();


		waitForStart();

		Actions.runBlocking(
			new SequentialAction(
				new ParallelAction(
					preDeposit,
					slideControl.raiseSlides(), // Raise slides
					cv4bControl.setPosition(Cv4bPosition.PRE_DEPOSIT) // Extend CV4B
				),
				cv4bControl.setPosition(Cv4bPosition.DUMP), // Dump
				
				new ParallelAction(
					cv4bControl.setPosition(Cv4bPosition.BASE), // Retract CV4B
					slideControl.lowerSlides(), // Lower slides
					intakeFirst,
					intakeControl.start() // Start intake
				),
				intakeControl.retract(), // Stop and retract intake
				new ParallelAction(
					depositFirst,
					new SequentialAction(
						intakeControl.transfer(), // Transfer
						slideControl.raiseSlides(), // Raise slides
						cv4bControl.setPosition(Cv4bPosition.PRE_DEPOSIT) // Extend CV4B
					)
				),
				cv4bControl.setPosition(Cv4bPosition.DUMP), // Dump
				
				new ParallelAction(
					cv4bControl.setPosition(Cv4bPosition.BASE), // Retract CV4B
					slideControl.lowerSlides(), // Lower slides
					intakeSecond,
					intakeControl.start() // Start intake
				),
				intakeControl.retract(), // Stop and retract intake
				new ParallelAction(
					depositSecond,
					new SequentialAction(
						intakeControl.transfer(), // Transfer
						slideControl.raiseSlides(), // Raise slides
						cv4bControl.setPosition(Cv4bPosition.PRE_DEPOSIT) // Extend CV4B
					)
				),
				cv4bControl.setPosition(Cv4bPosition.DUMP), // Dump
				
				new ParallelAction(
					cv4bControl.setPosition(Cv4bPosition.BASE), // Retract CV4B
					slideControl.lowerSlides(), // Lower slides
					intakeThird,
					intakeControl.start() // Start intake
				),
				intakeControl.retract(), // Stop and retract intake
				new ParallelAction(
					depositThird,
					new SequentialAction(
						intakeControl.transfer(), // Transfer
						slideControl.raiseSlides(), // Raise slides
						cv4bControl.setPosition(Cv4bPosition.PRE_DEPOSIT) // Extend CV4B
					)
				),
				cv4bControl.setPosition(Cv4bPosition.DUMP), // Dump
				
				new ParallelAction(
					cv4bControl.setPosition(Cv4bPosition.BASE), // Retract CV4B
					park,
					slideControl.slidesToAscend() // Lower slides
				)
			)
		);

		// Store pose for future use
		OpModeStorage.pose = drive.pose;
	}

	// public class PlaceholderAction implements Action {
	//	  private boolean initialized = false;

	//	  @Override
	//	  public boolean run(@NonNull TelemetryPacket packet) {
	//	if (!initialized) {
	//	 initialized = true;
	//	 return true;
	//	}

	//	// packet.put("Foo", "bar");

	//	return false;
	//	  }
	// }

	public class Intake {
		private DcMotor intakeSlides;
		private DcMotor scoopMotor;
		private ColorRangeSensor colourRangeSensor;

		public Intake(HardwareMap hardwareMap) {
			intakeSlides = hardwareMap.get(DcMotor.class, "intakeSlides");
			intakeSlides.setDirection(DcMotor.Direction.REVERSE);
			intakeSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			intakeSlides.setTargetPosition(0);
			intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			intakeSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			scoopMotor = hardwareMap.get(DcMotor.class, "scoopMotor");
			// scoopMotor.setDirection(DcMotor.Direction.REVERSE);
			colourRangeSensor = hardwareMap.get(ColorRangeSensor.class, "colorSensor");	 
		}

		public Action start() {
			return new Action() {
				@Override
				public boolean run(@NonNull TelemetryPacket packet) {
					intakeSlides.setPower(1);
					intakeSlides.setTargetPosition(100);  
					scoopMotor.setPower(0.25);

					return false;
				}
			};
		}

		public Action retract() {
			return new Action() {
				@Override
				public boolean run(@NonNull TelemetryPacket packet) {
					intakeSlides.setTargetPosition(0);  
					cv4bControl.setPositionMethod(Cv4bPosition.TRANSFER);

					return false;
				}
			};
		}

		public Action transfer() {
			return new Action() {
				@Override
				public boolean run(@NonNull TelemetryPacket packet) {
					if (!intakeSlides.isBusy()) {
						if (colourRangeSensor.getDistance(DistanceUnit.MM) < 10) {
							scoopMotor.setPower(-0.25);
						} else {
							scoopMotor.setPower(0);

							return false;
						}
					}

					return true;
				}
			};
		}
	}

	public class Slides {
		private DcMotor slideMotorLeft;
		private DcMotor slideMotorRight;

		public Slides(HardwareMap hardwareMap) {
			slideMotorLeft = hardwareMap.get(DcMotor.class, "slideMotorLeft");
			slideMotorRight = hardwareMap.get(DcMotor.class, "slideMotorRight");
			slideMotorLeft.setDirection(DcMotor.Direction.REVERSE);
			slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			slideMotorLeft.setTargetPosition(0);
			slideMotorRight.setTargetPosition(0);
			slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		}

		public Action raiseSlides() {
			return new Action() {
				private boolean initialized = false;
	
				@Override
				public boolean run(@NonNull TelemetryPacket packet) {
					if (!initialized) {
						slideMotorLeft.setPower(1);
						slideMotorRight.setPower(1);
						slideMotorLeft.setTargetPosition(3800);
						slideMotorRight.setTargetPosition(3800);
	
						initialized = true;
						return true;
					}
	
					if (!slideMotorLeft.isBusy() && !slideMotorRight.isBusy()) {
						return false;
					}
					return true;
				}
			};
		}

		public Action lowerSlides() {
			return new Action() {
				private boolean initialized = false;
	
				@Override
				public boolean run(@NonNull TelemetryPacket packet) {
					if (!initialized) {
						slideMotorLeft.setPower(1);
						slideMotorRight.setPower(1);
						slideMotorLeft.setTargetPosition(0);
						slideMotorRight.setTargetPosition(0);
	
						initialized = true;
						return true;
					}
	
					if (!slideMotorLeft.isBusy() && !slideMotorRight.isBusy()) {
						slideMotorLeft.setPower(0);
						slideMotorRight.setPower(0);

						return false;
					}
					return true;
				}
			};
		}

		public Action slidesToAscend() {
			return new Action() {
				private boolean initialized = false;
	
				@Override
				public boolean run(@NonNull TelemetryPacket packet) {
					if (!initialized) {
						slideMotorLeft.setPower(1);
						slideMotorRight.setPower(1);
						slideMotorLeft.setTargetPosition(2000);
						slideMotorRight.setTargetPosition(2000);
	
						initialized = true;
						return true;
					}
	
					if (!slideMotorLeft.isBusy() && !slideMotorRight.isBusy()) {
						return false;
					}
					return true;
				}
			};
		}
	}

	public class CV4B {
		private Servo cv4bLeftServo;
		private Servo cv4bRightServo;
		private Servo cv4bCoaxialServo;
	
		public CV4B(HardwareMap hardwareMao) {
			cv4bLeftServo = hardwareMap.get(Servo.class, "cv4bLeftServo");
			cv4bRightServo = hardwareMap.get(Servo.class, "cv4bRightServo");
			cv4bLeftServo.setDirection(Servo.Direction.FORWARD);
			cv4bRightServo.setDirection(Servo.Direction.REVERSE);
			cv4bCoaxialServo = hardwareMap.get(Servo.class, "cv4bCoaxialServo");
			cv4bCoaxialServo.setDirection(Servo.Direction.REVERSE);
		}

		public Action setPosition(Cv4bPosition position) {
			return setPosition(position, 0);
		}

		public Action setPosition(Cv4bPosition position, double length) {
			double v4bRot;
			double coaxialRot;

			switch (position) {
				default:
				case BASE:
					v4bRot = 0.14;
					coaxialRot = 0.2;
					break;
				case TRANSFER:
					v4bRot = 0.3;
					coaxialRot = 0.1;
					break;
				case PRE_DEPOSIT:
					v4bRot = 0.68;
					coaxialRot = 0.35;
					break;
				case DUMP:
					v4bRot = 0.68;
					coaxialRot = 0.8;
					break;
				case SPECIMEN_GRAB:
					v4bRot = 0.78;
					coaxialRot = 0.5;
					break;
			}

			return new Action() {
				private boolean initialized = false;
				private ElapsedTime timer = new ElapsedTime();

				@Override
				public boolean run(@NonNull TelemetryPacket packet) {
					if (!initialized) {
						cv4bLeftServo.setPosition(v4bRot);
						cv4bRightServo.setPosition(v4bRot);
						cv4bCoaxialServo.setPosition(coaxialRot);
				
						timer.reset();
						initialized = true;
						return true;
					}

					if (timer.time() < length) {
						return true;
					}
					return false;
				}
			};
		}

		public void setPositionMethod(Cv4bPosition position) {
			double v4bRot;
			double coaxialRot;

			switch (position) {
				default:
				case BASE:
					v4bRot = 0.14;
					coaxialRot = 0.2;
					break;
				case TRANSFER:
					v4bRot = 0.3;
					coaxialRot = 0.1;
					break;
				case PRE_DEPOSIT:
					v4bRot = 0.68;
					coaxialRot = 0.35;
					break;
				case DUMP:
					v4bRot = 0.68;
					coaxialRot = 0.8;
					break;
				case SPECIMEN_GRAB:
					v4bRot = 0.78;
					coaxialRot = 0.5;
					break;
			}

			cv4bLeftServo.setPosition(v4bRot);
			cv4bRightServo.setPosition(v4bRot);
			cv4bCoaxialServo.setPosition(coaxialRot);
		}
	}

	public Action placeholderAction() {
		return new Action() {
			private boolean initialized = false;

			@Override
			public boolean run(@NonNull TelemetryPacket packet) {
				if (!initialized) {
					initialized = true;
					return true;
				}
	
				// packet.put("Foo", "bar");
	
				return false;
			}
		};
	}
}
