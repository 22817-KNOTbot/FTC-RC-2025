package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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
import org.firstinspires.ftc.teamcode.Automations;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.CV4B;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;

@Config
@Autonomous(name = "Samples Auto", group = "Autonomous")
public class Samples extends LinearOpMode {
	private final Alliances ALLIANCE = Alliances.BLUE;
	public static double initialPoseX = -23;
	public static double initialPoseY = -62;
	public static double initialPoseHeading = 0;
	private IntakeActions intakeActions;
	private SlidesActions slidesActions;
	private CV4BActions cv4bActions;
	private ClawActions clawActions;

	public enum Alliances {
		RED,
		BLUE
	}

	@Override
	public void runOpMode() {
		Pose2d initialPose = new Pose2d(initialPoseX, initialPoseY, Math.toRadians(initialPoseHeading));
		MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
		// AprilTagDrive drive = new AprilTagDrive(hardwareMap, initialPose,
		// aprilTagProcessor, true);
		intakeActions = new IntakeActions(hardwareMap);
		slidesActions = new SlidesActions(hardwareMap);
		cv4bActions = new CV4BActions(hardwareMap);
		clawActions = new ClawActions(hardwareMap);

		TrajectoryActionBuilder firstSample = drive.actionBuilder(initialPose)
			.setTangent(90)
			.splineToSplineHeading(new Pose2d(-54, -54, Math.toRadians(45)), Math.toRadians(200));

		TrajectoryActionBuilder intakeFirst = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(45)))
			.splineTo(new Vector2d(-49, -43), Math.toRadians(90));

		TrajectoryActionBuilder depositFirst = drive.actionBuilder(new Pose2d(-49, -43, Math.toRadians(90)))
			.setReversed(true)
			.splineTo(new Vector2d(-54, -54), Math.toRadians(225));

		TrajectoryActionBuilder intakeSecond = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(45)))
			.splineToSplineHeading(new Pose2d(-58, -48, Math.toRadians(90)), Math.toRadians(90))
			.splineToConstantHeading(new Vector2d(-58, -43), Math.toRadians(90));

		TrajectoryActionBuilder depositSecond = drive.actionBuilder(new Pose2d(-58, -43, Math.toRadians(90)))
			.setReversed(true)
			.splineTo(new Vector2d(-54, -54), Math.toRadians(225));

		TrajectoryActionBuilder intakeThird = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(45)))
			.splineToSplineHeading(new Pose2d(-52, -38, Math.toRadians(145)), Math.toRadians(145));

		TrajectoryActionBuilder depositThird = drive.actionBuilder(new Pose2d(-52, -38, Math.toRadians(145)))
			.setReversed(true)
			.splineTo(new Vector2d(-54, -54), Math.toRadians(225));
			
		TrajectoryActionBuilder intakeFourth = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(45)))
			.splineTo(new Vector2d(-25, -10), Math.toRadians(0));

		TrajectoryActionBuilder intakeSweep = drive.actionBuilder(new Pose2d(-25, -10, Math.toRadians(0)))
			.strafeTo(new Vector2d(-25, 10))
			.strafeTo(new Vector2d(-25, -10));
			
		TrajectoryActionBuilder depositFourth = drive.actionBuilder(new Pose2d(-25, -10, Math.toRadians(0)))
			.setReversed(true)
			.splineTo(new Vector2d(-54, -54), Math.toRadians(225));

		TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(45)))
			.splineToLinearHeading(new Pose2d(-25, 0, Math.toRadians(180)), Math.toRadians(0));

		waitForStart();

		Actions.runBlocking(
			new SequentialAction(
				new ParallelAction(
					clawActions.close(),
					cv4bActions.setPosition(CV4B.Positions.SPECIMEN_GRAB, 0.1),
					firstSample.build(),
					slidesActions.raiseSlides(), // Raise slides
					intakeActions.init()
				),
				cv4bActions.setPosition(CV4B.Positions.DEPOSIT, 1), // Extend CV4B
				clawActions.open(), // Dump
				
				new ParallelAction(
					intakeFirst.build(),
					cv4bActions.setPosition(CV4B.Positions.PRE_TRANSFER, 1), // Retract CV4B
					slidesActions.lowerSlides() // Lower slides
					),
				intakeActions.start(Intake.SLIDE_POSITION_MAX, 0), // Start intake
				new SleepAction(1),
				intakeActions.retract(), // Stop and retract intake
				new ParallelAction(
					depositFirst.build(),
					new SequentialAction(
						intakeActions.transfer(), // Transfer
						slidesActions.raiseSlides() // Raise slides
					)
				),
				cv4bActions.setPosition(CV4B.Positions.DEPOSIT, 1), // Extend CV4B
				clawActions.open(), // Dump
				
				new ParallelAction(
					intakeSecond.build(),
					cv4bActions.setPosition(CV4B.Positions.PRE_TRANSFER, 1), // Retract CV4B
					slidesActions.lowerSlides() // Lower slides
				),
				intakeActions.start(Intake.SLIDE_POSITION_MAX,0), // Start intake
				new SleepAction(1),
				intakeActions.retract(), // Stop and retract intake
				new ParallelAction(
					depositSecond.build(),
					new SequentialAction(
						intakeActions.transfer(), // Transfer
						slidesActions.raiseSlides() // Raise slides
					)
				),
				cv4bActions.setPosition(CV4B.Positions.DEPOSIT, 1), // Extend CV4B
				clawActions.open(), // Dump
				
				new ParallelAction(
					intakeThird.build(),
					cv4bActions.setPosition(CV4B.Positions.PRE_TRANSFER, 1), // Retract CV4B
					slidesActions.lowerSlides() // Lower slides
				),
				intakeActions.start(700, 0.1), // Start intake
				new SleepAction(1),
				intakeActions.retract(), // Stop and retract intake
				new ParallelAction(
					depositThird.build(),
					new SequentialAction(
						intakeActions.transfer(), // Transfer
						slidesActions.raiseSlides() // Raise slides
					)
				),
				cv4bActions.setPosition(CV4B.Positions.DEPOSIT, 1), // Extend CV4B
				clawActions.open(), // Dump

				new ParallelAction(
					intakeFourth.build(),
					cv4bActions.setPosition(CV4B.Positions.PRE_TRANSFER, 1), // Retract CV4B
					slidesActions.lowerSlides() // Lower slides
				),
				intakeActions.start(), // Start intake
				new RaceAction(
					intakeSweep.build(),
					intakeActions.intakeWait(drive)
				),
				intakeActions.retract(), // Stop and retract intake
				new ParallelAction(
					depositFourth.build(),
					new SequentialAction(
						intakeActions.transfer(), // Transfer
						new ParallelAction(
							slidesActions.raiseSlides(), // Raise slides
							cv4bActions.setPosition(CV4B.Positions.DEPOSIT, 1) // Extend CV4B
						)
					)
				),
				clawActions.open(), // Dump
				
				new ParallelAction(
					park.build(),
					slidesActions.lowerSlides()
				),

				cv4bActions.setPosition(CV4B.Positions.LEVEL_ONE_ASCENT, 1) // Retract CV4B

				
				// firstSample.build(),
				// new SleepAction(2),
				// new SleepAction(2),
				// intakeFirst.build(),
				// new SleepAction(2),
				// depositFirst.build(),
				// new SleepAction(2),
				// intakeSecond.build(),
				// new SleepAction(2),
				// depositSecond.build(),
				// new SleepAction(2),
				// intakeThird.build(),
				// new SleepAction(2),
				// depositThird.build(),
				// new SleepAction(2),
				// intakeFourth.build(),
				// new SleepAction(2),
				// intakeSweep.build(),
				// new SleepAction(2),
				// depositFourth.build(),
				// new SleepAction(2),
				// park.build()
			)
		);

		// Store pose for future use
		OpModeStorage.pose = drive.pose;
		OpModeStorage.mode = Automations.Modes.SAMPLE;
	}

	public class IntakeActions {
		private Intake intake;

		public IntakeActions(HardwareMap hardwareMap) {
			intake = new Intake(hardwareMap, true);
		}

		public Action init() {
			return new Action() {
				private boolean initialized = false;
				private ElapsedTime timer = new ElapsedTime();

				@Override
				public boolean run(@NonNull TelemetryPacket packet) {
					if (!initialized) {
						timer.reset();
						initialized = true;
					}
					if (timer.time() > 1 && timer.time() < 1.2) {
						intake.setBucketPosition(Intake.Positions.TRANSFER);
					} else if (timer.time() > 1.2) {
						return false;
					}
					return true;
				}
			};
		}

		public Action start() {
			return start(Intake.SLIDE_POSITION_DEFAULT, 0.5);
		}

		public Action start(int position) {
			return start(position, 0.5);
		}

		public Action start(int position, double delay) {
			return new Action() {
				private boolean initialized = false;
				private ElapsedTime timer = new ElapsedTime();

				@Override
				public boolean run(@NonNull TelemetryPacket packet) {
					if (!initialized) {
						intake.setSlidePosition(position);
						intake.setPower(0.6);
						timer.reset();
						initialized = true;
					} else if (timer.time() > delay) {
						intake.setBucketPosition(Intake.Positions.INTAKE);
						return false;
					}

					return true;
				}
			};
		}

		public Action intakeWait(MecanumDrive drive) {
			return new Action() {
				private boolean initialized = false;
				private boolean extending = true;
				private ElapsedTime timer = new ElapsedTime();
				private boolean dumping = false;

				@Override
				public boolean run(@NonNull TelemetryPacket packet) {
					if (!initialized) {
						timer.reset();
						initialized = true;
					}
					if (extending) {
						intake.setSlidePosition(Intake.SLIDE_POSITION_MAX);
					} else {
						intake.setSlidePosition(Intake.SLIDE_POSITION_MIN + 500);
					}
					if (timer.time() > 0.7) {
						extending = !extending;
						timer.reset();
					}

					if ((!intake.colourSensorResponding() || intake.getDistance(DistanceUnit.MM) <= 50)
							|| intake.isTouched()) {
						if ((intake.checkSample(Intake.SampleColours.YELLOW)) || (ALLIANCE == Alliances.RED ? 
							(intake.checkSample(Intake.SampleColours.RED))
							: (intake.checkSample(Intake.SampleColours.BLUE)))) {
							drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));			

							return false;
						} else {
							intake.setPower(-0.5);
							dumping = true;
						}
					} else if (dumping) {
						intake.setPower(0.6);
						dumping = false;
					}
					return true;
				}
			};
		}

		public Action retract() {
			return new Action() {
				private boolean initialized = false;
				private ElapsedTime timer = new ElapsedTime();

				@Override
				public boolean run(@NonNull TelemetryPacket packet) {
					if (!initialized) {
						intake.setPosition(Intake.Positions.TRANSFER);
						cv4bActions.setPositionMethod(CV4B.Positions.PRE_TRANSFER);
						clawActions.setPositionMethod(Claw.Positions.OPEN);

						timer.reset();
						initialized = true;
					}

					if ((timer.time() > 0.3 && !intake.isSlideBusy()) || timer.time() > 1) {
						return false;
					}
					return true;
				}
			};
		}

		public Action transfer() {
			return new Action() {
				private ElapsedTime timer = new ElapsedTime();
				private boolean initialized = false;

				@Override
				public boolean run(@NonNull TelemetryPacket packet) {
					if (!initialized) {
						cv4bActions.setPositionMethod(CV4B.Positions.TRANSFER);

						timer.reset();
						initialized = true;
					} else if (timer.time() > 0.5) {
						if (timer.time() > 1.1) {
							return false;
						} else if (timer.time() > 0.8) {
							intake.setPosition(Intake.Positions.POST_TRANSFER);
						} else {
							clawActions.setPositionMethod(Claw.Positions.CLOSED);
							intake.setPower(0);
						}
					}

					return true;
				}
			};
		}
	}

	public class SlidesActions {
		private Slides slides;

		public SlidesActions(HardwareMap hardwareMap) {
			slides = new Slides(hardwareMap, true);
		}

		public Action raiseSlides() {
			return new Action() {
				private boolean initialized = false;
				private ElapsedTime timer = new ElapsedTime();

				@Override
				public boolean run(@NonNull TelemetryPacket packet) {
					if (!initialized) {
						slides.setPosition(Slides.Positions.HIGH_BASKET);

						timer.reset();
						initialized = true;
						return true;
					}

					if (!slides.isSlideBusy() || timer.time() > 1.5) {
						return false;
					}
					return true;
				}
			};
		}

		public Action lowerSlides() {
			return new Action() {
				private boolean initialized = false;
				private ElapsedTime timer = new ElapsedTime();

				@Override
				public boolean run(@NonNull TelemetryPacket packet) {
					if (!initialized) {
						slides.setPosition(Slides.Positions.RETRACTED);

						timer.reset();
						initialized = true;
						return true;
					}

					if (!slides.isSlideBusy()|| timer.time() > 1.5) {
						slides.setPower(0);

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
						slides.setPosition(Slides.Positions.ASCEND_ONE);

						initialized = true;
						return true;
					}

					if (!slides.isSlideBusy()) {
						return false;
					}
					return true;
				}
			};
		}
	}

	public class CV4BActions {
		private CV4B cv4b;

		public CV4BActions(HardwareMap hardwareMap) {
			cv4b = new CV4B(hardwareMap);
		}

		public Action setPosition(CV4B.Positions position) {
			return setPosition(position, 0);
		}

		public Action setPosition(CV4B.Positions position, double length) {
			return new Action() {
				private boolean initialized = false;
				private ElapsedTime timer = new ElapsedTime();

				@Override
				public boolean run(@NonNull TelemetryPacket packet) {
					if (!initialized) {
						cv4b.setPosition(position);

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

		public void setPositionMethod(CV4B.Positions position) {
			cv4b.setPosition(position);
		}
	}

	public class ClawActions {
		private Claw claw;

		public ClawActions(HardwareMap hardwareMap) {
			claw = new Claw(hardwareMap);
		}

		public Action open() {
			return new Action() {
				private boolean initialized = false;
				private ElapsedTime timer = new ElapsedTime();

				@Override
				public boolean run(@NonNull TelemetryPacket packet) {
					if (!initialized) {
						claw.setPosition(Claw.Positions.OPEN);
						
						timer.reset();
						initialized = true;
					} else if (timer.time() > 0.3) {
						return false;
					}
					return true;
				}
			};
		}

		public Action close() {
			return new Action() {
				private boolean initialized = false;
				private ElapsedTime timer = new ElapsedTime();

				@Override
				public boolean run(@NonNull TelemetryPacket packet) {
					if (!initialized) {
						claw.setPosition(Claw.Positions.CLOSED);
						
						timer.reset();
						initialized = true;
					} else if (timer.time() > 0.3) {
						return false;
					}

					return true;
				}
			};
		}

		public void setPositionMethod(Claw.Positions position) {
			claw.setPosition(position);
		}
	}
}
