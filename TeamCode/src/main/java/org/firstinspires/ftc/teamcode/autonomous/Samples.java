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
import org.firstinspires.ftc.teamcode.Automations;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.CV4B;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;

@Config
@Autonomous(name = "Samples Auto", group = "Autonomous")
public class Samples extends LinearOpMode {
	public static double initialPoseX = -23;
	public static double initialPoseY = -62;
	public static double initialPoseHeading = 0;
	private IntakeActions intakeActions; 
	private SlidesActions slidesActions;
	private CV4BActions cv4bActions;
	private ClawActions clawActions;
	
	@Override
	public void runOpMode() {
		Pose2d initialPose = new Pose2d(initialPoseX, initialPoseY, Math.toRadians(initialPoseHeading));
		MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
		// AprilTagDrive drive = new AprilTagDrive(hardwareMap, initialPose, aprilTagProcessor, true);
		intakeActions = new IntakeActions(hardwareMap);
		slidesActions = new SlidesActions(hardwareMap);
		cv4bActions = new CV4BActions(hardwareMap);

		Action firstSample = drive.actionBuilder(new Pose2d(-23, -62, Math.toRadians(0)))
			.setReversed(true)
			.splineToConstantHeading(new Vector2d(-40, -50), Math.toRadians(180))
			.splineToSplineHeading(new Pose2d(-50, -50, Math.toRadians(45)), Math.toRadians(180))
			.build();

		Action basket = drive.actionBuilder(new Pose2d(-50, -50, Math.toRadians(45)))
			.strafeTo(new Vector2d(-55, -55))
			.build();
		
		Action intakeFirst = drive.actionBuilder(new Pose2d(-55, -55, Math.toRadians(45)))
			.splineToSplineHeading(new Pose2d(-50, -50, Math.toRadians(85)), Math.toRadians(45))
			.build();

		Action depositFirst = drive.actionBuilder(new Pose2d(-50, -50, Math.toRadians(85)))
			.setReversed(true)
			.splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(225))
			.build();

		Action intakeSecond = drive.actionBuilder(new Pose2d(-55, -55, Math.toRadians(45)))
			.splineToSplineHeading(new Pose2d(-50, -50, Math.toRadians(105)), Math.toRadians(45))
			.build();

		Action depositSecond = drive.actionBuilder(new Pose2d(-50, -50, Math.toRadians(105)))
			.setReversed(true)
			.splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(225))
			.build();

		Action intakeThird = drive.actionBuilder(new Pose2d(-55, -55, Math.toRadians(45)))
			.splineToSplineHeading(new Pose2d(-48, -41, Math.toRadians(145)), Math.toRadians(145))
			.build();

		Action depositThird = drive.actionBuilder(new Pose2d(-48, -41, Math.toRadians(145)))
			.setReversed(true)
			.splineToLinearHeading(new Pose2d(-50, -50, Math.toRadians(45)), Math.toRadians(225))
			.build();

		Action park = drive.actionBuilder(new Pose2d(-55, -55, Math.toRadians(45)))
			.splineToLinearHeading(new Pose2d(-25, -10, Math.toRadians(180)), Math.toRadians(0))
			.build();

		waitForStart();

		Actions.runBlocking(
			new SequentialAction(
				new ParallelAction(
					// firstSample,
					slidesActions.raiseSlides(), // Raise slides
					cv4bActions.setPosition(CV4B.Positions.DEPOSIT, 1.5), // Extend CV4B
					intakeActions.init()
				),
				// basket,
				clawActions.open(), // Dump
				
				new ParallelAction(
					// intakeFirst,
					cv4bActions.setPosition(CV4B.Positions.PRE_TRANSFER, 1.5), // Retract CV4B
					slidesActions.lowerSlides(), // Lower slides
					intakeActions.start() // Start intake
				),
				intakeActions.retract(), // Stop and retract intake
				intakeActions.transfer(), // Transfer
				new ParallelAction(
					slidesActions.raiseSlides(), // Raise slides
					cv4bActions.setPosition(CV4B.Positions.DEPOSIT, 1.5) // Extend CV4B
				),
				// depositFirst,
				clawActions.open(), // Dump
				
				new ParallelAction(
					// intakeSecond,
					cv4bActions.setPosition(CV4B.Positions.PRE_TRANSFER, 1.5), // Retract CV4B
					slidesActions.lowerSlides(), // Lower slides
					intakeActions.start() // Start intake
				),
				intakeActions.retract(), // Stop and retract intake
				intakeActions.transfer(), // Transfer
				new ParallelAction(
					slidesActions.raiseSlides(), // Raise slides
					cv4bActions.setPosition(CV4B.Positions.DEPOSIT, 1.5) // Extend CV4B
				),
				// depositSecond,
				clawActions.open(), // Dump
				
				new ParallelAction(
					// intakeThird,
					cv4bActions.setPosition(CV4B.Positions.PRE_TRANSFER, 1.5), // Retract CV4B
					slidesActions.lowerSlides(), // Lower slides
					intakeActions.start() // Start intake
				),
				intakeActions.retract(), // Stop and retract intake
				new ParallelAction(
					// depositThird,
					new SequentialAction(
						intakeActions.transfer(), // Transfer
						new ParallelAction(
							slidesActions.raiseSlides(), // Raise slides
							cv4bActions.setPosition(CV4B.Positions.DEPOSIT, 1.5) // Extend CV4B
						)
					)
				),
				// basket,
				clawActions.open(), // Dump
				
				new ParallelAction(
					// park,
					cv4bActions.setPosition(CV4B.Positions.PRE_TRANSFER, 1.5), // Retract CV4B
					slidesActions.slidesToAscend() // Lower slides
				)
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
				private ElapsedTime timer = new ElapsedTime();
				@Override
				public boolean run(@NonNull TelemetryPacket packet) {
					if (timer.time() < 0.3 && timer.time() < 0.6) {
						intake.setBucketPosition(Intake.Positions.TRANSFER);
					} else {
						return false;
					}
					return true;
				}
			};
		}

		public Action start() {
			return new Action() {
				@Override
				public boolean run(@NonNull TelemetryPacket packet) {
					intake.setPosition(Intake.Positions.INTAKE);
					intake.setPower(0.5);
					
					return false;
				}
			};
		}

		public Action retract() {
			return new Action() {
				private ElapsedTime timer = new ElapsedTime();

				@Override
				public boolean run(@NonNull TelemetryPacket packet) {
					intake.setPosition(Intake.Positions.TRANSFER);
					intake.setPower(0);
					cv4bActions.setPositionMethod(CV4B.Positions.PRE_TRANSFER);
					clawActions.setPositionMethod(Claw.Positions.OPEN);

					if (timer.time() > 1.5 && !intake.isSlideBusy()) {
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
						cv4bActions.setPosition(CV4B.Positions.TRANSFER);
						initialized = true;
					} else if (timer.time() > 0.3) {
						clawActions.setPositionMethod(Claw.Positions.CLOSED);
						return false;
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
	
				@Override
				public boolean run(@NonNull TelemetryPacket packet) {
					if (!initialized) {
						slides.setPosition(Slides.Positions.HIGH_BASKET);
						
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

		public Action lowerSlides() {
			return new Action() {
				private boolean initialized = false;
	
				@Override
				public boolean run(@NonNull TelemetryPacket packet) {
					if (!initialized) {
						slides.setPosition(Slides.Positions.RETRACTED);
						
						initialized = true;
						return true;
					}
	
					if (!slides.isSlideBusy()) {
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
						
						initialized = true;
						return true;
					}

					if (timer.time() < 0.3) {
						return true;
					}
					return false;
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
						
						initialized = true;
						return true;
					}

					if (timer.time() < 0.3) {
						return true;
					}
					return false;
				}
			};
		}

		public void setPositionMethod(Claw.Positions position) {
			claw.setPosition(position);
		}
	}
}
