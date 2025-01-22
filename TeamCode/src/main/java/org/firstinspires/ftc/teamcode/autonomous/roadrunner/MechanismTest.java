// package org.firstinspires.ftc.teamcode.autonomous.roadrunner;

// import androidx.annotation.NonNull;

// import com.acmerobotics.dashboard.canvas.Canvas;
// import com.acmerobotics.dashboard.config.Config;
// import com.acmerobotics.dashboard.FtcDashboard;
// import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

// import com.acmerobotics.roadrunner.Action;
// import com.acmerobotics.roadrunner.Pose2d;
// import com.acmerobotics.roadrunner.ParallelAction;
// import com.acmerobotics.roadrunner.SequentialAction;
// import com.acmerobotics.roadrunner.Vector2d;
// import com.acmerobotics.roadrunner.ftc.Actions;

// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.ColorRangeSensor;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.HardwareMap;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.util.ElapsedTime;

// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import org.firstinspires.ftc.teamcode.MecanumDrive;
// // import org.firstinspires.ftc.teamode.AprilTagDrive;
// import org.firstinspires.ftc.teamcode.subsystems.CV4B;
// import org.firstinspires.ftc.teamcode.util.OpModeStorage;

// @Config
// @Autonomous(name = "Mechanism Testing Auto", group = "Testing")
// public class MechanismTest extends LinearOpMode {
// 	public static double initialPoseX = -23;
// 	public static double initialPoseY = -62;
// 	public static double initialPoseHeading = 0;
// 	private Intake intakeControl; 
// 	private Slides slideControl;
// 	private CV4BActions cv4bControl;	
// 	private ClawActions clawControl;
	
// 	@Override
// 	public void runOpMode() {
// 		Pose2d initialPose = new Pose2d(initialPoseX, initialPoseY, Math.toRadians(initialPoseHeading));
// 		MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
// 		// AprilTagDrive drive = new AprilTagDrive(hardwareMap, initialPose, aprilTagProcessor, true);
// 		intakeControl = new Intake(hardwareMap);
// 		slideControl = new Slides(hardwareMap);
// 		cv4bControl = new CV4BActions(hardwareMap);

// 		waitForStart();

// 		Actions.runBlocking(
// 			new SequentialAction(
//                 new SequentialAction(
//                     intakeControl.init(),
//                     new ParallelAction(
//                         slideControl.raiseSlides(), // Raise slides
//                         cv4bControl.setPosition(CV4B.Positions.PRE_DEPOSIT, 1.5) // Extend CV4B
//                     )
//                 ),
// 				cv4bControl.setPosition(CV4B.Positions.DUMP, 1.25), // Dump
				
// 				new ParallelAction(
// 					cv4bControl.setPosition(CV4B.Positions.TRANSFER, 1.5), // Retract CV4B
// 					slideControl.lowerSlides(), // Lower slides
// 					intakeControl.start() // Start intake
// 				),
// 				intakeControl.retract(), // Stop and retract intake
// 				new ParallelAction(
// 					new SequentialAction(
// 						intakeControl.transfer(), // Transfer
// 						new ParallelAction(
// 							slideControl.raiseSlides(), // Raise slides
// 							cv4bControl.setPosition(CV4B.Positions.PRE_DEPOSIT, 1.5) // Extend CV4B
// 						)
// 					)
// 				),
// 				cv4bControl.setPosition(CV4B.Positions.DUMP, 1.25), // Dump
				
// 				new ParallelAction(
// 					cv4bControl.setPosition(CV4B.Positions.TRANSFER, 1.5), // Retract CV4B
// 					slideControl.lowerSlides(), // Lower slides
// 					intakeControl.start() // Start intake
// 				),
// 				intakeControl.retract(), // Stop and retract intake
// 				new ParallelAction(
// 					new SequentialAction(
// 						intakeControl.transfer(), // Transfer
// 						new ParallelAction(
// 							slideControl.raiseSlides(), // Raise slides
// 							cv4bControl.setPosition(CV4B.Positions.PRE_DEPOSIT, 1.5) // Extend CV4B
// 						)
// 					)
// 				),
// 				cv4bControl.setPosition(CV4B.Positions.DUMP, 1.25), // Dump
				
// 				new ParallelAction(
// 					cv4bControl.setPosition(CV4B.Positions.TRANSFER, 1.5), // Retract CV4B
// 					slideControl.lowerSlides(), // Lower slides
// 					intakeControl.start() // Start intake
// 				),
// 				intakeControl.retract(), // Stop and retract intake
// 				new ParallelAction(
// 					new SequentialAction(
// 						intakeControl.transfer(), // Transfer
// 						new ParallelAction(
// 							slideControl.raiseSlides(), // Raise slides
// 							cv4bControl.setPosition(CV4B.Positions.PRE_DEPOSIT, 1.5) // Extend CV4B
// 						)
// 					)
// 				),
// 				cv4bControl.setPosition(CV4B.Positions.DUMP, 1.25), // Dump
				
// 				new ParallelAction(
// 					cv4bControl.setPosition(CV4B.Positions.TRANSFER, 1.5), // Retract CV4B
// 					slideControl.slidesToAscend() // Lower slides
// 				)
// 			)
// 		);

// 		// Full
// 		// Actions.runBlocking(
// 		// 	new SequentialAction(
// 				// new ParallelAction(
// 				// 	firstSample,
// 				// 	new SequentialAction(
// 				// 		intakeControl.init(),
// 				// 		new ParallelAction(
// 				// 			slideControl.raiseSlides(), // Raise slides
// 				// 			cv4bControl.setPosition(CV4B.Positions.PRE_DEPOSIT, 1.5) // Extend CV4B
// 				// 		)
// 				// 	)
// 				// ),
// 		// 		cv4bControl.setPosition(CV4B.Positions.DUMP, 1.25), // Dump
				
// 		// 		new ParallelAction(
// 		// 			intakeFirst,
// 		// 			cv4bControl.setPosition(CV4B.Positions.TRANSFER, 1.5), // Retract CV4B
// 		// 			slideControl.lowerSlides(), // Lower slides
// 		// 			intakeControl.start() // Start intake
// 		// 		),
// 		// 		intakeControl.retract(), // Stop and retract intake
// 		// 		new ParallelAction(
// 		// 			depositFirst,
// 		// 			new SequentialAction(
// 		// 				intakeControl.transfer(), // Transfer
// 		// 				new ParallelAction(
// 		// 					slideControl.raiseSlides(), // Raise slides
// 		// 					cv4bControl.setPosition(CV4B.Positions.PRE_DEPOSIT, 1.5) // Extend CV4B
// 		// 				)
// 		// 			)
// 		// 		),
// 		// 		cv4bControl.setPosition(CV4B.Positions.DUMP, 1.25), // Dump
				
// 		// 		new ParallelAction(
// 		// 			intakeSecond,
// 		// 			cv4bControl.setPosition(CV4B.Positions.TRANSFER, 1.5), // Retract CV4B
// 		// 			slideControl.lowerSlides(), // Lower slides
// 		// 			intakeControl.start() // Start intake
// 		// 		),
// 		// 		intakeControl.retract(), // Stop and retract intake
// 		// 		new ParallelAction(
// 		// 			depositSecond,
// 		// 			new SequentialAction(
// 		// 				intakeControl.transfer(), // Transfer
// 		// 				new ParallelAction(
// 		// 					slideControl.raiseSlides(), // Raise slides
// 		// 					cv4bControl.setPosition(CV4B.Positions.PRE_DEPOSIT, 1.5) // Extend CV4B
// 		// 				)
// 		// 			)
// 		// 		),
// 		// 		cv4bControl.setPosition(CV4B.Positions.DUMP, 1.25), // Dump
				
// 		// 		new ParallelAction(
// 		// 			intakeThird,
// 		// 			cv4bControl.setPosition(CV4B.Positions.TRANSFER, 1.5), // Retract CV4B
// 		// 			slideControl.lowerSlides(), // Lower slides
// 		// 			intakeControl.start() // Start intake
// 		// 		),
// 		// 		intakeControl.retract(), // Stop and retract intake
// 		// 		new ParallelAction(
// 		// 			depositThird,
// 		// 			new SequentialAction(
// 		// 				intakeControl.transfer(), // Transfer
// 		// 				new ParallelAction(
// 		// 					slideControl.raiseSlides(), // Raise slides
// 		// 					cv4bControl.setPosition(CV4B.Positions.PRE_DEPOSIT, 1.5) // Extend CV4B
// 		// 				)
// 		// 			)
// 		// 		),
// 		// 		cv4bControl.setPosition(CV4B.Positions.DUMP, 1.25), // Dump
				
// 		// 		new ParallelAction(
// 		// 			park,
// 		// 			cv4bControl.setPosition(CV4B.Positions.TRANSFER, 1.5), // Retract CV4B
// 		// 			slideControl.slidesToAscend() // Lower slides
// 		// 		)
// 		// 	)
// 		// );

// 		// Store pose for future use
// 		OpModeStorage.pose = drive.pose;
// 	}

// 	public class Intake {
// 		private DcMotor intakeSlides;
// 		private DcMotor scoopMotor;
// 		private ColorRangeSensor colourRangeSensor;
// 		private Servo flipServo;

// 		public Intake(HardwareMap hardwareMap) {
// 			intakeSlides = hardwareMap.get(DcMotor.class, "intakeSlides");
// 			intakeSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 			intakeSlides.setTargetPosition(0);
// 			intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
// 			intakeSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
// 			scoopMotor = hardwareMap.get(DcMotor.class, "scoopMotor");
// 			scoopMotor.setDirection(DcMotor.Direction.REVERSE);
// 			colourRangeSensor = hardwareMap.get(ColorRangeSensor.class, "colorSensor");	 
// 			flipServo = hardwareMap.get(Servo.class, "flipServo");
// 		}

// 		public Action init() {
// 			return new Action() {
// 				private boolean initialized = false;
// 				private ElapsedTime timer = new ElapsedTime();
// 				@Override
// 				public boolean run(@NonNull TelemetryPacket packet) {
// 					if (!initialized) {
// 						flipServo.setPosition(0.875);
// 						intakeSlides.setTargetPosition(600);

//                         initialized = true;
// 					} else {
// 						if (timer.time() > 0.5) {
// 							return false;
// 						}
// 					}
// 					return true;
// 				}
// 			};
// 		}

// 		public Action start() {
// 			return new Action() {
// 				@Override
// 				public boolean run(@NonNull TelemetryPacket packet) {
// 					intakeSlides.setPower(1);
// 					intakeSlides.setTargetPosition(600);  
// 					scoopMotor.setPower(0.5);
// 					flipServo.setPosition(0.875);

// 					return false;
// 				}
// 			};
// 		}

// 		public Action retract() {
// 			return new Action() {
// 				private ElapsedTime timer = new ElapsedTime();

// 				@Override
// 				public boolean run(@NonNull TelemetryPacket packet) {
// 					intakeSlides.setTargetPosition(600);  
// 					flipServo.setPosition(0.76);
// 					cv4bControl.setPositionMethod(CV4B.Positions.TRANSFER);

// 					if (timer.time() > 1.5 && !intakeSlides.isBusy()) {
// 						return false;
// 					}
// 					return true;
// 				}
// 			};
// 		}

// 		public Action transfer() {
// 			return new Action() {
// 				private ElapsedTime timer = new ElapsedTime();
// 				private boolean finish = false;
// 				@Override
// 				public boolean run(@NonNull TelemetryPacket packet) {
// 					if (colourRangeSensor.getDistance(DistanceUnit.MM) < 75) {
// 						scoopMotor.setPower(-1);
// 					} else {
// 						if (!finish) {
// 							flipServo.setPosition(0.78);
// 							scoopMotor.setPower(0);
// 							timer.reset();
// 							finish = true;
// 						} else if (timer.time() > 0.5) {
// 							return false;
// 						}

// 					}

// 					return true;
// 				}
// 			};
// 		}
// 	}

// 	public class Slides {
// 		private DcMotor slideMotorLeft;
// 		private DcMotor slideMotorRight;

// 		public Slides(HardwareMap hardwareMap) {
// 			slideMotorLeft = hardwareMap.get(DcMotor.class, "slideMotorLeft");
// 			slideMotorRight = hardwareMap.get(DcMotor.class, "slideMotorRight");
// 			slideMotorLeft.setTargetPosition(0);
// 			slideMotorRight.setTargetPosition(0);
// 			slideMotorLeft.setDirection(DcMotor.Direction.REVERSE);
// 			slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 			slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 			slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
// 			slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
// 			slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
// 			slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
// 		}

// 		public Action raiseSlides() {
// 			return new Action() {
// 				private boolean initialized = false;
	
// 				@Override
// 				public boolean run(@NonNull TelemetryPacket packet) {
// 					if (!initialized) {
// 						slideMotorLeft.setPower(1);
// 						slideMotorRight.setPower(1);
// 						slideMotorLeft.setTargetPosition(4100);
// 						slideMotorRight.setTargetPosition(4100);
	
// 						initialized = true;
// 						return true;
// 					}
	
// 					if (!slideMotorLeft.isBusy() && !slideMotorRight.isBusy()) {
// 						return false;
// 					}
// 					return true;
// 				}
// 			};
// 		}

// 		public Action lowerSlides() {
// 			return new Action() {
// 				private boolean initialized = false;
	
// 				@Override
// 				public boolean run(@NonNull TelemetryPacket packet) {
// 					if (!initialized) {
// 						slideMotorLeft.setPower(1);
// 						slideMotorRight.setPower(1);
// 						slideMotorLeft.setTargetPosition(0);
// 						slideMotorRight.setTargetPosition(0);
	
// 						initialized = true;
// 						return true;
// 					}
	
// 					if (!slideMotorLeft.isBusy() && !slideMotorRight.isBusy()) {
// 						slideMotorLeft.setPower(0);
// 						slideMotorRight.setPower(0);

// 						return false;
// 					}
// 					return true;
// 				}
// 			};
// 		}

// 		public Action slidesToAscend() {
// 			return new Action() {
// 				private boolean initialized = false;
	
// 				@Override
// 				public boolean run(@NonNull TelemetryPacket packet) {
// 					if (!initialized) {
// 						slideMotorLeft.setPower(1);
// 						slideMotorRight.setPower(1);
// 						slideMotorLeft.setTargetPosition(2000);
// 						slideMotorRight.setTargetPosition(2000);
	
// 						initialized = true;
// 						return true;
// 					}
	
// 					if (!slideMotorLeft.isBusy() && !slideMotorRight.isBusy()) {
// 						return false;
// 					}
// 					return true;
// 				}
// 			};
// 		}
// 	}

// 	public class CV4BActions {
// 		private CV4B cv4b;
	
// 		public CV4BActions(HardwareMap hardwareMap) {
// 			cv4b = new CV4B(hardwareMap);
// 		}

// 		public Action setPosition(CV4B.Positions position) {
// 			return setPosition(position, 0);
// 		}

// 		public Action setPosition(CV4B.Positions position, double length) {
// 			return new Action() {
// 				private boolean initialized = false;
// 				private ElapsedTime timer = new ElapsedTime();

// 				@Override
// 				public boolean run(@NonNull TelemetryPacket packet) {
// 					if (!initialized) {
// 						cv4b.setPosition(position);
						
// 						initialized = true;
// 						return true;
// 					}

// 					if (timer.time() < length) {
// 						return true;
// 					}
// 					return false;
// 				}
// 			};
// 		}

// 		public void setPositionMethod(CV4B.Positions position) {
// 			cv4b.setPosition(position);
// 		}
// 	}

// 	public class ClawActions {
// 		private Claw claw;
	
// 		public CV4BActions(HardwareMap hardwareMap) {
// 			claw = new Claw(hardwareMap);
// 		}

// 		public Action setPosition(Claw.Positions position) {
// 			return setPosition(position, 0);
// 		}

// 		public Action setPosition(Claw.Positions position, double length) {
// 			return new Action() {
// 				private boolean initialized = false;
// 				private ElapsedTime timer = new ElapsedTime();

// 				@Override
// 				public boolean run(@NonNull TelemetryPacket packet) {
// 					if (!initialized) {
// 						claw.setPosition(position);
						
// 						initialized = true;
// 						return true;
// 					}

// 					if (timer.time() < length) {
// 						return true;
// 					}
// 					return false;
// 				}
// 			};
// 		}
// 	}
// }
