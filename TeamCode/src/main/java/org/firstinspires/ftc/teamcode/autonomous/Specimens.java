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
@Autonomous(name = "Specimens Testing Auto", group = "Testing")
public class Specimens extends LinearOpMode {
	public static double initialPoseX = -23;
	public static double initialPoseY = -62;
	public static double initialPoseHeading = 0;
    private Mechanisms mechanismControl;
    
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
        mechanismControl = new Mechanisms(hardwareMap);

        Action pushSamples = drive.actionBuilder(new Pose2d(24, -62, Math.toRadians(180)))
            .setReversed(true)
            .splineToConstantHeading(new Vector2d(36, -28), Math.toRadians(90))
            .splineToConstantHeading(new Vector2d(43, -10), Math.toRadians(270))
            .splineToConstantHeading(new Vector2d(43, -53), Math.toRadians(270))

            .splineToConstantHeading(new Vector2d(45, -28), Math.toRadians(90))
            .splineToConstantHeading(new Vector2d(52, -10), Math.toRadians(270))
            .splineToConstantHeading(new Vector2d(52, -53), Math.toRadians(270))

            .splineToConstantHeading(new Vector2d(54, -28), Math.toRadians(90))
            .splineToConstantHeading(new Vector2d(60, -10), Math.toRadians(270))
            .splineToConstantHeading(new Vector2d(60, -53), Math.toRadians(270))
            .build();
        
        Action firstGrabSpecimen = drive.actionBuilder(new Pose2d(60, -53, Math.toRadians(180)))
            .splineToLinearHeading(new Pose2d(36, -58, Math.toRadians(90)), Math.toRadians(270))
            .build();

        Action grabSpecimen = drive.actionBuilder(new Pose2d(8, -35, Math.toRadians(270)))
            .splineToLinearHeading(new Pose2d(36, -58, Math.toRadians(90)), Math.toRadians(270))
            .build();
            
        Action hangSpecimen = drive.actionBuilder(new Pose2d(36, -58, Math.toRadians(90)))
            .splineToLinearHeading(new Pose2d(10, -35, Math.toRadians(270)), Math.toRadians(90))
            .build();

        Action pushSpecimen = drive.actionBuilder(new Pose2d(10, -35, Math.toRadians(270)))
            .strafeTo(new Vector2d(8, -35))
            .build();

        Action park = drive.actionBuilder(new Pose2d(8, -35, Math.toRadians(270)))
            .splineToLinearHeading(new Pose2d(36, -58, Math.toRadians(90)), Math.toRadians(270))
            .build();

		waitForStart();

		Actions.runBlocking(
			new SequentialAction(
                pushSamples,
                mechanismControl.init(),

                new ParallelAction(
                    firstGrabSpecimen,
                    mechanismControl.grabSpecimen(800) // Grab specimen
                ),
				hangSpecimen,
                mechanismControl.hangSpecimen(), // Hang specimen
                pushSpecimen,
                mechanismControl.releaseSpecimen(), // Release specimen

                new ParallelAction(
                    grabSpecimen,
                    mechanismControl.grabSpecimen(1000) // Grab specimen
                ),
				hangSpecimen,
                mechanismControl.hangSpecimen(), // Hang specimen
                pushSpecimen,
                mechanismControl.releaseSpecimen(), // Release specimen

                new ParallelAction(
                    grabSpecimen,
                    mechanismControl.grabSpecimen(1000) // Grab specimen
                ),
				hangSpecimen,
                mechanismControl.hangSpecimen(), // Hang specimen
                pushSpecimen,
                mechanismControl.releaseSpecimen(), // Release specimen

                new ParallelAction(
                    grabSpecimen,
                    mechanismControl.grabSpecimen(1000) // Grab specimen
                ),
				hangSpecimen,
                mechanismControl.hangSpecimen(), // Hang specimen
                pushSpecimen,
                mechanismControl.releaseSpecimen(), // Release specimen

                new ParallelAction(
                    grabSpecimen,
                    mechanismControl.grabSpecimen(1000) // Grab specimen
                ),
				hangSpecimen,
                mechanismControl.hangSpecimen(), // Hang specimen
                pushSpecimen,
                mechanismControl.releaseSpecimen(), // Release specimen

                park
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

	public class Mechanisms {
		private DcMotor slideMotorLeft;
		private DcMotor slideMotorRight;
        private Servo cv4bLeftServo;
		private Servo cv4bRightServo;
		private Servo cv4bCoaxialServo;
        private Servo clawServo;

		public Mechanisms(HardwareMap hardwareMap) {
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
            cv4bLeftServo = hardwareMap.get(Servo.class, "cv4bLeftServo");
			cv4bRightServo = hardwareMap.get(Servo.class, "cv4bRightServo");
			cv4bLeftServo.setDirection(Servo.Direction.FORWARD);
			cv4bRightServo.setDirection(Servo.Direction.REVERSE);
			cv4bCoaxialServo = hardwareMap.get(Servo.class, "cv4bCoaxialServo");
			cv4bCoaxialServo.setDirection(Servo.Direction.REVERSE);
            clawServo = hardwareMap.get(Servo.class, "clawServo");
            clawServo.setDirection(Servo.Direction.REVERSE);
            clawServo.scaleRange(0, 0.2);    
		}

        public Action init() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    setPosition(Cv4bPosition.SPECIMEN_GRAB);
                    return false;
                }
            };
        }

        public Action grabSpecimen(int delayMs) {
            return new Action() {
                private boolean initialized = false;
                private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (timer.time() < delayMs) return true;
                    if (!initialized) {
                        slideMotorLeft.setPower(1);
						slideMotorRight.setPower(1);
						slideMotorLeft.setTargetPosition(500);
						slideMotorRight.setTargetPosition(500);
                        clawServo.setPosition(0);

                        initialized = true;
                        return true;
                    }
                    if (timer.time() > (delayMs + 1000)) {
                        clawServo.setPosition(1);
                    } else if (timer.time() > (delayMs + 1300)) {
						slideMotorLeft.setTargetPosition(2500);
						slideMotorRight.setTargetPosition(2500);
                        return false;
                    }
                    return true;
                }
            };
        }

        public Action hangSpecimen() {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
					if (!initialized) {
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

        public Action releaseSpecimen() {
            return new Action () {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    clawServo.setPosition(0);
                    slideMotorLeft.setTargetPosition(500);
                    slideMotorRight.setTargetPosition(500);

                    return false;
                }
            };
        }

        public void setPosition(Cv4bPosition position) {
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
