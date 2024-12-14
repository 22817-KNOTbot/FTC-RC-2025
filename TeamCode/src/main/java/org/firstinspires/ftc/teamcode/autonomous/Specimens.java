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
import org.firstinspires.ftc.teamcode.subsystems.CV4B;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;

@Config
@Autonomous(name = "4 Specimen Auto", group = "Official")
public class Specimens extends LinearOpMode {
	public static double initialPoseX = 24;
	public static double initialPoseY = -61;
	public static double initialPoseHeading = 270;
    private Mechanisms mechanismControl;
    // GRAB: 19 from bottom
    // HANG: 21 from subm
    	
	@Override
	public void runOpMode() {
		Pose2d initialPose = new Pose2d(initialPoseX, initialPoseY, Math.toRadians(initialPoseHeading));
		MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
		// AprilTagDrive drive = new AprilTagDrive(hardwareMap, initialPose, aprilTagProcessor, true);
        mechanismControl = new Mechanisms(hardwareMap);
        // -48 y for spec hang

        TrajectoryActionBuilder firstHangSpecimen = drive.actionBuilder(new Pose2d(24, -61, Math.toRadians(270)))
            .setReversed(true)
            .splineToConstantHeading(new Vector2d(6, -39), Math.toRadians(90));

        TrajectoryActionBuilder pushSamples = drive.actionBuilder(new Pose2d(4, -39, Math.toRadians(270)))
            // .splineToConstantHeading(new Vector2d(36, -28), Math.toRadians(90))
            .splineTo(new Vector2d(39, -20), Math.toRadians(90))
            .splineToSplineHeading(new Pose2d(42, -12, Math.toRadians(0)), Math.toRadians(290))
            .splineToConstantHeading(new Vector2d(44, -60), Math.toRadians(270))

            .splineToConstantHeading(new Vector2d(50, -28), Math.toRadians(90))
            .splineToConstantHeading(new Vector2d(57, -10), Math.toRadians(270))
            .splineToConstantHeading(new Vector2d(57, -60), Math.toRadians(270))

            .splineToConstantHeading(new Vector2d(60, -28), Math.toRadians(90))
            .splineToConstantHeading(new Vector2d(64, -10), Math.toRadians(270))
            .splineToConstantHeading(new Vector2d(64, -53), Math.toRadians(270))

            // .setReversed(true)
            .splineToSplineHeading(new Pose2d(36, -58, Math.toRadians(90)), Math.toRadians(220))
;	

        TrajectoryActionBuilder grabSpecimen = drive.actionBuilder(new Pose2d(4, -43, Math.toRadians(270)))
            .splineToLinearHeading(new Pose2d(36, -58, Math.toRadians(90)), Math.toRadians(270));
            
        TrajectoryActionBuilder hangSpecimen = drive.actionBuilder(new Pose2d(36, -58, Math.toRadians(90)))
            .splineToLinearHeading(new Pose2d(6, -43, Math.toRadians(272)), Math.toRadians(90));

        TrajectoryActionBuilder pushSpecimen = drive.actionBuilder(new Pose2d(6, -43, Math.toRadians(272)))
            .strafeTo(new Vector2d(4, -43));

        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(6, -43, Math.toRadians(270)))
            .splineTo(new Vector2d(40, -58), Math.toRadians(345));

		waitForStart();

		Actions.runBlocking(
            new SequentialAction(
                firstHangSpecimen.build(),
                pushSpecimen.build(),
                pushSamples.build(),
                hangSpecimen.build(),
                pushSpecimen.build(),
                grabSpecimen.build(),
                hangSpecimen.build(),
                pushSpecimen.build(),
                grabSpecimen.build(),
                hangSpecimen.build(),
                pushSpecimen.build(),
                park.build()    
            )
        );
		// Actions.runBlocking(
		// 	new SequentialAction(
        //         new ParallelAction(
        //             mechanismControl.init(),
        //             firstHangSpecimen.build()
        //         ),
        //         pushSpecimen.build(),
        //         mechanismControl.hangSpecimen(),
        //         mechanismControl.releaseSpecimen(),
                        
        //         new ParallelAction(
        //             pushSamples.build(),
        //             mechanismControl.grabSpecimen(9400) // Grab specimen
        //         ),
		// 		hangSpecimen.build(),
        //         pushSpecimen.build(),
        //         mechanismControl.hangSpecimen(), // Hang specimen
        //         mechanismControl.releaseSpecimen(), // Release specimen

        //         new ParallelAction(
        //             grabSpecimen.build(),
        //             mechanismControl.grabSpecimen(1400) // Grab specimen
        //         ),
		// 		hangSpecimen.build(),
        //         pushSpecimen.build(),
        //         mechanismControl.hangSpecimen(), // Hang specimen
        //         mechanismControl.releaseSpecimen(), // Release specimen

        //         new ParallelAction(
        //             grabSpecimen.build(),
        //             mechanismControl.grabSpecimen(1400) // Grab specimen
        //         ),
		// 		hangSpecimen.build(),
        //         pushSpecimen.build(),
        //         mechanismControl.hangSpecimen(), // Hang specimen
        //         mechanismControl.releaseSpecimen(), // Release specimen

        //         park.build()
		// 	)
		// );

		// Store pose for future use
		OpModeStorage.pose = drive.pose;
	}

	public class Mechanisms {
		private DcMotor slideMotorLeft;
		private DcMotor slideMotorRight;
        private CV4B cv4b;
        private Servo clawServo;
        private Servo flipServo;

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
            cv4b = new CV4B(hardwareMap);
            clawServo = hardwareMap.get(Servo.class, "clawServo");
            clawServo.setDirection(Servo.Direction.REVERSE);
            clawServo.scaleRange(0, 0.01);    
            flipServo = hardwareMap.get(Servo.class, "flipServo");
		}

        public Action init() {
            return new Action() {
                private boolean initialized = false;
                private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        flipServo.setPosition(0.84);
                        slideMotorLeft.setPower(1);
						slideMotorRight.setPower(1);
                        clawServo.setPosition(1);

                        initialized = true;
                        return true;
                    }
                    if (timer.time() > 750) {
                        setPosition(CV4B.Positions.SPECIMEN_GRAB);
						slideMotorLeft.setTargetPosition(2100);
						slideMotorRight.setTargetPosition(2100);
                    } else if (timer.time() > 1600 && !slideMotorLeft.isBusy() && !slideMotorRight.isBusy()) {
                        flipServo.setPosition(0.76);
                        return false;
                    }
                    return true;
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
                        clawServo.setPosition(0);

                        initialized = true;
                        return true;
                    }
                    if (timer.time() > (delayMs + 1000)) {
                        clawServo.setPosition(1);
                    } else if (timer.time() > (delayMs + 1300)) {
						slideMotorLeft.setTargetPosition(2100);
						slideMotorRight.setTargetPosition(2100);
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
						slideMotorLeft.setTargetPosition(1500);
						slideMotorRight.setTargetPosition(1500);
                        setPosition(CV4B.Positions.SPECIMEN_HANG);

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
                    slideMotorLeft.setTargetPosition(0);
                    slideMotorRight.setTargetPosition(0);
                    setPosition(CV4B.Positions.SPECIMEN_GRAB);

                    return false;
                }
            };
        }

        public Action extendIntake() {
            return new Action() {
                private boolean initialized = false;
    
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        flipServo.setPosition(0.887);
                        return false;
                    }
                    return false;
                }
            };
        }

        public void setPosition(CV4B.Positions position) {
            cv4b.setPosition(position);
        }
	}
}
