package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public class Constants {
	public static double forwardTicksToInches = 0.001997263346781042;
	public static double strafeTicksToInches = 0.00201150257521259037;
	public static double turnTicksToInches = 0.0020001;
	public static double leftPodY = 7.113188976;
	public static double rightPodY = -7.113188976;
	public static double strafePodX = -6.5;

	public final static FollowerConstants followerConstants = new FollowerConstants()
			.mass(14.5);

	public final static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

	public final static MecanumConstants driveConstants = new MecanumConstants()
			.maxPower(1)
			.leftFrontMotorName("frontLeftMotor")
			.rightFrontMotorName("frontRightMotor")
			.leftRearMotorName("backLeftMotor")
			.rightRearMotorName("backRightMotor")
			.leftFrontMotorDirection(DcMotor.Direction.REVERSE)
			.leftRearMotorDirection(DcMotor.Direction.REVERSE)
			.rightFrontMotorDirection(DcMotor.Direction.FORWARD)
			.rightRearMotorDirection(DcMotor.Direction.FORWARD)
			.useBrakeModeInTeleOp(true);

	public final static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
			.forwardTicksToInches(0.001987263346781042)
			.strafeTicksToInches(0.0019750257521259037)
			.turnTicksToInches(0.0019990158631831103)
			.leftPodY(7.113188976)
			.rightPodY(-7.113188976)
			.strafePodX(-7.01423031496)
			.leftEncoder_HardwareMapName("frontLeftMotor")
			.rightEncoder_HardwareMapName("shooterMotorRight")
			.strafeEncoder_HardwareMapName("backLeftMotor")
			.leftEncoderDirection(Encoder.REVERSE)
			.rightEncoderDirection(Encoder.REVERSE)
			.strafeEncoderDirection(Encoder.FORWARD);
			// .IMU_HardwareMapName("imu")
			// .IMU_Orientation(new RevHubOrientationOnRobot(
			// 		RevHubOrientationOnRobot.LogoFacingDirection.LEFT, 
			// 		RevHubOrientationOnRobot.UsbFacingDirection.UP));

	public static Follower createFollower(HardwareMap hardwareMap) {
		return new FollowerBuilder(followerConstants, hardwareMap)
				.pathConstraints(pathConstraints)
				.mecanumDrivetrain(driveConstants)
				.threeWheelLocalizer(
						new ThreeWheelConstants()
								.forwardTicksToInches(forwardTicksToInches)
								.strafeTicksToInches(strafeTicksToInches)
								.turnTicksToInches(turnTicksToInches)
								.leftPodY(leftPodY)
								.rightPodY(rightPodY)
								.strafePodX(strafePodX)
								// .strafePodX(-7.01423031496)
								.leftEncoder_HardwareMapName("frontLeftMotor")
								.rightEncoder_HardwareMapName("shooterMotorRight")
								.strafeEncoder_HardwareMapName("backLeftMotor")
								.leftEncoderDirection(Encoder.REVERSE)
								.rightEncoderDirection(Encoder.REVERSE)
								.strafeEncoderDirection(Encoder.FORWARD)
				)
				.build();
	}
}
