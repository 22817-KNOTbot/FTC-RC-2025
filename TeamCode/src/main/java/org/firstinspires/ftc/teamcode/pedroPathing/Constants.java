package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Constants {
	public static FollowerConstants followerConstants = new FollowerConstants()
			.mass(14.5);

	public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

	public static MecanumConstants driveConstants = new MecanumConstants()
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

	public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
			.forwardTicksToInches(0.0020153192)
			.strafeTicksToInches(0.0019831647)
			.turnTicksToInches(0.0020013505)
			.leftPodY(7.286)
			.rightPodY(-7.144)
			.strafePodX(-7.042)
			.leftEncoder_HardwareMapName("frontLeftMotor")
			.rightEncoder_HardwareMapName("backRightMotor")
			.strafeEncoder_HardwareMapName("frontRightMotor")
			.leftEncoderDirection(Encoder.REVERSE)
			.rightEncoderDirection(Encoder.REVERSE)
			.strafeEncoderDirection(Encoder.FORWARD);

	public static Follower createFollower(HardwareMap hardwareMap) {
		return new FollowerBuilder(followerConstants, hardwareMap)
				.pathConstraints(pathConstraints)
				.mecanumDrivetrain(driveConstants)
				.threeWheelLocalizer(localizerConstants)
				.build();
	}
}
