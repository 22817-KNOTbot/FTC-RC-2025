package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods;
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
			.mass(14.06136347)
			.headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.08, 0.025))
			.predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.05, 0.05896956845946485, 0.0022047887068822377))
			.centripetalScaling(0.0);

	public final static PathConstraints pathConstraints = new PathConstraints(0.97, 100, 0.98, 1);

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
			.useBrakeModeInTeleOp(true)
			.xVelocity(76.27418674258735)
			.yVelocity(58.68122875784327);

	public final static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-DistanceUnit.INCH.fromMm(122.5))
            .strafePodX(DistanceUnit.INCH.fromMm(100))
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
			.encoderResolution(GoBildaOdometryPods.goBILDA_4_BAR_POD);

	public static Follower createFollower(HardwareMap hardwareMap) {
		return new FollowerBuilder(followerConstants, hardwareMap)
				.pathConstraints(pathConstraints)
				.mecanumDrivetrain(driveConstants)
				.pinpointLocalizer(localizerConstants)
				.build();
	}
}
