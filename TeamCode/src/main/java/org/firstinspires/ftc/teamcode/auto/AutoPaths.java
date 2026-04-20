package org.firstinspires.ftc.teamcode.auto;

import java.util.ArrayList;
import java.util.List;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathPoint;

public class AutoPaths {
	public static class Red {
		// Close Paths
		public static PathChain getBottomApproach(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
					.addPath(
							new BezierCurve(startingPose,
            						new Pose(90.000, 35.000),
            						new Pose(94.000, 35.000)))
					.setConstantHeadingInterpolation(Math.toRadians(0))
        			.build();
		}

		public static PathChain getMiddleApproach(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
					.addPath(
							new BezierLine(startingPose, new Pose(94.500, 54.000)))
					.setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(0))
					.setVelocityConstraint(0.5)
					.build();
		}

		public static PathChain getTopApproach(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
					.addPath(
							new BezierLine(startingPose, new Pose(90.000, 83.500)))
					.setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(0))
					.build();
		}
		
		public static PathChain getIntakePreparedSet(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
					.addPath(
							new BezierLine(startingPose, startingPose.withX(startingPose.getX() + 35)))
					.setConstantHeadingInterpolation(Math.toRadians(0))
					// .setGlobalDeceleration(1.5)
					.build();
		}

		public static PathChain getLoadingZoneApproach(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
				.addPath(
					new BezierLine(startingPose, new Pose(94.000, 11.000)))
				.setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(0))
				.build();
		}

		public static PathChain getLoadingZoneApproachClose(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
				.addPath(
					new BezierLine(startingPose, new Pose(90.000, 11.000)))
				.setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(0))
				.build();
		}

		public static PathChain getGateIntakeApproach(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
					.addPath(
        					new BezierCurve(
									startingPose,
									new Pose(90.000, 69.000),
									new Pose(124.000, 69.000)
							))
					// .setBrakingStrength(0.5)
					.setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(0))
					.build();
		}

		public static PathChain getGateIntakePrepared(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
					.addPath(
						new BezierLine(startingPose, new Pose(127.000, 58.000))
					)
					// .setBrakingStrength(0.5)
					.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
					.build();
		}

		public static PathChain getGateIntake(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
					.addPath(
						new BezierCurve(
							startingPose, 
							new Pose(87.000, 53.000),
							new Pose(120.000, 55.000),
							new Pose(128.000, 55.000)
						)
					)
					// .setBrakingStrength(0.5)
					// .setGlobalDeceleration()
					.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(20))
					.build();
		}

		public static PathChain getUpLaunch(Follower follower, Pose startingPose) {
			if (startingPose.getY() >= 60) {
				return follower.pathBuilder()
						.addPath(
								new BezierLine(startingPose, new Pose(84.000, 84.000)))
						.setHeadingInterpolation(
								HeadingInterpolator.piecewise(
										new HeadingInterpolator.PiecewiseNode(0, 0.25,
												HeadingInterpolator.constant(startingPose.getHeading() + Math.PI)),
										new HeadingInterpolator.PiecewiseNode(0.25, 1, HeadingInterpolator.tangent)))
						.setReversed()
						.build();
			} else {
				return follower.pathBuilder()
						.addPath(
								new BezierCurve(
										startingPose, 
										new Pose(90.000, 60.000),
										new Pose(84.000, 84.000)
								))
						.setHeadingInterpolation(
								HeadingInterpolator.piecewise(
										new HeadingInterpolator.PiecewiseNode(0, 0.25,
												HeadingInterpolator.constant(startingPose.getHeading() + Math.PI)),
										new HeadingInterpolator.PiecewiseNode(0.25, 1, HeadingInterpolator.tangent)))
						.setReversed()
						.build();
			}
		}

		public static PathChain getUpLaunch(Follower follower, Pose startingPose, double targetHeadingDeg) {
			return follower.pathBuilder()
					.addPath(
							new BezierLine(startingPose, new Pose(84.000, 84.000)))
					.setHeadingInterpolation(
							HeadingInterpolator.piecewise(
									new HeadingInterpolator.PiecewiseNode(0, 0.15,
											HeadingInterpolator.constant(startingPose.getHeading() + Math.PI)),
									new HeadingInterpolator.PiecewiseNode(0.15, 1,
											HeadingInterpolator.linear(startingPose.getHeading(),
													Math.toRadians(targetHeadingDeg)))))
					.setReversed()
					.build();
		}

		public static PathChain getExitUpperShootingZone(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
					.addPath(
							new BezierLine(startingPose, new Pose(84.000, 105.000)))
					.setConstantHeadingInterpolation(Math.toRadians(340))
					.build();
		}

		// Other Paths

		public static PathChain getPushGate(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
					.addPath(
							new BezierCurve(
									startingPose,
									new Pose(100.000, 54.000),
									new Pose(128.000, 59.000)))
					.setConstantHeadingInterpolation(Math.toRadians(0))
					.build();
		}

		public static PathChain getLowLaunch(Follower follower, Pose startingPose) {
			if (startingPose.getY() >= 40) {
				return follower.pathBuilder()
						.addPath(
								new BezierCurve(startingPose,
										startingPose.withX(96),
										new Pose(96.000, 11.000)))
						.setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(0))
						.build();
			} else {
				return follower.pathBuilder()
						.addPath(
								new BezierLine(startingPose, new Pose(96.000, 11.000)))
						.setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(0))
						.build();
			}
		}

		public static PathChain getLowLaunchCurved(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
					.addPath(
							new BezierCurve(startingPose,
									startingPose.withX(96),
									new Pose(96.000, 11.000)))
					.setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(0))
					.build();
		}

		public static PathChain getExitLowShootingZone(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
					.addPath(
							new BezierLine(startingPose, new Pose(115.000, 11.000)))
					.setConstantHeadingInterpolation(Math.toRadians(0))
					.build();
		}

		public static PathChain getExitUpShootingZone(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
					.addPath(
							new BezierLine(startingPose, new Pose(96.000, 70.000)))
					.setConstantHeadingInterpolation(Math.toRadians(0))
					.build();
		}
	}

	public static class Blue {
		public static PathChain getBottomApproach(Follower follower, Pose startingPose) {
			// return mirrorPathChain(Red.getBottomApproach(follower, startingPose.mirror()), follower);
			return mirrorPathChain(follower.pathBuilder()
					.addPath(
							new BezierCurve(startingPose,
									new Pose(90.000, 35.000),
									new Pose(91.000, 35.000)))
					.setConstantHeadingInterpolation(Math.toRadians(0))
					.build(), follower);
		}

		public static PathChain getMiddleApproach(Follower follower, Pose startingPose) {
			return mirrorPathChain(Red.getMiddleApproach(follower, startingPose.mirror()), follower);
		}

		public static PathChain getPushGate(Follower follower, Pose startingPose) {
			return mirrorPathChain(Red.getPushGate(follower, startingPose.mirror()), follower);
		}

		public static PathChain getTopApproach(Follower follower, Pose startingPose) {
			return mirrorPathChain(Red.getTopApproach(follower, startingPose.mirror()), follower);
		}

		public static PathChain getIntakePreparedSet(Follower follower, Pose startingPose) {
			return mirrorPathChain(Red.getIntakePreparedSet(follower, startingPose.mirror()), follower);
		}

		public static PathChain getLoadingZoneApproach(Follower follower, Pose startingPose) {
			return mirrorPathChain(Red.getLoadingZoneApproach(follower, startingPose.mirror()), follower);
		}

		public static PathChain getExitUpperShootingZone(Follower follower, Pose startingPose) {
			return mirrorPathChain(Red.getExitUpperShootingZone(follower, startingPose.mirror()), follower);
		}

		public static PathChain getGateIntakeApproach(Follower follower, Pose startingPose) {
			return mirrorPathChain(Red.getGateIntakeApproach(follower, startingPose.mirror()), follower);
		}

		public static PathChain getGateIntakePrepared(Follower follower, Pose startingPose) {
			return mirrorPathChain(Red.getGateIntakePrepared(follower, startingPose.mirror()), follower);
		}

		public static PathChain getGateIntake(Follower follower, Pose startingPose) {
			// return mirrorPathChain(Red.getGateIntake(follower, startingPose.mirror()), follower);
			return mirrorPathChain(follower.pathBuilder()
					.addPath(
							new BezierCurve(
									startingPose,
									new Pose(87.000, 53.000),
									new Pose(120.000, 55.000),
									new Pose(127.000, 56.000)))
					// .setBrakingStrength(0.5)
					// .setGlobalDeceleration()
					.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(20))
					.build(), follower
			);
		}

		public static PathChain getLowLaunch(Follower follower, Pose startingPose) {
			return mirrorPathChain(Red.getLowLaunch(follower, startingPose.mirror()), follower);
		}

		public static PathChain getUpLaunch(Follower follower, Pose startingPose) {
			return mirrorPathChain(Red.getUpLaunch(follower, startingPose.mirror()), follower);
		}

		public static PathChain getUpLaunch(Follower follower, Pose startingPose, double targetHeadingDeg) {
			return mirrorPathChain(Red.getUpLaunch(follower, startingPose.mirror(), targetHeadingDeg + 180), follower);
		}

		public static PathChain getExitLowShootingZone(Follower follower, Pose startingPose) {
			return mirrorPathChain(Red.getExitLowShootingZone(follower, startingPose.mirror()), follower);
		}

		public static PathChain getExitUpShootingZone(Follower follower, Pose startingPose) {
			return mirrorPathChain(Red.getExitUpShootingZone(follower, startingPose.mirror()), follower);
		}
	}

	public static PathChain mirrorPathChain(PathChain pathChain, Follower follower) {
		final double FIELD_LENGTH = 146;

		PathBuilder newPathBuilder = follower.pathBuilder();
		for (int i = 0; i < pathChain.size(); i++) {
			Path path = pathChain.getPath(i);
			List<Pose> controlPoints = path.getControlPoints();
			List<Pose> newControlPoints = new ArrayList<>();
			for (Pose controlPoint : controlPoints) {
				newControlPoints.add(mirrorPose(controlPoint, FIELD_LENGTH));
			}

			Path newPath;
			if (newControlPoints.size() >= 3) {
				newPath = new Path(new BezierCurve(newControlPoints, path.getConstraints()), path.getConstraints());
			} else {
				newPath = new Path(new BezierLine(newControlPoints.get(0), newControlPoints.get(1)), path.getConstraints());
			}
			newPath.setHeadingInterpolation(closestPoint -> {
				Pose mirroredPose = mirrorPose(closestPoint.getPose(), FIELD_LENGTH);
				Vector mirroredVector = closestPoint.tangentVector.copy();
				mirroredVector.setTheta(MathFunctions.normalizeAngle(Math.PI - mirroredVector.getTheta()));
				PathPoint mirroredClosestPoint = new PathPoint(closestPoint.tValue, mirroredPose, mirroredVector);
				return MathFunctions.normalizeAngle(Math.PI - path.getHeadingInterpolator().interpolate(mirroredClosestPoint));
			});
			newPathBuilder.addPath(newPath);
		}

		return newPathBuilder.build();
	}

	public static Pose mirrorPose(Pose pose, double fieldLength) {
		Pose k = pose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
		return new Pose(fieldLength - k.getX(), k.getY(), MathFunctions.normalizeAngle(Math.PI - k.getHeading()),
				PedroCoordinates.INSTANCE);
	}
}
