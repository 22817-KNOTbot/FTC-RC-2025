package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.scoring.Artifact.Pattern;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.teleop.Automations;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.BlueAlliance;
import org.firstinspires.ftc.teamcode.util.RedAlliance;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoComponents {
	private Alliance alliance;

	public AutoComponents(Alliance alliance) {
		this.alliance = alliance;
	}

	public static abstract class AutoState {
		public abstract String getNameString();

		public abstract AutoAction[] getAutoActions();
	}

	public static abstract class StartAutoState extends AutoState {
		public abstract Pose getStartPose();
	}

	public static abstract class AutoAction {
		public abstract String getNameString();

		public abstract AutoActionCommand getActionCommand();

		public abstract AutoState getResultingState();

		public abstract PathChain getPathChain(Follower follower, Pose startingPose);
	}

	public static interface AutoActionCommand {
		// Returns true if finished, false if not
		public boolean run(Follower follower, Automations automationHandler);
	}

	/*
	 * States
	 */

	public class StartLowState extends StartAutoState {
		public String getNameString() {
			return "Start Low";
		}

		public AutoAction[] getAutoActions() {
			return new AutoAction[] {
					new ShootFarAction(),
					new PrepareIntakeBottomAction(),
					new PrepareIntakeMiddleAction(),
					new PrepareIntakeTopAction(),
					// new PrepareIntakeLoadingZoneAction(),
					new LeaveLowAction(),
					new EndAction()
			};
		}

		public Pose getStartPose() {
			return new Pose(96, 9, Math.toRadians(0));
		}
	}

	public class StartUpState extends StartAutoState {
		public String getNameString() {
			return "Start Up";
		}

		public AutoAction[] getAutoActions() {
			return new AutoAction[] {
					new ShootCloseAction(),
					new PrepareIntakeMiddleAction(),
					new PrepareGateIntakeAction(),
					new PrepareIntakeTopAction(),
					new PrepareIntakeBottomAction(),
					new LeaveUpAction(),
					new EndAction()
			};
		}

		public Pose getStartPose() {
			return new Pose(123, 124, Math.toRadians(125));
		}
	}

	public class PrepareIntakeState extends AutoState {
		public String getNameString() {
			return "Prepare Intake";
		}

		public AutoAction[] getAutoActions() {
			return new AutoAction[] {
					new IntakePreparedAction(),
					new EndAction()
			};
		}
	}

	public class PrepareGateIntakeState extends AutoState {
		public String getNameString() {
			return "Prepare Intake";
		}

		public AutoAction[] getAutoActions() {
			return new AutoAction[] {
					new PreparedGateIntakeAction(),
					new EndAction()
			};
		}
	}

	public class IntakeState extends AutoState {
		public String getNameString() {
			return "Intake";
		}

		public AutoAction[] getAutoActions() {
			return new AutoAction[] {
					new ShootCloseAction(),
					new ShootFarAction(),
					new OpenGateAction(),
					new EndAction()
			};
		}
	}

	public class GateIntakeState extends AutoState {
		public String getNameString() {
			return "Intake";
		}

		public AutoAction[] getAutoActions() {
			return new AutoAction[] {
					new ShootCloseAction(),
					new ShootFarAction(),
					new EndAction()
			};
		}
	}

	public class OpenGateState extends AutoState {
		public String getNameString() {
			return "Open Gate";
		}

		public AutoAction[] getAutoActions() {
			return new AutoAction[] {
					new ShootCloseAction(),
					new ShootFarAction(),
					new EndAction()
			};
		}
	}

	public class ShootState extends AutoState {
		public String getNameString() {
			return "Shoot";
		}

		public AutoAction[] getAutoActions() {
			return new AutoAction[] {
					new ShootSortedAction(),
					new ShootUnsortedAction(),
					new EndAction()
			};
		}
	}

	public class ShootFinishState extends AutoState {
		public String getNameString() {
			return "Shoot Finish";
		}

		public AutoAction[] getAutoActions() {
			return new AutoAction[] {
					new PrepareIntakeMiddleAction(),
					new PrepareGateIntakeAction(),
					new PrepareIntakeTopAction(),
					new PrepareIntakeBottomAction(),
					new LeaveUpAction(),
					new EndAction()
			};
		}
	}

	/*
	 * Actions
	 */

	public class EndAction extends AutoAction {
		public String getNameString() {
			return "End";
		}

		public AutoActionCommand getActionCommand() {
			return (follower, automationHandler) -> {
				return false;
			};
		}

		public AutoState getResultingState() {
			return null;
		}

		public PathChain getPathChain(Follower follower, Pose startingPose) {
			return null;
		}
	}

	public class LeaveLowAction extends AutoAction {
		public String getNameString() {
			return "Leave Lower Zone And End";
		}

		public AutoActionCommand getActionCommand() {
			return (follower, automationHandler) -> {
				return false;
			};
		}

		public AutoState getResultingState() {
			return null;
		}

		public PathChain getPathChain(Follower follower, Pose startingPose) {
			if (alliance instanceof RedAlliance) {
				return AutoPaths.Red.getExitLowShootingZone(follower, startingPose);
			} else if (alliance instanceof BlueAlliance) {
				return AutoPaths.Blue.getExitLowShootingZone(follower, startingPose);
			}
			return null;
		}
	}

	public class LeaveUpAction extends AutoAction {
		public String getNameString() {
			return "Leave Upper Zone And End";
		}

		public AutoActionCommand getActionCommand() {
			return (follower, automationHandler) -> {
				return false;
			};
		}

		public AutoState getResultingState() {
			return null;
		}

		public PathChain getPathChain(Follower follower, Pose startingPose) {
			if (alliance instanceof RedAlliance) {
				return AutoPaths.Red.getExitUpperShootingZone(follower, startingPose);
			} else if (alliance instanceof BlueAlliance) {
				return AutoPaths.Blue.getExitLowShootingZone(follower, startingPose);
			}
			return null;
		}
	}

	public class PrepareIntakeBottomAction extends AutoAction {
		public String getNameString() {
			return "Prepare Intake Bottom";
		}

		public AutoActionCommand getActionCommand() {
			return new PrepareIntakeCommand();
		}

		public AutoState getResultingState() {
			return new PrepareIntakeState();
		}

		public PathChain getPathChain(Follower follower, Pose startingPose) {
			if (alliance instanceof RedAlliance) {
				return AutoPaths.Red.getBottomApproach(follower, startingPose);
			} else if (alliance instanceof BlueAlliance) {
				return AutoPaths.Blue.getBottomApproach(follower, startingPose);
			}
			return null;
		}
	}

	public class PrepareIntakeMiddleAction extends AutoAction {
		public String getNameString() {
			return "Prepare Intake Middle";
		}

		public AutoActionCommand getActionCommand() {
			return new PrepareIntakeCommand();
		}

		public AutoState getResultingState() {
			return new PrepareIntakeState();
		}

		public PathChain getPathChain(Follower follower, Pose startingPose) {
			if (alliance instanceof RedAlliance) {
				return AutoPaths.Red.getMiddleApproach(follower, startingPose);
			} else if (alliance instanceof BlueAlliance) {
				return AutoPaths.Blue.getMiddleApproach(follower, startingPose);
			}
			return null;
		}
	}

	public class PrepareGateIntakeAction extends AutoAction {
		public String getNameString() {
			return "Prepare Gate Intake Action";
		}

		public AutoActionCommand getActionCommand() {
			return new PrepareIntakeCommand();
		}

		public AutoState getResultingState() {
			return new PrepareGateIntakeState();
		}

		public PathChain getPathChain(Follower follower, Pose startingPose) {
			if (alliance instanceof RedAlliance) {
				return AutoPaths.Red.getGateIntake(follower, startingPose);
			} else if (alliance instanceof BlueAlliance) {
				return AutoPaths.Blue.getGateIntake(follower, startingPose);
			}
			return null;
		}
	}

	public class PrepareIntakeTopAction extends AutoAction {
		public String getNameString() {
			return "Prepare Intake Top";
		}

		public AutoActionCommand getActionCommand() {
			return new PrepareIntakeCommand();
		}

		public AutoState getResultingState() {
			return new PrepareIntakeState();
		}

		public PathChain getPathChain(Follower follower, Pose startingPose) {
			if (alliance instanceof RedAlliance) {
				return AutoPaths.Red.getTopApproach(follower, startingPose);
			} else if (alliance instanceof BlueAlliance) {
				return AutoPaths.Blue.getTopApproach(follower, startingPose);
			}
			return null;
		}
	}

	// public class PrepareIntakeLoadingZoneAction extends AutoAction {
	// public String getNameString() {
	// return "Prepare Intake Loading Zone";
	// }

	// public AutoActionCommand getActionCommand() {
	// return (follower, automationHandler) -> {
	// return !follower.isBusy();};
	// }

	// public AutoState getResultingState() {
	// return = new PrepareIntakeState();
	// }

	// public PathChain getPathChain(Follower follower, Pose startingPose) {
	// if (alliance instanceof RedAlliance) {
	// return AutoPaths.Red.PATHCHAIN;
	// } else if (alliance instanceof BlueAlliance) {
	// return AutoPaths.Blue.PATHCHAIN;
	// }
	// return null;
	// }
	// }

	public class IntakePreparedAction extends AutoAction {
		public String getNameString() {
			return "Intake Prepared Set";
		}

		public AutoActionCommand getActionCommand() {
			return new AutoActionCommand() {
				private boolean initialized = false;
				private ElapsedTime timer = new ElapsedTime();

				public boolean run(Follower follower, Automations automationHandler) {
					if (!initialized) {
						follower.setMaxPower(0.25);
						automationHandler.intakeEnableActions(true);
						timer.reset();
					}

					if (!follower.isBusy() || (timer.time() > 1.5 && follower.getVelocity().getMagnitude() < 0.2)) {
						follower.setMaxPower(1);
						return true;
					}

					return false;
				}
			};
		}

		public AutoState getResultingState() {
			return new IntakeState();
		}

		public PathChain getPathChain(Follower follower, Pose startingPose) {
			if (alliance instanceof RedAlliance) {
				return AutoPaths.Red.getIntakePreparedSet(follower, startingPose);
			} else if (alliance instanceof BlueAlliance) {
				return AutoPaths.Blue.getIntakePreparedSet(follower, startingPose);
			}
			return null;
		}
	}

	public class PreparedGateIntakeAction extends AutoAction {
		public String getNameString() {
			return "Prepared Gate Intake";
		}

		public AutoActionCommand getActionCommand() {
			return new AutoActionCommand() {
				private boolean initialized = false;
				private ElapsedTime timer = new ElapsedTime();

				public boolean run(Follower follower, Automations automationHandler) {
					if (!initialized) {
						follower.setMaxPower(0.25);
						automationHandler.intakeEnableActions(true);
						timer.reset();
					}

					if (!follower.isBusy() || (timer.time() > 2.5 && follower.getVelocity().getMagnitude() < 0.2)) {
						follower.setMaxPower(1);
						return true;
					}

					return false;
				}
			};
		}

		public AutoState getResultingState() {
			return new GateIntakeState();
		}

		public PathChain getPathChain(Follower follower, Pose startingPose) {
			if (alliance instanceof RedAlliance) {
				return AutoPaths.Red.getIntakePreparedSet(follower, startingPose);
			} else if (alliance instanceof BlueAlliance) {
				return AutoPaths.Blue.getIntakePreparedSet(follower, startingPose);
			}
			return null;
		}
	}

	public class ShootFarAction extends AutoAction {
		public String getNameString() {
			return "Shoot Far";
		}

		public AutoActionCommand getActionCommand() {
			return new PostIntakeCommand();
		}

		public AutoState getResultingState() {
			return new ShootState();
		}

		public PathChain getPathChain(Follower follower, Pose startingPose) {
			if (alliance instanceof RedAlliance) {
				return AutoPaths.Red.getLowLaunch(follower, startingPose);
			} else if (alliance instanceof BlueAlliance) {
				return AutoPaths.Blue.getLowLaunch(follower, startingPose);
			}
			return null;
		}
	}

	public class ShootCloseAction extends AutoAction {
		public String getNameString() {
			return "Shoot Close";
		}

		public AutoActionCommand getActionCommand() {
			return new PostIntakeCommand();
		}

		public AutoState getResultingState() {
			return new ShootState();
		}

		public PathChain getPathChain(Follower follower, Pose startingPose) {
			if (alliance instanceof RedAlliance) {
				return AutoPaths.Red.getUpLaunch(follower, startingPose);
			} else if (alliance instanceof BlueAlliance) {
				return AutoPaths.Blue.getUpLaunch(follower, startingPose);
			}
			return null;
		}
	}

	public class GateIntakeShootCloseAction extends AutoAction {
		public String getNameString() {
			return "Shoot Close";
		}

		public AutoActionCommand getActionCommand() {
			return new PostIntakeCommand();
		}

		public AutoState getResultingState() {
			return new ShootState();
		}

		public PathChain getPathChain(Follower follower, Pose startingPose) {
			if (alliance instanceof RedAlliance) {
				return AutoPaths.Red.getUpLaunch(follower, startingPose);
			} else if (alliance instanceof BlueAlliance) {
				return AutoPaths.Blue.getUpLaunch(follower, startingPose);
			}
			return null;
		}
	}

	public class OpenGateAction extends AutoAction {
		public String getNameString() {
			return "Open Gate";
		}

		public AutoActionCommand getActionCommand() {
			return new PostIntakeCommand();
		}

		public AutoState getResultingState() {
			return new OpenGateState();
		}

		public PathChain getPathChain(Follower follower, Pose startingPose) {
			if (alliance instanceof RedAlliance) {
				return AutoPaths.Red.getPushGate(follower, startingPose);
			} else if (alliance instanceof BlueAlliance) {
				return AutoPaths.Blue.getPushGate(follower, startingPose);
			}
			return null;
		}
	}

	public class ShootSortedAction extends AutoAction {
		public String getNameString() {
			return "Shoot Sorted";
		}

		public AutoActionCommand getActionCommand() {
			return new AutoActionCommand() {
				private boolean initialized = false;
				private ElapsedTime shootingTimer = new ElapsedTime();
				private boolean shootingTimerSet = false;

				public boolean run(Follower follower, Automations automationHandler) {
					if (follower.isBusy())
						return false;
					if (!initialized) {
						automationHandler.setTransferMode(Storage.TransferMode.FULL_SPIN);
						automationHandler.setRapidFire(true);
						automationHandler.setIgnoreVelocity(false);
						Pattern pattern = automationHandler.getArtifactPattern();
						if (pattern == null || pattern.getPattern() == null || automationHandler
								.prepareOrShootArtifactSequence(pattern.getPattern()) == Storage.TurnDirection.NONE) {
							automationHandler.shootActiveArtifact(true);
						}

						initialized = true;
					}

					if (automationHandler.getStorageState() == Automations.StorageState.WAITING) {
						return true;
					} else if (automationHandler.getTransferState() == Storage.TransferState.RAMP_OUT) {
						if (!shootingTimerSet) {
							shootingTimer.reset();
							shootingTimerSet = true;
						}
					}

					if (shootingTimerSet && shootingTimer.time() > 5) {
						automationHandler.setIgnoreVelocity(true);
					}

					return false;
				}
			};
		}

		public AutoState getResultingState() {
			return new ShootFinishState();
		}

		public PathChain getPathChain(Follower follower, Pose startingPose) {
			return null;
		}
	}

	public class ShootUnsortedAction extends AutoAction {
		public String getNameString() {
			return "Shoot Unsorted";
		}

		public AutoActionCommand getActionCommand() {
			return (follower, automationHandler) -> {
				return !follower.isBusy();
			};
		}

		public AutoState getResultingState() {
			return new ShootFinishState();
		}

		public PathChain getPathChain(Follower follower, Pose startingPose) {
			return null;
		}
	}

	/*
	 * Reusable action commands
	 */

	public class PrepareIntakeCommand implements AutoActionCommand {
		public boolean run(Follower follower, Automations automationHandler) {
			return follower.atParametricEnd();
		}
	}

	public class PostIntakeCommand implements AutoActionCommand {
		public boolean run(Follower follower, Automations automationHandler) {
			if (follower.getPathCompletion() >= 0.9) {
				automationHandler.intakeEnable(false);
			}
			return !follower.isBusy() && automationHandler.getStorageState() == Automations.StorageState.WAITING;
		}
	}
}