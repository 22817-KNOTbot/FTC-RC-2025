package org.firstinspires.ftc.teamcode.auto;

import java.util.concurrent.Callable;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.BlueAlliance;
import org.firstinspires.ftc.teamcode.util.RedAlliance;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class AutoComponents {
	private Alliance alliance;

	public AutoComponents(Alliance alliance) {
		this.alliance = alliance;
	}

	public static abstract class AutoState {
		private AutoState[] parents;
		public abstract String getNameString();
		public abstract AutoAction[] getAutoActions();
		
		public AutoState[] getParents()  {
			return parents;
		}
		
		public void setParents(AutoState[] parents) {
			this.parents = parents;
		}
	}

	public static abstract class StartAutoState extends AutoState {
		public abstract Pose getStartPose(Alliance alliance);
	}
	
	public static abstract class AutoAction {
		private AutoState[] parents;
		public abstract String getNameString();
		public abstract Callable<Void> getCallable();
		public abstract AutoState getResultingState();
		public abstract PathChain getPathChain(Follower follower, Pose startingPose);

		public AutoState[] getParents()  {
			return parents;
		}

		public void setParents(AutoState[] parents) {
			this.parents = parents;
		}
	}

	/*
	 * States
	 */

	public class StartLowState extends StartAutoState {
		public String getNameString() {
			return "Start";
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

		public Pose getStartPose(Alliance alliance) {
			Pose startPose = new Pose(96, 9, Math.toRadians(0));
			if (alliance instanceof BlueAlliance) {
				startPose = startPose.mirror();
			}
			return startPose;
		}
	}

	public class StartUpState extends StartAutoState {
		public String getNameString() {
			return "Start";
		}

		public AutoAction[] getAutoActions() {
			return new AutoAction[] {
				new ShootCloseAction(),
				new PrepareIntakeBottomAction(),
				new PrepareIntakeMiddleAction(),
				new PrepareIntakeTopAction(),
				// new PrepareIntakeLoadingZoneAction(),
				new LeaveUpAction(),
				new EndAction()
			};
		}

		public Pose getStartPose(Alliance alliance) {
			Pose startPose = new Pose(123, 124, Math.toRadians(125));
			if (alliance instanceof BlueAlliance) {
				startPose = startPose.mirror();
			}
			return startPose;
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
				new PrepareIntakeBottomAction(),
				new PrepareIntakeMiddleAction(),
				new PrepareIntakeTopAction(),
				// new PrepareIntakeLoadingZoneAction(),
				new LeaveLowAction(),
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

		public Callable<Void> getCallable() {
			return null;
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

		public Callable<Void> getCallable() {
			return null;
		}	

		public AutoState getResultingState() {
			return null;
		}	
	
		public PathChain getPathChain(Follower follower, Pose startingPose) {
			if (alliance instanceof RedAlliance) {
				return AutoPaths.Red.getExitLowShootingZone(follower);
			} else if (alliance instanceof BlueAlliance) {
				return AutoPaths.Blue.getExitLowShootingZone(follower);
			}
			return null;
		}
	}	

	public class LeaveUpAction extends AutoAction {
		public String getNameString() {
			return "Leave Upper Zone And End";
		}	

		public Callable<Void> getCallable() {
			return null;
		}	

		public AutoState getResultingState() {
			return null;
		}	
	
		public PathChain getPathChain(Follower follower, Pose startingPose) {
			if (alliance instanceof RedAlliance) {
				return AutoPaths.Red.getExitUpShootingZone(follower);
			} else if (alliance instanceof BlueAlliance) {
				return AutoPaths.Blue.getExitUpShootingZone(follower);
			}
			return null;
		}
	}	

	public class PrepareIntakeBottomAction extends AutoAction {
		public String getNameString() {
			return "Prepare Intake Bottom";
		}

		public Callable<Void> getCallable() {
			return null;
		}

		public AutoState getResultingState() {
			AutoState resultingState = new PrepareIntakeState();
			resultingState.setParents(getParents());
			return resultingState;
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

		public Callable<Void> getCallable() {
			return null;
		}

		public AutoState getResultingState() {
			AutoState resultingState = new PrepareIntakeState();
			resultingState.setParents(getParents());
			return resultingState;
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

	public class PrepareIntakeTopAction extends AutoAction {
		public String getNameString() {
			return "Prepare Intake Top";
		}	

		public Callable<Void> getCallable() {
			return null;
		}	

		public AutoState getResultingState() {
			AutoState resultingState = new PrepareIntakeState();
			resultingState.setParents(getParents());
			return resultingState;
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
	// 	public String getNameString() {
	// 		return "Prepare Intake Loading Zone";
	// 	}	

	// 	public Callable<Void> getCallable() {
	// 		return null;
	// 	}	

	// 	public AutoState getResultingState() {
	//		AutoState resultingState = new PrepareIntakeState();
	//		resultingState.setParents(getParents());
	//		return resultingState;
	// 	}	
	
	// 	public PathChain getPathChain(Follower follower, Pose startingPose) {
	// 		if (alliance instanceof RedAlliance) {
	// 			return AutoPaths.Red.PATHCHAIN;
	// 		} else if (alliance instanceof BlueAlliance) {
	// 			return AutoPaths.Blue.PATHCHAIN;
	// 		}
	// 		return null;
	// 	}
	// }	

	public class IntakePreparedAction extends AutoAction {
		public String getNameString() {
			return "Intake Prepared Set";
		}

		public Callable<Void> getCallable() {
			return null;
		}

		public AutoState getResultingState() {
			AutoState resultingState = new IntakeState();
			resultingState.setParents(getParents());
			return resultingState;
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

		public Callable<Void> getCallable() {
			return null;
		}

		public AutoState getResultingState() {
			AutoState resultingState = new ShootState();
			resultingState.setParents(getParents());
			return resultingState;
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

		public Callable<Void> getCallable() {
			return null;
		}

		public AutoState getResultingState() {
			AutoState resultingState = new ShootState();
			resultingState.setParents(getParents());
			return resultingState;
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

		public Callable<Void> getCallable() {
			return null;
		}

		public AutoState getResultingState() {
			AutoState resultingState = new OpenGateState();
			resultingState.setParents(getParents());
			return resultingState;
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

		public Callable<Void> getCallable() {
			return null;
		}

		public AutoState getResultingState() {
			AutoState resultingState = new ShootFinishState();
			resultingState.setParents(getParents());
			return resultingState;
		}
	
		public PathChain getPathChain(Follower follower, Pose startingPose) {
			return null;
		}
	}

	public class ShootUnsortedAction extends AutoAction {
		public String getNameString() {
			return "Shoot Unsorted";
		}

		public Callable<Void> getCallable() {
			return null;
		}

		public AutoState getResultingState() {
			AutoState resultingState = new ShootFinishState();
			resultingState.setParents(getParents());
			return resultingState;
		}
	
		public PathChain getPathChain(Follower follower, Pose startingPose) {
			return null;
		}
	}
}