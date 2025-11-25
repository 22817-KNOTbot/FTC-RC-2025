package org.firstinspires.ftc.teamcode.auto;

import java.util.concurrent.Callable;

public class AutoConfiguration {
	public static abstract class AutoState {
		private AutoState[] parents;
		public abstract String getNameString();
		public abstract AutoAction[] getAutoActions();

		public AutoState[] getParents() {
			return parents;
		}

		public AutoState() {
			this(new AutoState[] {});
		}

		public AutoState(AutoState[] parents) {
			this.parents = parents;
		}
	}

	public static interface AutoAction {
		public String getNameString();
		public Callable<Void> getCallable();
		public AutoState getResultingState();
	}

	/*
	 * States
	 */

	public static class StartState extends AutoState {
		public String getNameString() {
			return "Start";
		}

		public AutoAction[] getAutoActions() {
			return new AutoAction[] {
				new PrepareIntakeBottomAction(),
				new PrepareIntakeMiddleAction(),
				new PrepareIntakeTopAction(),
				new PrepareIntakeLoadingZoneAction(),
				new LeaveAction(),
				new EndAction()
			};
		}
	}

	public static class PrepareIntakeState extends AutoState {
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

	public static class IntakeState extends AutoState {
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

	public static class OpenGateState extends AutoState {
		public String getNameString() {
			return "Open Gate";
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

	public static class ShootState extends AutoState {
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

	public static class ShootFinishState extends AutoState {
		public String getNameString() {
			return "Shoot Finish";
		}

		public AutoAction[] getAutoActions() {
			return new AutoAction[] {
				new PrepareIntakeBottomAction(),
				new PrepareIntakeMiddleAction(),
				new PrepareIntakeTopAction(),
				new PrepareIntakeLoadingZoneAction(),
				new LeaveAction(),
				new EndAction()
			};
		}
	}

	/*
	 * Actions
	 */

	public static class EndAction implements AutoAction {
		public String getNameString() {
			return "End";
		}

		public Callable<Void> getCallable() {
			return null;
		}

		public AutoState getResultingState() {
			return null;
		}
	}

	public static class LeaveAction implements AutoAction {
		public String getNameString() {
			return "Leave and end";
		}	

		public Callable<Void> getCallable() {
			return null;
		}	

		public AutoState getResultingState() {
			return null;
		}	
	}	

	public static class PrepareIntakeBottomAction implements AutoAction {
		public String getNameString() {
			return "Prepare Intake Bottom";
		}

		public Callable<Void> getCallable() {
			return null;
		}

		public AutoState getResultingState() {
			return new PrepareIntakeState();
		}
	}
	
	public static class PrepareIntakeMiddleAction implements AutoAction {
		public String getNameString() {
			return "Prepare Intake Middle";
		}

		public Callable<Void> getCallable() {
			return null;
		}

		public AutoState getResultingState() {
			return new PrepareIntakeState();
		}
	}

	public static class PrepareIntakeTopAction implements AutoAction {
		public String getNameString() {
			return "Prepare Intake Top";
		}	

		public Callable<Void> getCallable() {
			return null;
		}	

		public AutoState getResultingState() {
			return new PrepareIntakeState();
		}	
	}	

	public static class PrepareIntakeLoadingZoneAction implements AutoAction {
		public String getNameString() {
			return "Prepare Intake Loading Zone";
		}	

		public Callable<Void> getCallable() {
			return null;
		}	

		public AutoState getResultingState() {
			return new PrepareIntakeState();
		}	
	}	

	public static class IntakePreparedAction implements AutoAction {
		public String getNameString() {
			return "Intake Prepared";
		}

		public Callable<Void> getCallable() {
			return null;
		}

		public AutoState getResultingState() {
			return new IntakeState();
		}
	}

	public static class ShootFarAction implements AutoAction {
		public String getNameString() {
			return "Shoot Far";
		}

		public Callable<Void> getCallable() {
			return null;
		}

		public AutoState getResultingState() {
			return new ShootState();
		}
	}

	public static class ShootCloseAction implements AutoAction {
		public String getNameString() {
			return "Shoot Close";
		}

		public Callable<Void> getCallable() {
			return null;
		}

		public AutoState getResultingState() {
			return new ShootState();
		}
	}

	public static class OpenGateAction implements AutoAction {
		public String getNameString() {
			return "Open Gate";
		}

		public Callable<Void> getCallable() {
			return null;
		}

		public AutoState getResultingState() {
			return new OpenGateState();
		}
	}

	public static class ShootSortedAction implements AutoAction {
		public String getNameString() {
			return "Shoot Sorted";
		}

		public Callable<Void> getCallable() {
			return null;
		}

		public AutoState getResultingState() {
			return new ShootFinishState();
		}
	}

	public static class ShootUnsortedAction implements AutoAction {
		public String getNameString() {
			return "Shoot Unsorted";
		}

		public Callable<Void> getCallable() {
			return null;
		}

		public AutoState getResultingState() {
			return new ShootFinishState();
		}
	}
}
