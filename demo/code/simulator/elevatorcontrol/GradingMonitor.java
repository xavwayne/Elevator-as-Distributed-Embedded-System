/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.*;
import simulator.framework.Direction;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.Harness;
import simulator.framework.RuntimeMonitor;
import simulator.framework.Side;
import simulator.payloads.AtFloorPayload.ReadableAtFloorPayload;
import simulator.payloads.CarLanternPayload.ReadableCarLanternPayload;
import simulator.payloads.CarLightPayload.ReadableCarLightPayload;
import simulator.payloads.DoorClosedPayload.ReadableDoorClosedPayload;
import simulator.payloads.DoorOpenPayload.ReadableDoorOpenPayload;
import simulator.payloads.DriveSpeedPayload.ReadableDriveSpeedPayload;
import simulator.payloads.HallLightPayload.ReadableHallLightPayload;
import simulator.payloads.CarWeightPayload.ReadableCarWeightPayload;

/**
 * This module checks the high level requirements described in Project 8.
 *  
 * @author justinr2
 * @author danielli
 *
 * Change Log:
 * 4/25/2011 - edited to work with new simulator
 * 4/26/2011 - added monitor summary.
 * 4/28/2011 - added warning for violating 8.3 but car is empty
 */
public class GradingMonitor extends RuntimeMonitor {

	private final static int NONE = -1;

	//count variables for keeping track of violations of high level requirements
	private int rt6v = 0;
	private int rt7v = -1; //off by one error
	private int rt81v = 0;
	private int rt82v = 0;
	private int rt83v = 0;
	private int rt9v = 0;



	//parameters
	protected SimTime ignoreRecentCallInterval = new SimTime(400, SimTime.SimTimeUnit.MILLISECOND);
	//protected SimTime ignoreDoubleOpeningTimeout = new SimTime(5, SimTime.SimTimeUnit.SECOND);
	protected SimTime mDesiredFloorCheckInterval = new SimTime(100, SimTime.SimTimeUnit.MILLISECOND);
	protected SimTime mDesiredFloorInitialCheckInterval = new SimTime(500, SimTime.SimTimeUnit.MILLISECOND);

	//state variables
	protected int currentFloor = GradingMonitor.NONE;
	protected int lastStoppedFloor = GradingMonitor.NONE;
	protected int carWeight = 0; //used for tracking if the car is empty.
	protected boolean fastSpeedReached = false;
	protected boolean anyDoorOpen = false;
	protected boolean carLanternWasLit = false;
	protected boolean hasPendingOtherFloorCalls = false;
	protected String pendingCallString;
	protected boolean anyHallDoorOpen[] = new boolean[2]; //one for each hallway
	protected boolean anyHallDoorCompletelyOpen[] = new boolean[2]; //one for each hallway
	protected Direction lastCarLanternDirection = Direction.STOP;
	//protected boolean wasStopped = true;
	protected boolean hasMoved = false;  //blocks warnings until the first time the car moves
	protected DriveState driveState = DriveState.STOPPED;
	//use the system timer instead
	//protected Timer mDesiredFloorCheckTimer = new Timer(this);

	protected CallHistory callHistory = new CallHistory();

	protected boolean driveSpeedOrder[];
	protected int driveSpeedOrderVal[];
	protected final int STOPPED_SPEED = 0;
	protected final int LEVEL_SPEED = 1;
	protected final int SLOW_SPEED = 2;
	protected final int FAST_SPEED = 3;
	protected int orderOfSpeed = 0;
  public int flag_violate = 1;
	/**
	 * Todo
	 * @return
	 */
	@Override
	protected String[] summarize() {
		String[] summary = new String[6];
		summary[0] = rt6v + " RT-6 violations";
		summary[1] = rt7v + " RT-7 violations";
		summary[2] = rt81v + " RT-8.1 violations";
		summary[3] = rt82v + " RT-8.2 violations";
		summary[4] = rt83v + " RT-8.3 violations";
		summary[5] = rt9v + " RT-9 violations";
		return summary;
	}

	/**
	 * Keeps timestamps for when calls occur so that we can ignore very recent calls
	 */
	protected class CallHistory {

		final protected SimTime[][] carCallTimes = new SimTime[Elevator.numFloors][2];
		final protected SimTime[][][] hallCallTimes = new SimTime[Elevator.numFloors][2][2];
		final protected boolean[][] carCallState = new boolean[Elevator.numFloors][2];
		final protected boolean[][][] hallCallState = new boolean[Elevator.numFloors][2][2];

		public CallHistory() {
			for (int i = 0; i < Elevator.numFloors; i++) {
				for (Hallway h : Hallway.replicationValues) {
					carCallState[i][h.ordinal()] = false;
					for (Direction d : Direction.replicationValues) {
						hallCallState[i][h.ordinal()][d.ordinal()] = false;
					}
				}
			}
		}

		public void receive(ReadableCarLightPayload msg) {
			int cfloor = msg.getFloor();
			Hallway hallway = msg.getHallway();
			if (!carCallState[cfloor - 1][hallway.ordinal()] && msg.isLighted()) {
				//was false and is now true, so a call is registered
				carCallTimes[cfloor - 1][hallway.ordinal()] = Harness.getTime();
			}
			carCallState[cfloor - 1][hallway.ordinal()] = msg.isLighted();
		}

		public void receive(ReadableHallLightPayload msg) {
			int cfloor = msg.getFloor();
			Hallway hallway = msg.getHallway();
			Direction direction = msg.getDirection();
			if (!hallCallState[cfloor - 1][hallway.ordinal()][direction.ordinal()] && msg.lighted()) {
				//was false and is now true, so a call is registered
				hallCallTimes[cfloor - 1][hallway.ordinal()][direction.ordinal()] = Harness.getTime();
			}
			hallCallState[cfloor - 1][hallway.ordinal()][direction.ordinal()] = msg.lighted();
		}

		public boolean checkForCalls(int ignoreFloor, SimTime onlyOlderThan) {
			for (int floor = 0; floor < Elevator.numFloors; ++floor) {
				for (Hallway h : Hallway.replicationValues) {
					if (floor != ignoreFloor - 1) {
						if (carCallState[floor][h.ordinal()] &&
								checkOlder(carCallTimes[floor][h.ordinal()], onlyOlderThan)) {
							return true;
						}
						for (Direction d : Direction.replicationValues) {
							if (hallCallState[floor][h.ordinal()][d.ordinal()] &&
									checkOlder(hallCallTimes[floor][h.ordinal()][d.ordinal()], onlyOlderThan)) {
								return true;
							}
						}
					}
				}
			}
			return false;
		}

		public String getCallString(int ignoreFloor, SimTime onlyOlderThan) {
			StringBuffer sb = new StringBuffer();
			for (int floor = 0; floor < Elevator.numFloors; ++floor) {
				int cor_floor = floor+1;
				for (Hallway h : Hallway.replicationValues) {
					if (floor != ignoreFloor - 1) {
						if (carCallState[floor][h.ordinal()] &&
								checkOlder(carCallTimes[floor][h.ordinal()], onlyOlderThan)) {
							sb.append("CC-" + cor_floor + "-" + h + ",");
						}
						for (Direction d : Direction.replicationValues) {
							if (hallCallState[floor][h.ordinal()][d.ordinal()] &&
									checkOlder(hallCallTimes[floor][h.ordinal()][d.ordinal()], onlyOlderThan)) {
								sb.append("HC-" + cor_floor + "-" + h + "-" + d + ",");
							}
						}
					}
				}
			}
			return sb.toString();
		}

		public boolean hasACall(int floor, SimTime onlyOlderThan) {
			return hasACall(floor, Hallway.FRONT, onlyOlderThan) || hasACall(floor, Hallway.BACK, onlyOlderThan);
		}

		public boolean hasACall(int floor, Hallway h, SimTime onlyOlderThan) {
			if (floor == GradingMonitor.NONE) {
				return false;
			}
			if ((carCallState[floor - 1][h.ordinal()] && checkOlder(carCallTimes[floor - 1][h.ordinal()], onlyOlderThan)) ||
					(hallLights[floor - 1][h.ordinal()][Direction.UP.ordinal()].lighted() && checkOlder(hallCallTimes[floor - 1][h.ordinal()][Direction.UP.ordinal()], onlyOlderThan)) ||
					(hallLights[floor - 1][h.ordinal()][Direction.DOWN.ordinal()].lighted() && checkOlder(hallCallTimes[floor - 1][h.ordinal()][Direction.DOWN.ordinal()], onlyOlderThan))) {
				return true;
			}
			return false;
		}

		private boolean checkOlder(SimTime targetTime, SimTime onlyOlderThan) {
			if (targetTime == null) {
				return false;
			}
			return SimTime.subtract(Harness.getTime(), targetTime).isGreaterThan(onlyOlderThan);
		}
	}

	protected enum DriveState {

		MOVING,
		STOPPED
	}

	protected enum TimerCallbacks {

		CALLBACK_CHECK_DESIRED_FLOOR,
		CALLBACK_RECHECK_DESIRED_FLOOR
	}

	public GradingMonitor() {
		anyHallDoorOpen[0] = anyHallDoorOpen[1] = false;
		anyHallDoorCompletelyOpen[0] = anyHallDoorCompletelyOpen[1] = false;
		/* Will be called only once */
		if(hasMoved == false) {
			driveSpeedOrder = new boolean[4];
			driveSpeedOrderVal = new int [4];
			for(int i = 0; i < 4; i++){
				driveSpeedOrder[i] = false;
				driveSpeedOrderVal[i] = 0;
			}
		}
	}


	public void timerExpired(Object callbackData) {
	}

	@Override
	public void receive(ReadableDoorClosedPayload msg) {
		Hallway hall = msg.getHallway();
		boolean oldDoorOpen = anyHallDoorOpen[hall.ordinal()];
		//compute the new any door open value for this hallway
		anyHallDoorOpen[hall.ordinal()] = isAnyDoorOpen(hall);
		if (!oldDoorOpen && anyHallDoorOpen[hall.ordinal()]) {
			doorStartedOpening(hall);
		}
		if (oldDoorOpen && !anyHallDoorOpen[hall.ordinal()]) {
			doorClosed(hall);
		}
	}

	@Override
	public void receive(ReadableDoorOpenPayload msg) {
		Hallway hall = msg.getHallway();
		boolean oldDoorCompletelyOpen = anyHallDoorCompletelyOpen[hall.ordinal()];
		anyHallDoorCompletelyOpen[hall.ordinal()] = isDoorCompletelyOpen(hall);
		if (anyHallDoorCompletelyOpen[hall.ordinal()] && !oldDoorCompletelyOpen) {
			doorCompletedOpening(hall);
		}
	}

	@Override
	public void receive(ReadableAtFloorPayload msg) {
		updateCurrentFloor(msg);
	}

	/**
	 * called whenever the drive is going slow
	 */
	private void driveLevel() {
		driveSpeedOrder[LEVEL_SPEED] = true;
		driveSpeedOrderVal[LEVEL_SPEED] = ++orderOfSpeed;
	}
	/**
	 * called whenever the drive is going slow
	 */
	private void driveSlow() {
		driveSpeedOrder[SLOW_SPEED] = true;
		driveSpeedOrderVal[SLOW_SPEED] = ++orderOfSpeed;
	}
    
    private void driveStop() {
        driveSpeedOrder[STOPPED_SPEED] = true;
        driveSpeedOrderVal[STOPPED_SPEED] = ++orderOfSpeed;
    }

	/**
	 * called whenever the drive is going faster than the slow speed
	 */
	private void driveFast() {
		//remember that the car went fast at some point
		fastSpeedReached = true;
		driveSpeedOrder[FAST_SPEED] = true;
		driveSpeedOrder[SLOW_SPEED] = false;
		driveSpeedOrder[LEVEL_SPEED] = false;
		driveSpeedOrder[STOPPED_SPEED] = false;
		driveSpeedOrderVal[FAST_SPEED] = 1;
		driveSpeedOrderVal[STOPPED_SPEED] = 0;
		driveSpeedOrderVal[SLOW_SPEED] = 0;
		driveSpeedOrderVal[LEVEL_SPEED] = 0;
		orderOfSpeed = 0;
	}

	public void checkIfViolation(){
        
        int flag = 0;
        if((driveSpeedOrder[SLOW_SPEED] == true) && (driveSpeedOrderVal[SLOW_SPEED] == 1)){
            
            if((driveSpeedOrder[LEVEL_SPEED] == true) && (driveSpeedOrderVal[LEVEL_SPEED] == 2)){
                
                if((driveSpeedOrder[STOPPED_SPEED] == true && driveSpeedOrderVal[STOPPED_SPEED] == 3)){
                    flag = 1;
                }
            }
            else if((driveSpeedOrder[STOPPED_SPEED] == true && driveSpeedOrderVal[STOPPED_SPEED] == 2)){
                flag = 1;
            }
        }
        if (flag_violate == 0){
        }
	}
	@Override
	public void receive(ReadableDriveSpeedPayload msg) {
		if (msg.speed() > 0) {
			hasMoved = true;
		}
		/* Will happen when transition from stop -> moving */
		if ((msg.speed() > DriveObject.LevelingSpeed && driveState == DriveState.STOPPED)){
			driveState = DriveState.MOVING;
			driveStarted(msg.direction());
			/* Initially was stop so make it true */
			driveSpeedOrder[STOPPED_SPEED] = false;
			driveSpeedOrder[SLOW_SPEED] = false;
			driveSpeedOrder[LEVEL_SPEED] = false;
			driveSpeedOrder[FAST_SPEED] = false;
			/* Initially was stop so make it true */
			driveSpeedOrderVal[STOPPED_SPEED] = 0;
			driveSpeedOrderVal[SLOW_SPEED] = 0;
			driveSpeedOrderVal[LEVEL_SPEED] = 0;
			driveSpeedOrderVal[FAST_SPEED] = 0;
			orderOfSpeed = 0;
		}
		/* Fast Speed Detection */
		if (msg.speed() > DriveObject.SlowSpeed) {
			driveState = DriveState.MOVING;
			driveFast();
		}
		/* Slow Speed Detection */
		else if ((msg.speed() > 0) && (msg.speed() > DriveObject.LevelingSpeed && msg.speed() <= DriveObject.SlowSpeed)) {
			driveState = DriveState.MOVING;
			driveSlow();
		}
		/* Level Speed Detection */
		else if((msg.speed() > 0) && (msg.speed() <= DriveObject.LevelingSpeed)){
			driveState = DriveState.MOVING;
			driveLevel();
		}
		/* Stop Speed Detection */
		else if ((msg.speed() == 0) && (msg.direction() == Direction.STOP) && (driveState == DriveState.MOVING) && (currentFloor != GradingMonitor.NONE)) {
			driveState = DriveState.STOPPED;
            driveStop();
			if (!this.isAnyDoorOpen())
				driveStoppedAtFloor();
			checkIfViolation();
            /* Initially was stop so make it true */
			driveSpeedOrder[STOPPED_SPEED] = false;
			driveSpeedOrder[SLOW_SPEED] = false;
			driveSpeedOrder[LEVEL_SPEED] = false;
			driveSpeedOrder[FAST_SPEED] = false;
			/* Initially was stop so make it true */
			driveSpeedOrderVal[STOPPED_SPEED] = 0;
			driveSpeedOrderVal[SLOW_SPEED] = 0;
			driveSpeedOrderVal[LEVEL_SPEED] = 0;
			driveSpeedOrderVal[FAST_SPEED] = 0;
			orderOfSpeed = 0;
		}
	}

	@Override
	public void receive(ReadableCarLanternPayload msg) {
		updateCarLantern(msg);
	}

	@Override
	public void receive(ReadableCarLightPayload msg) {
		callHistory.receive(msg);
	}

	@Override
	public void receive(ReadableHallLightPayload msg) {
		callHistory.receive(msg);
	}

	@Override
	public void receive(ReadableCarWeightPayload msg) {
		if (msg.weight() != carWeight)
			updateWeight(msg.weight());
	}

	/*--------------------------------------------------------------------------
	 * Event functions - these are triggered by the received methods above
	 *------------------------------------------------------------------------*/
	/**
	 * Called when the car comes to a stop
	 */
	private void driveStoppedAtFloor() {
		//warn if no pending calls at this currentFloor
		//skip the warning if there are no calls at all in the system, since the car has to stop somewhere
		//if (!callHistory.hasACall(currentFloor, ignoreRecentCallInterval) && callHistory.checkForCalls(Control.NONE, ignoreRecentCallInterval)) {
		if (!callHistory.hasACall(currentFloor, SimTime.ZERO) && callHistory.checkForCalls(GradingMonitor.NONE, ignoreRecentCallInterval)) {
			if (mDesiredFloor.getFloor() != currentFloor) {
				rt6v++;
				blockableWarning("R-T6 violated:    Elevator stopped at floor " + currentFloor + " when there are no pending calls at that floor.  Current calls:  " + callHistory.getCallString(GradingMonitor.NONE, ignoreRecentCallInterval));
			}
		}
		//check to see if fast was reached during the previous run
		if (lastStoppedFloor != currentFloor) {
			//we've stopped at a new currentFloor
			if (fastSpeedAttainable(lastStoppedFloor, currentFloor)) {
				//check and see if the drive was ever reached fast
				if (!fastSpeedReached) {
					rt9v++;
					blockableWarning("R-T9 violated:    The drive was not commanded to FAST on the trip between " + lastStoppedFloor + " and " + currentFloor);
				}
			}
			//now that the check is done, set the lastStoppedFloor to this currentFloor
			lastStoppedFloor = currentFloor;
			//reset fastSpeedReached
			fastSpeedReached = false;
		}
	}

	/**
	 * Called when the car starts moving
	 * @param d - direction of movement
	 */
	private void driveStarted(Direction direction) {
		//check to see if we're moving in the direction of the last lantern
		if (lastCarLanternDirection != Direction.STOP && direction != lastCarLanternDirection) {
			if (carWeight == 0) {
				rt83v++;
				blockableWarning("R-T8.3 violated (Car Empty):  The car lanterns indicated desired direction as " + lastCarLanternDirection + " but the car is moving " + direction);
			}
			else {
				if (lastCarLanternDirection == Direction.UP) {
					//Check for delayed car call presses
					for(int floor = currentFloor+1; floor < Elevator.numFloors; floor++) {
						if (callHistory.hasACall(floor, ignoreRecentCallInterval)) {
							rt83v++;
							blockableWarning("R-T8.3 violated:  The car lanterns indicated desired direction as " + lastCarLanternDirection + " but the car is moving " + direction);
							break;
						}
					}
				}
				else {
					//Check for delayed car call presses
					for(int floor = currentFloor-1; floor > 0; floor--) {
						if (callHistory.hasACall(floor, ignoreRecentCallInterval)) {
							rt83v++;
							blockableWarning("R-T8.3 violated:  The car lanterns indicated desired direction as " + lastCarLanternDirection + " but the car is moving " + direction);
							break;
						}
					}
				}
			}
		}
	}


	/**
	 * Called when the door on the hallway side is completely open.
	 * @param hallway
	 */
	private void doorCompletedOpening(Hallway hallway) {

	}

	/**
	 * called when the door opens.  
	 * @param hallway - the hallway of the door that opened
	 */
	private void doorStartedOpening(Hallway hallway) {
		//check to see if there are pending calls
		//if (!callHistory.hasACall(currentFloor, hallway, ignoreRecentCallInterval) && carWeightPayload.weight < Elevator.MaxCarCapacity) {
		if (!callHistory.hasACall(currentFloor, hallway, SimTime.ZERO) && carWeightPayload.weight() < Elevator.MaxCarCapacity) {
			//door opened without any calls on this side
			rt7v++;
			blockableWarning("R-T7 violated:    The door opened at " + currentFloor + "," + hallway + " without any pending calls. Current calls: " + callHistory.getCallString(GradingMonitor.NONE, ignoreRecentCallInterval));
		}
		//set state to be used to check the car lanterns when the doors close
		carLanternWasLit = false;
		lastCarLanternDirection = Direction.STOP;
		hasPendingOtherFloorCalls = callHistory.checkForCalls(currentFloor, ignoreRecentCallInterval);
		pendingCallString = callHistory.getCallString(currentFloor, ignoreRecentCallInterval);
	}

	/**
	 * called when the door closes.  
	 * @param hallway - the hallway of the door that closed
	 */
	private void doorClosed(Hallway hallway) {
		//check the car lanterns
		if (hasPendingOtherFloorCalls && !carLanternWasLit && !isAnyDoorOpen()) {
			rt81v++;
			blockableWarning("R-T8.1 violated:  CarLantern was not lit at floor " + currentFloor + " while there were pending calls elsewhere.  Current calls:  " + pendingCallString);
		}
	}
	/**
	 * called when car weight changes
	 * @param weight - the new weight of the car
	 */
	private void updateWeight(int weight) {
		carWeight = weight;
	}

	/*--------------------------------------------------------------------------
	 * Utility and helper functions
	 *------------------------------------------------------------------------*/
	private boolean fastSpeedAttainable(int startFloor, int endFloor) {
		//for now, fast speed is attainable between all floors
		if (startFloor == GradingMonitor.NONE || endFloor == GradingMonitor.NONE) {
			return false;
		}
		if (startFloor != endFloor) {
			return true;
		}
		return false;
	}

	/*private boolean checkForCalls(int ignoreFloor) {
    for (int floor = 0; floor < Elevator.numFloors; ++floor) {
    for (Hallway h : Hallway.replicationValues) {
    if (floor != ignoreFloor - 1) {
    if (carLights[floor][h.ordinal()].lighted) {
    return true;
    }
    for (Direction d : Direction.replicationValues) {
    if (hallLights[floor][h.ordinal()][d.ordinal()].lighted) {
    return true;
    }
    }
    }
    }
    }
    return false;
    }
    private String getCallString(int ignoreFloor) {
    StringBuffer sb = new StringBuffer();
    for (int floor = 0; floor < Elevator.numFloors; ++floor) {
    for (Hallway h : Hallway.replicationValues) {
    if (floor != ignoreFloor - 1) {
    if (carLights[floor][h.ordinal()].lighted) {
    sb.append("CC-" + floor + "-" + h + ",");
    }
    for (Direction d : Direction.replicationValues) {
    if (hallLights[floor][h.ordinal()][d.ordinal()].lighted) {
    sb.append("HC-" + floor + "-" + h + "-" + d + ",");
    }
    }
    }
    }
    }
    return sb.toString();
    }
    private boolean hasACall(int floor) {
    return hasACall(floor, Hallway.FRONT) || hasACall(floor, Hallway.BACK);
    }
    private boolean hasACall(int floor, Hallway h) {
    if (floor == Control.NONE) {
    return false;
    }
    if (carLights[floor - 1][h.ordinal()].lighted ||
    hallLights[floor - 1][h.ordinal()][Direction.UP.ordinal()].lighted ||
    hallLights[floor - 1][h.ordinal()][Direction.DOWN.ordinal()].lighted) {
    return true;
    }
    return false;
    }*/

	private void blockableWarning(String warning) {
		if (hasMoved) {
			warning(warning);
		}
	}

	private boolean isAnyDoorOpen() {
		return isAnyDoorOpen(Hallway.FRONT) || isAnyDoorOpen(Hallway.BACK);
	}

	private boolean isAnyDoorOpen(Hallway h) {
		return !(doorCloseds[h.ordinal()][Side.LEFT.ordinal()].isClosed() ||
				doorCloseds[h.ordinal()][Side.RIGHT.ordinal()].isClosed());
	}

	private boolean isDoorCompletelyOpen(Hallway h) {
		return doorOpeneds[h.ordinal()][Side.LEFT.ordinal()].isOpen() &&
				doorOpeneds[h.ordinal()][Side.RIGHT.ordinal()].isOpen();
	}

	private void updateCurrentFloor(ReadableAtFloorPayload lastAtFloor) {
		if (lastAtFloor.getFloor() == currentFloor) {
			//the atFloor message is for the currentfloor, so check both sides to see if they a
			if (!atFloors[lastAtFloor.getFloor() - 1][Hallway.BACK.ordinal()].value() && !atFloors[lastAtFloor.getFloor() - 1][Hallway.FRONT.ordinal()].value()) {
				//both sides are false, so set to NONE
				currentFloor = GradingMonitor.NONE;
			}
			//otherwise at least one side is true, so leave the current currentFloor as is
		} else {
			if (lastAtFloor.value()) {
				currentFloor = lastAtFloor.getFloor();
			}
		}
	}

	private void updateCarLantern(ReadableCarLanternPayload p) {
		if (isAnyDoorOpen()) {
			//while doors are open, track the last lantern direction
			if (p.lighted()) {
				carLanternWasLit = true;
				if (lastCarLanternDirection == Direction.STOP) {
					//if last direction was stop, then set it to the newly observed value
					lastCarLanternDirection = p.getDirection();
				} else if (lastCarLanternDirection != p.getDirection()) {
					//if last direction is opposite, both lanterns have been turned on during one cycle:
					rt82v++;
					blockableWarning("R-T8.2 violated:  Car lanterns in both directions were lit during one door cycle.");
					//change the last direction even though there was a warning
					//so that the movement direction check will check against
					//the last direction displayed.
					lastCarLanternDirection = p.getDirection();
				} else {
					//last direction in the same direction as the light, so do nothing.
				}
			}
		} else {
			//while doors are closed, don't check the lantern direction
			if (mDesiredFloor.getDirection()==Direction.STOP && driveState == DriveState.STOPPED && !p.lighted()) {
				lastCarLanternDirection = Direction.STOP;
			}
		}
	}
}

