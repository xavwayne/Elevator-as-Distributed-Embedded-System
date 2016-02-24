/**
 * 18649-Fall-2015
 * Group 3
 * Jiyu Shi(jiyus) ; Shuai Wang(shuaiwa1); Xiaoyu Wang(xiaoyuw); Xiao Guo(xiaog)
 */
package simulator.elevatorcontrol;

import java.util.HashMap;

import jSimPack.SimTime;
import simulator.elevatorcontrol.Utility.AtFloorArray;
import simulator.elevatormodules.CarWeightCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.elevatormodules.DoorOpenedCanPayloadTranslator;
import simulator.elevatormodules.DoorReversalCanPayloadTranslator;
import simulator.framework.*;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.DoorMotorPayload;
import simulator.payloads.DoorMotorPayload.WriteableDoorMotorPayload;
import simulator.elevatorcontrol.NewIntegerCanPayloadTranslator;

/**
 * @author Xiao Guo
 * DoorControl Class is for the control of a single door.
 */
public class DoorControl extends Controller implements TimeSensitive {
	
	//store the period for the controller
    private SimTime period;
    static final int maxReversal = 5;


    static final SimTime DEFAULT_DOOR_DWELL = new SimTime(4000, SimTime.SimTimeUnit.MILLISECOND);
    
    //these variables keep track of which instance this is.
    private final Hallway hallway;
    //private final Side side;
    
    //inputs
    //mDesiredFloor
    ReadableCanMailbox networkDesiredFloor;
    DesiredFloorCanPayloadTranslator networkDesiredFloorTranslator;
    //mDesiredDwell
    ReadableCanMailbox networkDesiredDwell;
    NewIntegerCanPayloadTranslator networkDesiredDwellTranslator;
    //mDoorClosed[b, r]
    ReadableCanMailbox networkDoorClosed;
    DoorClosedCanPayloadTranslator networkDoorClosedTranslator;
    //mDoorOpened[b, r]
    ReadableCanMailbox networkDoorOpened;
    DoorOpenedCanPayloadTranslator networkDoorOpenedTranslator;
    //mDoorReversal
    ReadableCanMailbox networkDoorReversal_1;
    DoorReversalCanPayloadTranslator networkDoorReversalCanPayloadTranslator_1;
    ReadableCanMailbox networkDoorReversal_2;
    DoorReversalCanPayloadTranslator networkDoorReversalCanPayloadTranslator_2;
    //mDriveSpeed
    ReadableCanMailbox networkDriveSpeed;
    NewIntegerCanPayloadTranslator networkDriveSpeedTranslator;
    //mCarWeight
    ReadableCanMailbox networkCarWeight;
    CarWeightCanPayloadTranslator networkCarWeightTranslator;
    //mCarCall
    HashMap<Integer, NewBooleanCanPayloadTranslator> mCarCallArray;
    //mHallCall
    HashMap<Integer, NewBooleanCanPayloadTranslator> mHallCallArray;
    //mAtFloor[f, b]
    AtFloorArray atFloorArray;
    
    //outputs
    //DoorMotor[b, r]
    WriteableDoorMotorPayload localDoorMotor;
    WriteableCanMailbox networkDoorMotor;
    DoorMotorCommandCanPayloadTranslator networkDoorMotorTranslator;
    
    //these variables keep track of the elevator's position.
    private int currentFloor;
    private int driveSpeed;
    
    //the timers to remain door open.
    private SimTime currentDwell;
    private SimTime openTime;
    
    //the indicator for reversal
    private int reversalCount;
    
    //enumerate states
    private enum State {
    	STATE_STOP_CLOSED,
        STATE_STOP_OPEN,
        STATE_NUDGE,
        STATE_OPEN,
        STATE_RE_OPEN,
        STATE_CLOSE
    }
    //state variable initialized to the initial state STATE_STOP_CLOSED
    private State state = State.STATE_STOP_CLOSED;
    
	/**
	 * @param hallway
	 * @param side
	 * @param simTime
	 * @param verbose
	 */
	public DoorControl(Hallway hallway, Side side, SimTime simTime, boolean verbose) {
		super("DoorControl" + ReplicationComputer.makeReplicationString(hallway, side), verbose);
		this.period = simTime;
		this.hallway = hallway;
		//this.side = side;
		this.currentDwell = DEFAULT_DOOR_DWELL;
		log("Created DoorControl with period = ", period, ", hallway=",
				hallway.toString(), ", side=", side.toString());
		
		//outputs
		//Physical DoorMotor
		localDoorMotor = DoorMotorPayload.getWriteablePayload(hallway, side);
        physicalInterface.sendTimeTriggered(localDoorMotor, period);
        //Network DoorMotor
        networkDoorMotor = CanMailbox.getWriteableCanMailbox(MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID +
        		ReplicationComputer.computeReplicationId(hallway, side));
        networkDoorMotorTranslator = new DoorMotorCommandCanPayloadTranslator(networkDoorMotor);
        networkDoorMotorTranslator.set(DoorCommand.STOP.ordinal());
        canInterface.sendTimeTriggered(this.networkDoorMotor, simTime);
        
        //inputs
        //mDesiredFloor
        networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
        networkDesiredFloorTranslator = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
        canInterface.registerTimeTriggered(networkDesiredFloor);
        //mDesiredDwell
        networkDesiredDwell = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_DWELL_BASE_CAN_ID  +
        		ReplicationComputer.computeReplicationId(hallway));
        networkDesiredDwellTranslator = new NewIntegerCanPayloadTranslator(networkDesiredDwell);
        canInterface.registerTimeTriggered(networkDesiredDwell);
        //mDoorClosed
        networkDoorClosed = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID  +
        		ReplicationComputer.computeReplicationId(hallway, side));
        networkDoorClosedTranslator = new DoorClosedCanPayloadTranslator(networkDoorClosed, hallway, side);
        canInterface.registerTimeTriggered(networkDoorClosed);
        //mDoorOpened
        networkDoorOpened = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_OPEN_SENSOR_BASE_CAN_ID  +
        		ReplicationComputer.computeReplicationId(hallway, side));
        networkDoorOpenedTranslator = new DoorOpenedCanPayloadTranslator(networkDoorOpened, hallway, side);
        canInterface.registerTimeTriggered(networkDoorOpened);
        //mDoorReversal
        networkDoorReversal_1 = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_REVERSAL_SENSOR_BASE_CAN_ID  +
        		ReplicationComputer.computeReplicationId(hallway, Side.LEFT));
        networkDoorReversalCanPayloadTranslator_1 = new DoorReversalCanPayloadTranslator(networkDoorReversal_1, hallway, Side.LEFT);
        canInterface.registerTimeTriggered(networkDoorReversal_1);
        networkDoorReversal_2 = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_REVERSAL_SENSOR_BASE_CAN_ID  +
        		ReplicationComputer.computeReplicationId(hallway, Side.RIGHT));
        networkDoorReversalCanPayloadTranslator_2 = new DoorReversalCanPayloadTranslator(networkDoorReversal_2, hallway, Side.RIGHT);
        canInterface.registerTimeTriggered(networkDoorReversal_2);
        //mDriveSpeed
        networkDriveSpeed = CanMailbox.getReadableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
        networkDriveSpeedTranslator = new NewIntegerCanPayloadTranslator(networkDriveSpeed);
        canInterface.registerTimeTriggered(networkDriveSpeed);
        //mCarWeight
        networkCarWeight = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_CAN_ID);
        networkCarWeightTranslator = new CarWeightCanPayloadTranslator(networkCarWeight);
        canInterface.registerTimeTriggered(networkCarWeight);
        //mCarCall
        mCarCallArray = new HashMap<Integer, NewBooleanCanPayloadTranslator>();
        for (int i = 1; i <= Elevator.numFloors; i++) {
        	for (Hallway myhall : Hallway.replicationValues) {
		        ReadableCanMailbox networkCarCall 
		        = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_CALL_BASE_CAN_ID 
		        		+ ReplicationComputer.computeReplicationId(i, myhall));
		        NewBooleanCanPayloadTranslator networkCarCallTranslator = new NewBooleanCanPayloadTranslator(networkCarCall);
		        canInterface.registerTimeTriggered(networkCarCall);
		        mCarCallArray.put(ReplicationComputer.computeReplicationId(i, myhall), networkCarCallTranslator);
        	}
        } 
        //mHallCall
        mHallCallArray = new HashMap<Integer, NewBooleanCanPayloadTranslator>();
        for (int i = 1; i <= Elevator.numFloors; i++) {
        	for (Hallway myhall : Hallway.replicationValues) {
        		for(Direction myDirection : Direction.replicationValues){
		        ReadableCanMailbox networkCarCall 
		        = CanMailbox.getReadableCanMailbox(MessageDictionary.HALL_CALL_BASE_CAN_ID 
		        		+ ReplicationComputer.computeReplicationId(i, myhall, myDirection));
		        NewBooleanCanPayloadTranslator networkCarCallTranslator 
		        = new NewBooleanCanPayloadTranslator(networkCarCall);
		        canInterface.registerTimeTriggered(networkCarCall);
		        mHallCallArray.put(ReplicationComputer.computeReplicationId(i, myhall, myDirection), networkCarCallTranslator);
        		}
        	}
        }
        //mAtFloor
        atFloorArray = new Utility.AtFloorArray(canInterface);
        
        //reversal count
        reversalCount = 0;
        
        //set the timer
        this.timer.start(period);
	}

	/* 
	 * timerExpired() is executed when the timer is alarmed. This method will detect the
	 * state change.
	 * 
	 * @see simulator.framework.TimeSensitive#timerExpired(java.lang.Object)
	 */
	@Override
	public void timerExpired(Object callbackData) {
		//next state
		State newState = state;
		//read inputs in
		currentFloor = atFloorArray.getCurrentFloor();
		driveSpeed = networkDriveSpeedTranslator.getValue();
		//log the start statue
		log("Current Floor=", currentFloor);
		log("Moving Speed=", driveSpeed);

        Hallway desiredHallway = networkDesiredFloorTranslator.getHallway();
        Direction desiredDirection = networkDesiredFloorTranslator.getDirection();

        boolean callSign = false;
        if(desiredDirection!=Direction.STOP){
            if (currentFloor != -1) {
            if(desiredHallway == hallway || desiredHallway == Hallway.BOTH) {
                callSign = mHallCallArray.get(ReplicationComputer.computeReplicationId(currentFloor, hallway, desiredDirection)).getValue()
                        || mCarCallArray.get(ReplicationComputer.computeReplicationId(currentFloor, hallway)).getValue();
            }
        }

        }else{
            if (currentFloor != -1) {
            if(desiredHallway == hallway || desiredHallway == Hallway.BOTH) {
                callSign = mHallCallArray.get(ReplicationComputer.computeReplicationId(currentFloor, hallway, Direction.UP)).getValue()
                        || mHallCallArray.get(ReplicationComputer.computeReplicationId(currentFloor, hallway, Direction.DOWN)).getValue() || mCarCallArray.get(ReplicationComputer.computeReplicationId(currentFloor, hallway)).getValue();
            }

            }
        }

        
		
        switch (state) {
        	case STATE_STOP_CLOSED:
        		//reversalCount = 0;  
        		//#transition 'T5.4'
        		if (driveSpeed == 0.0 && (currentFloor == -1 ? false : atFloorArray.isAtFloor(currentFloor, hallway)) &&
        		currentFloor == networkDesiredFloorTranslator.getFloor() && callSign) {
        			newState = State.STATE_OPEN;
                } else {//stay
                	localDoorMotor.set(DoorCommand.STOP);
                	networkDoorMotorTranslator.set(DoorCommand.STOP.ordinal());
                	newState = State.STATE_STOP_CLOSED;
                }
        		break;
        	case STATE_STOP_OPEN:
        		//#transition 'T5.2'
        		if(SimTime.subtract(Harness.getTime(),openTime).isGreaterThan(currentDwell) && reversalCount < maxReversal) {
        			newState = State.STATE_CLOSE;
        		//#transition 'T5.8'
        		} else if (SimTime.subtract(Harness.getTime(),openTime).isGreaterThan(currentDwell) && reversalCount == maxReversal) {
        			newState = State.STATE_NUDGE;
        		} else {//stay
	        		localDoorMotor.set(DoorCommand.STOP);
	        		networkDoorMotorTranslator.set(DoorCommand.STOP.ordinal());
	        		newState = State.STATE_STOP_OPEN;
        		}
        		break;
        	case STATE_NUDGE:
        		//#transition 'T5.10'
        		if((networkCarWeightTranslator.getValue() >= Elevator.MaxCarCapacity)) {
        			newState = State.STATE_OPEN;
        		//#transition 'T5.9'
        		} else if (networkDoorClosedTranslator.getValue()) {
        			newState = State.STATE_STOP_CLOSED;
        		} else {
        			reversalCount = 0;
        			localDoorMotor.set(DoorCommand.NUDGE);
	        		networkDoorMotorTranslator.set(DoorCommand.NUDGE.ordinal());
	        		newState = State.STATE_NUDGE;
        		}
        		break;
        	case STATE_CLOSE:
        		//#transition 'T5.3'
        		if((networkCarWeightTranslator.getValue() >= Elevator.MaxCarCapacity) 
        				|| callSign) {
        			newState = State.STATE_OPEN;
        		//#transition 'T5.6'
        		} else if ((networkDoorReversalCanPayloadTranslator_1.getValue())
        				|| (networkDoorReversalCanPayloadTranslator_2.getValue())) {
        			reversalCount ++;
        			newState = State.STATE_RE_OPEN;
        		//#transition 'T5.5'
        		}else if (networkDoorClosedTranslator.getValue()) {
        			newState = State.STATE_STOP_CLOSED;
        		} else {
        			localDoorMotor.set(DoorCommand.CLOSE);
	        		networkDoorMotorTranslator.set(DoorCommand.CLOSE.ordinal());
	        		newState = State.STATE_CLOSE;
        		}
        		break;
        	case STATE_OPEN:
                //#transition 'T5.1'
        		if (networkDoorOpenedTranslator.getValue()) {
        			//keep down the system time when the door opened.
        			openTime = Harness.getTime();	
        			//get the dwell
        			if (networkDesiredDwellTranslator.getValue() >= 0) {
        	            currentDwell = new SimTime(networkDesiredDwellTranslator.getValue(),
        	            		SimTime.SimTimeUnit.MILLISECOND);
        	        }
        			newState = State.STATE_STOP_OPEN;
                } else {//stay
                    localDoorMotor.set(DoorCommand.OPEN);
                    networkDoorMotorTranslator.set(DoorCommand.OPEN.ordinal());
                    newState = State.STATE_OPEN;
                }
        		break;
        	case STATE_RE_OPEN:
        		//#transition 'T5.7'
        		if (networkDoorOpenedTranslator.getValue()) {
        			//keep down the system time when the door opened.
        			openTime = Harness.getTime();	
        			//get the dwell
        			if (networkDesiredDwellTranslator.getValue() >= 0) {
        	            currentDwell = new SimTime(networkDesiredDwellTranslator.getValue(),
        	            		SimTime.SimTimeUnit.MILLISECOND);
        	        }
        			newState = State.STATE_STOP_OPEN;
                } else {//stay
                    localDoorMotor.set(DoorCommand.OPEN);
                    networkDoorMotorTranslator.set(DoorCommand.OPEN.ordinal());
                    newState = State.STATE_RE_OPEN;
                }
        		break;
        }
                //log the results of this iteration
       if (state == newState) {
            log("remains in state: ",state);

        } else {
            log("Transition:",state,"->",newState);
            //System.out.println("Transition:"+state+"->" + newState);
        }
        
        state = newState;
        //report the current state
        setState(STATE_KEY,newState.toString());
        timer.start(period);
	}
}
