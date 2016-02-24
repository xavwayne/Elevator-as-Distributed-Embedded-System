/**
 * 18649-Fall-2015
 * Group 3
 * Jiyu Shi(jiyus) ; Shuai Wang(shuaiwa1); Xiaoyu Wang(xiaoyuw); Xiao Guo(xiaog)
 */
package simulator.elevatorcontrol;

import java.util.HashMap;

import jSimPack.SimTime;
import simulator.elevatorcontrol.Utility.AtFloorArray;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.framework.TimeSensitive;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.CarLanternPayload;
import simulator.payloads.CarLanternPayload.WriteableCarLanternPayload;

/**
 * @author Xiao Guo
 * 
 * Lantern Control
 * Implemented with compare between current floor and desired floor.
 */
public class LanternControl extends Controller implements TimeSensitive {

	//store the period for the controller
    private SimTime period;
    
    //inputs
    //mDesiredFloor
    ReadableCanMailbox networkDesiredFloor;
    DesiredFloorCanPayloadTranslator networkDesiredFloorTranslator;
    //mAtFloor[f, b]
    AtFloorArray atFloorArray;
    //mDoorClosed[b, r]
    HashMap<Integer, DoorClosedCanPayloadTranslator> doorClosedArray;
    
    //outputs
    WriteableCarLanternPayload localCarLantern;
    WriteableCanMailbox networkCarLantern;
    NewBooleanCanPayloadTranslator networkCarLanternTranslator;
    
    //these variables keep track of the elevator's information.
    private Direction currentDirection;
    private Direction desiredDirection;
    private int currentFloor;
    private int desiredFloor;
     
    //enumerate states
    private enum State {
    	STATE_ON,
        STATE_OFF,
    }
    
    //state variable initialized to the initial state STATE_STOP_CLOSED
    private State state = State.STATE_OFF;
    
	/**
	 * @param direction
	 * @param simTime
	 * @param verbose
	 */
	public LanternControl(Direction direction, SimTime simTime, boolean verbose) {
		super("LanternControl" + ReplicationComputer.makeReplicationString(direction), verbose);
		
		//Initialization
		period = simTime;
		currentDirection = direction;
		desiredDirection = Direction.STOP;
		
		//inputs
        //mDesiredFloor
        networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
        networkDesiredFloorTranslator = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
        canInterface.registerTimeTriggered(networkDesiredFloor);
        //mAtFloor
        atFloorArray = new Utility.AtFloorArray(canInterface);
        //mDoorClosed
        doorClosedArray = new HashMap<Integer, DoorClosedCanPayloadTranslator>();
        for (Hallway hallway : Hallway.replicationValues) {
            for (Side side : Side.values()) {
            	int key = MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID  +
                		ReplicationComputer.computeReplicationId(hallway, side);
            	ReadableCanMailbox networkDoorClosed = CanMailbox.getReadableCanMailbox(key);
            	doorClosedArray.put(ReplicationComputer.computeReplicationId(hallway, side), 
            			new DoorClosedCanPayloadTranslator(networkDoorClosed, hallway, side));
            	canInterface.registerTimeTriggered(networkDoorClosed);
            }
        }
        
        //outputs
        //CarLantern
        localCarLantern = CarLanternPayload.getWriteablePayload(direction);
        localCarLantern.set(false);
        physicalInterface.sendTimeTriggered(localCarLantern, simTime);
        //mCarLantern
        networkCarLantern = CanMailbox.getWriteableCanMailbox(MessageDictionary.CAR_LANTERN_BASE_CAN_ID +
        		ReplicationComputer.computeReplicationId(direction));
        networkCarLanternTranslator = new NewBooleanCanPayloadTranslator(networkCarLantern);
        networkCarLanternTranslator.set(false);
        canInterface.sendTimeTriggered(networkCarLantern, simTime);
        
        //set the timer
        timer.start(period);
	}

	/* 
	 * Two states: On and OFF
	 */
	@Override
	public void timerExpired(Object callbackData) {
		//next state
		State newState = state;
		//read inputs in
		currentFloor = atFloorArray.getCurrentFloor();
		desiredDirection = networkDesiredFloorTranslator.getDirection();
		desiredFloor = networkDesiredFloorTranslator.getFloor();
		//log the start statue
		log("Current Floor=", currentFloor);
		log("Start State=" + state);
		
        switch (state) {
        case STATE_OFF:
        	//desired floor method
        	//#transition 'T7.1'
        	if((!isAllDoorsClosed()) && (desiredDirection == currentDirection)) {
        		newState = State.STATE_ON;
        	}
        	//floor comparing method
        	/*
        	//Sort this control as up and down
        	if (currentDirection == Direction.UP) {
        		if ((!isAllDoorsClosed()) && (desiredFloor > currentFloor)) {
        			newState = State.STATE_ON;
        		}
        	} else if (currentDirection == Direction.DOWN) {
        		//floor comparing method
        		if ((!isAllDoorsClosed()) && (desiredFloor < currentFloor)) {
        			newState = State.STATE_ON;
        		}
        	}
        	*/
        	//Outputs settings
        	localCarLantern.set(false);
        	networkCarLanternTranslator.set(false);
        	break;
        case STATE_ON:
        	if (isAllDoorsClosed() || (desiredDirection == Direction.STOP)) {
        		//#transition 'T7.2'
        		newState = State.STATE_OFF;
        	}
        	//Outputs settings
        	localCarLantern.set(true);
        	networkCarLanternTranslator.set(true);
        	break;
        }
        state = newState;
        //report the current state
        setState(STATE_KEY,newState.toString());
        log("End State=" + state);
        timer.start(period);
	}
	
	/* 
	 * isAllDoorsClosed()
	 * A helper class to test if all doors are closed.
	 */
	private boolean isAllDoorsClosed() {
		boolean result = true;
		for (Hallway hallway : Hallway.replicationValues) {
            for (Side side : Side.values()) {
            	result = result && doorClosedArray.get
            			(ReplicationComputer.computeReplicationId(hallway, side)).getValue();
            }
        }
		return result;
	}
}
