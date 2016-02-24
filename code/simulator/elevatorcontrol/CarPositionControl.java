/**
 * 18649-Fall-2015
 * Group 3
 * Jiyu Shi(jiyus) ; Shuai Wang(shuaiwa1); Xiaoyu Wang(xiaoyuw); Xiao Guo(xiaog)
 */
package simulator.elevatorcontrol;

import java.util.Map;
import java.util.HashMap;

import jSimPack.SimTime;

import simulator.elevatormodules.CarLevelPositionCanPayloadTranslator;
import simulator.elevatorcontrol.NewIntegerCanPayloadTranslator;

import simulator.elevatorcontrol.DesiredFloorCanPayloadTranslator;
import simulator.framework.Direction;

import simulator.elevatorcontrol.CarPositionControl;
import simulator.elevatorcontrol.Utility;
import simulator.elevatorcontrol.Utility.AtFloorArray;
import simulator.framework.Controller;
import simulator.framework.Elevator;
import simulator.framework.TimeSensitive;
import simulator.framework.Timer;
import simulator.payloads.CANNetwork;

import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;

import simulator.payloads.CarPositionIndicatorPayload;
import simulator.payloads.CarPositionIndicatorPayload.WriteableCarPositionIndicatorPayload;

import simulator.payloads.PhysicalNetwork;
import simulator.payloads.PhysicalPayload;


/**
 * This is for CarPositionControl.
 *
 * @author Shuai Wang
 */
public class CarPositionControl extends Controller 
			implements TimeSensitive {
	//inputs are Readable objects, while outputs are Writeable objects

	//CarPositionIndicator
	private WriteableCarPositionIndicatorPayload localCarPositionIndicator;
	
    //mCarPositionIndicator
    private NewIntegerCanPayloadTranslator mCarPositionIndicator;
    private WriteableCanMailbox networkCarPositionIndicatorOut;

    //mCarLevelPosition
	private CarLevelPositionCanPayloadTranslator mCarLevelPosition;
    private ReadableCanMailbox networkCarLevelPosition;
	//mDriveSpeed
	private NewIntegerCanPayloadTranslator mDriveSpeed;
    private ReadableCanMailbox networkDriveSpeed;
    //mDesiredFloor
    private ReadableCanMailbox networkDesiredFloor;
    private DesiredFloorCanPayloadTranslator mDesiredFloor;
	
	
	private AtFloorArray atFloorArray;

	private int numFloors;
	private int currentFloor;
    private int lastFloor;

	private SimTime period;

    private double Ts=0.3;

	//enumerate states
    private enum State {
    	STATE_AT_FLOOR,
    	STATE_BETWEEN_FLOORS,
    }
    private State state = State.STATE_AT_FLOOR;

    private double currentSpeed;
    private double position;

    private Map<Integer, Double> commitPointUpMap = new HashMap<>();
    private Map<Integer, Double> commitPointDownMap = new HashMap<>();

    public CarPositionControl(SimTime period, boolean verbose) {
    	super("CarPositionControl", verbose);

    	this.period = period;
        this.numFloors = Elevator.numFloors;
        this.currentFloor = 1;
        this.lastFloor = 1;

        log("Created CarPositionControl with period = ", period);

        localCarPositionIndicator = CarPositionIndicatorPayload.getWriteablePayload();
        physicalInterface.sendTimeTriggered(
        	(PhysicalPayload.PhysicalWriteablePayload)localCarPositionIndicator, period);


        //mCarPositionIndicator : output
        networkCarPositionIndicatorOut = CanMailbox.getWriteableCanMailbox(
                        MessageDictionary.CAR_POSITION_CAN_ID);
        mCarPositionIndicator = new NewIntegerCanPayloadTranslator(
            networkCarPositionIndicatorOut);
        canInterface.sendTimeTriggered(networkCarPositionIndicatorOut, period);


        //networkCarLevelPosition
        networkCarLevelPosition = CanMailbox.getReadableCanMailbox(
        	MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
        mCarLevelPosition = new CarLevelPositionCanPayloadTranslator(
        	networkCarLevelPosition);
        canInterface.registerTimeTriggered(networkCarLevelPosition);
        
        //networkDriveSpeed
        networkDriveSpeed = CanMailbox.getReadableCanMailbox(
        	MessageDictionary.DRIVE_SPEED_CAN_ID);
        mDriveSpeed = new NewIntegerCanPayloadTranslator(
        	networkDriveSpeed);
        canInterface.registerTimeTriggered(networkDriveSpeed);
        
        //networkDesidredFloor
        networkDesiredFloor = CanMailbox.getReadableCanMailbox(
                        MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(
                        networkDesiredFloor);
        canInterface.registerTimeTriggered(networkDesiredFloor);
        

        atFloorArray = new AtFloorArray(canInterface);
        timer.start(period);
    }

	public void timerExpired(Object callbackData) {
        currentSpeed = mDriveSpeed.getValue()/1000.0;
		State newState = state;
        switch (state) {
            case STATE_AT_FLOOR:
                localCarPositionIndicator.set(currentFloor);
                mCarPositionIndicator.set(currentFloor);

                lastFloor = currentFloor;

                //#transition 'T10.1'
                if(mDriveSpeed.getValue() != 0) {
                    newState = State.STATE_BETWEEN_FLOORS;
                }
                break;
            case STATE_BETWEEN_FLOORS:

                localCarPositionIndicator.set(currentFloor);
                mCarPositionIndicator.set(currentFloor);

                lastFloor = currentFloor;
                
                int destinationFloor = mDesiredFloor.getFloor();
                int diff = destinationFloor - currentFloor;

                //calculate commit point
                if(diff > 0) {
                    computeCommitPointUp();

                    position = mCarLevelPosition.getPosition();  //milimeters
                    double nextCommitPoint = commitPointUpMap.get(currentFloor + 1);
                   
                    if(position >= nextCommitPoint) {
                        currentFloor = lastFloor + 1;
                    }
                } else if(diff < 0) {
                    computeCommitPointDown();

                    position = mCarLevelPosition.getPosition();  //milimeters
                    double nextCommitPoint = commitPointDownMap.get(currentFloor - 1);
                    
                    if(position <= nextCommitPoint) {
                        currentFloor = lastFloor - 1;
                    }
                }
                
                //#transition 'T10.2'
                if(mDriveSpeed.getValue() == 0) {
                    newState = State.STATE_AT_FLOOR;
                }
                break;
            default:
                throw new RuntimeException(
                        "State " + state + " was not recognized.");
        }

        //log the results of this iteration
    	if (state == newState) {
    		log("remains in state: ", state);
    	} else {
    		log("Transition:", state, "->", newState);
    	}

    	state = newState;
        //report the current state
        setState(STATE_KEY,newState.toString());
        //schedule the next iteration of the controller
        timer.start(period);
    }

    private void computeCommitPointUp() {
        int destinationFloor = mDesiredFloor.getFloor();
        for(int i = currentFloor + 1; i <= destinationFloor; i++) {
            double desiredPosition = (i - 1) * 5;
            double commitPoint = desiredPosition - (currentSpeed * currentSpeed / 2 
                                    + currentSpeed * 2 * Ts);
            commitPointUpMap.put(i, commitPoint*1000);
        }
    }

    private void computeCommitPointDown() {
        int destinationFloor = mDesiredFloor.getFloor();
        for(int i = currentFloor - 1; i >= destinationFloor; i--) {
            double desiredPosition = (i - 1) * 5;
            double commitPoint = desiredPosition + (currentSpeed * currentSpeed / 2 
                                    + currentSpeed * 2 * Ts);
            commitPointDownMap.put(i, commitPoint*1000);
        }
    }

}