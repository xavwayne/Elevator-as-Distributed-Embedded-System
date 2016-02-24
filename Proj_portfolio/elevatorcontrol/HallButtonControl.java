/**
 * 18649-Fall-2015
 * Group 3
 * Jiyu Shi(jiyus) ; Shuai Wang(shuaiwa1); Xiaoyu Wang(xiaoyuw); Xiao Guo(xiaog)
 */
package simulator.elevatorcontrol;

import jSimPack.SimTime;

import simulator.elevatorcontrol.DesiredFloorCanPayloadTranslator;

import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.elevatormodules.AtFloorCanPayloadTranslator;

import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.Harness;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;

import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.HallCallPayload;
import simulator.payloads.HallCallPayload.ReadableHallCallPayload;
import simulator.payloads.HallLightPayload;
import simulator.payloads.HallLightPayload.WriteableHallLightPayload;

import simulator.payloads.CANNetwork;
import simulator.payloads.PhysicalNetwork;
import simulator.payloads.PhysicalPayload;

/**
 * This HallButtonControl uses hall calls and hall lights, so it can be 
 * instantiated for any [floor,hallway,direction] combination.
 *
 * @author Shuai Wang
 */
public class HallButtonControl extends Controller{
	//inputs are Readable objects, while outputs are Writeable objects

	/* 
	 * local physical state
	 */
	//HallCall[f,b,d] input
	private ReadableHallCallPayload localHallCall;  
	//HallLight[f,b,d] output   
    private WriteableHallLightPayload localHallLight;  

	/*
	 * network interface
	 */
	//receive hall call from the other button
    //private WriteableCanMailbox networkHallLightOut; //deleted in project11
    private WriteableCanMailbox networkHallCallOut;
	//translator for the hall light message(generic translator)
	//mHallLight[f,b,d] output
	//private BooleanCanPayloadTranslator mHallLight;	//deleted in project11    
	//mHallCall[f,b,d] output
	private NewBooleanCanPayloadTranslator mHallCall;     

	//received door closed and at floor message
	private ReadableCanMailbox networkDoorClosedLeft;
    private ReadableCanMailbox networkDoorClosedRight;
	private ReadableCanMailbox networkAtFloor;   

	//translator for the doorClosed and atFloor message -- this translator 
	//is specific to this messages, and is provided the elevatormodules package
	private DoorClosedCanPayloadTranslator mDoorClosedLeft;
    private DoorClosedCanPayloadTranslator mDoorClosedRight;
    //mAtFloor[f,b] input
	private AtFloorCanPayloadTranslator mAtFloor;

    //mDesiredFloor
    ReadableCanMailbox networkDesiredFloor;
    DesiredFloorCanPayloadTranslator networkDesiredFloorTranslator;

	private final Hallway hallway;
	private final Direction direction;
	private final int floor;
	private SimTime period;
    private boolean bothDoorClosed;

    private enum State {
    	STATE_HALL_LIGHT_OFF,
    	STATE_HALL_LIGHT_ON,
    }
    private State state = State.STATE_HALL_LIGHT_OFF;

    /**
     * Constructor to initialize HallButtonControl
     *
     * @param floor which floor the hall call happens
     *
     * @param hallway which hallway the hall call happens
     *
     * @param direction which direction the hall call happens
     *
     * @param period time period for message been send
     *
     * @param verbose decide if Log is enabled
     *
     */
    public HallButtonControl(int floor, Hallway hallway, 
    	Direction direction, SimTime period, boolean verbose) {
    	//call to the Controller superclass constructor is required
    	super("HallButtonControl" + ReplicationComputer.
    					makeReplicationString(floor, hallway, direction), verbose);

    	//store the constructor arguments in internal state
    	this.period = period;
    	this.hallway = hallway;
    	this.direction = direction;
    	this.floor = floor;

    	log("Created hallButtonControl with period = ", period);

    	/*
    	 * initialize physical state
    	 */
    	//create a payload object for this floor, hallway, direction using 
    	//the static factory method in HallCallPayload
    	localHallCall = HallCallPayload.getReadablePayload(
    								floor, hallway, direction);
    	//register the payload with the physical interface(as in input) 
    	//-- it will be updated periodically 
    	//when the hall call button state is modified.
    	physicalInterface.registerTimeTriggered(localHallCall);

    	//create a payload object for this floor,hallway, direction
    	//this is an output, so it is created with
    	//the Writeable static factory method
    	localHallLight = HallLightPayload.getWriteablePayload(
    								floor, hallway, direction);
    	//register the payload to be sent periodically -- whatever value 
    	//is stored in the localHallLight object will be sent out periodically 
    	//with the period specified by the period parameter.
    	physicalInterface.sendTimeTriggered(localHallLight, period);

    	/*
    	 * initialize network interface
    	 */
    	//create a can mailbox - this object has the binary representation 
    	//of the message data the CAN message ids are declared 
    	//in the MessageDictionary class. The ReplicationComputer class provides
    	//utility methods for computing offsets for replicated comtrollers
//    	networkHallLightOut = CanMailbox.getWriteableCanMailbox(
//    					MessageDictionary.HALL_LIGHT_BASE_CAN_ID + 
//    					ReplicationComputer.computeReplicationId(
//    						floor, hallway, direction));
    	networkHallCallOut = CanMailbox.getWriteableCanMailbox(
    					MessageDictionary.HALL_CALL_BASE_CAN_ID +
    					ReplicationComputer.computeReplicationId(
    						floor, hallway, direction));

    	//Create a translator with a reference to the CanMailbox.  Use the 
        //translator to read and write values to the mailbox
        //mHallLight = new BooleanCanPayloadTranslator(networkHallLightOut);
        mHallCall = new NewBooleanCanPayloadTranslator(networkHallCallOut);
        //register the mailbox to have its value broadcast on the network 
        //periodically with a period specified by the period parameter.
//        canInterface.sendTimeTriggered(networkHallLightOut, period);
        canInterface.sendTimeTriggered(networkHallCallOut, period);


        //registration for the DoorClosed and atFloor message are 
        //similar to the mHallLight message
        networkDoorClosedLeft = CanMailbox.getReadableCanMailbox(
        				MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + 
        				ReplicationComputer.computeReplicationId(
        					hallway, Side.LEFT));
        networkDoorClosedRight = CanMailbox.getReadableCanMailbox(
                        MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + 
                        ReplicationComputer.computeReplicationId(
                            hallway, Side.RIGHT));
       
        networkAtFloor = CanMailbox.getReadableCanMailbox(
        				MessageDictionary.AT_FLOOR_BASE_CAN_ID +
        				ReplicationComputer.computeReplicationId(
        					floor, hallway));

        mDoorClosedLeft = new DoorClosedCanPayloadTranslator(
        				networkDoorClosedLeft, hallway, Side.LEFT);
        mDoorClosedRight = new DoorClosedCanPayloadTranslator(
                        networkDoorClosedRight, hallway, Side.RIGHT);

        mAtFloor = new AtFloorCanPayloadTranslator(
        					networkAtFloor, floor, hallway);
        //register to receive preiodic updates to the mailbox 
        //via the CAN network, the period of updates will be 
        //determined by the sender of the message
        canInterface.registerTimeTriggered(networkDoorClosedLeft);
        canInterface.registerTimeTriggered(networkDoorClosedRight);
        
        canInterface.registerTimeTriggered(networkAtFloor);


        //mDesiredFloor
        networkDesiredFloor = CanMailbox.getReadableCanMailbox(
                        MessageDictionary.DESIRED_FLOOR_CAN_ID);
        networkDesiredFloorTranslator = new DesiredFloorCanPayloadTranslator(
                        networkDesiredFloor);
        canInterface.registerTimeTriggered(networkDesiredFloor);


        timer.start(period);
    }

    /**
     * The timer callback is where the main controller code is executed.
     * Each case block executes actions for that state, then executes
     * a transition to the next state if the transition conditions are met.
     *
     * @param callbackData 
     */
    public void timerExpired(Object callbackData) {
    	State newState = state;
    	switch (state) {
    		case STATE_HALL_LIGHT_OFF:
    			//state actions for 'HALL LIGHT OFF'
                localHallLight.set(false);
                //mHallLight.set(false);
                mHallCall.set(false);

    			//#transition 'T8.1'
    			if (localHallCall.pressed() == true) {
    				newState = State.STATE_HALL_LIGHT_ON;
    			} else {
    				newState = state;
    			}
                break;
            case STATE_HALL_LIGHT_ON:
            	//state actions for 'HALL LIGHT ON'
                localHallLight.set(true);
                //mHallLight.set(true);
                mHallCall.set(true);

                if (localHallCall.pressed() == false) {
                    bothDoorClosed = mDoorClosedLeft.getValue() && mDoorClosedRight.getValue();
                    //#transition 'T8.2'
                    if(mAtFloor.getValue() == true
                            && !bothDoorClosed 
                            && (networkDesiredFloorTranslator.getDirection() == this.direction 
                                || networkDesiredFloorTranslator.getDirection() == Direction.STOP)) {
                        newState = State.STATE_HALL_LIGHT_OFF;
                    } else {
                        newState = state;
                    }
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

    	//update the state variable
        state = newState;

        //report the current state
        setState(STATE_KEY,newState.toString());

        //schedule the next iteration of the controller
        timer.start(period);
    }
}