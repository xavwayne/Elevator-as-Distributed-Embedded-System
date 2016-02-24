/**
 * 18649-Fall-2015
 * Group 3
 * Jiyu Shi(jiyus) ; Shuai Wang(shuaiwa1); Xiaoyu Wang(xiaoyuw); Xiao Guo(xiaog)
 */
package simulator.elevatorcontrol;
/*
**
 *
 * @author Jiyu Shi
 */

import jSimPack.SimTime;

import simulator.elevatormodules.DoorClosedCanPayloadTranslator;

import simulator.elevatormodules.AtFloorCanPayloadTranslator; //used for get info of mAtFloor

import simulator.framework.Controller;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;


//those three parts are important to receive all the message !!!
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;

//this is only neccessary for physical connection
import simulator.payloads.CarCallPayload;
import simulator.payloads.CarCallPayload.ReadableCarCallPayload;
import simulator.payloads.CarLightPayload;
import simulator.payloads.CarLightPayload.WriteableCarLightPayload;

import simulator.elevatorcontrol.MessageDictionary;

public class CarButtonControl extends Controller {
    //local physical state
    private ReadableCarCallPayload localCarCall;    //CarCall[f,b]   input
    private WriteableCarLightPayload localCarLight; //CarLight[f,b]  output

    //network interface
    // receive hall call from the other button
    //private WriteableCanMailbox networkCarLightOut;  //Store 8 buttons or only one? output mCarLight //deleted in project11
    private WriteableCanMailbox networkCarCaLLOut;
    // translator for the hall light message -- this is a generic translator
    private NewBooleanCanPayloadTranslator mCarCall;    //output
	//private CustomizedBooleanCanPayloadTranslator mCarLight;	 //output	//deleted in project11

    //received door closed message
    private ReadableCanMailbox networkDoorClosed;
    private ReadableCanMailbox networkAtFloor;  //input mAtFloor[f,b]

    //translator for the doorClosed message -- this translator is specific
    //to this messages, and is provided the elevatormodules package
    private DoorClosedCanPayloadTranslator mDoorClosed;
	private AtFloorCanPayloadTranslator mAtFloor;

    //these variables keep track of which instance this is.
    private final Hallway hallway;
    private final int floor;
    //store the period for the controller
    private SimTime period;

    //enumerate states
    private enum State {
        STATE_LIGHT_ON,
        STATE_LIGHT_OFF,
    }
    //state variable initialized to the initial state FLASH_OFF
    private State state = State.STATE_LIGHT_OFF;
    //constructor
	public CarButtonControl( int floor, Hallway hallway, SimTime period, boolean verbose){
		super("CarButtonControl" + ReplicationComputer.makeReplicationString(floor, hallway), verbose); //!!!NEED TO CONFIRM REPLICATIONCOM
		this.floor = floor;
		this.period = period;
		this.hallway = hallway;

        log("Created testlight with period = ", period);

        //initialize physical state
        //create a payload object for this floor,hallway,direction using the
        //static factory method in HallCallPayload.
        localCarCall = CarCallPayload.getReadablePayload(floor, hallway);
        localCarLight = CarLightPayload.getWriteablePayload(floor, hallway);

        physicalInterface.registerTimeTriggered(localCarCall);   
        physicalInterface.sendTimeTriggered(localCarLight, period);  
        // above two used for receive p-carcall and send p-carlight

//        networkCarLightOut = CanMailbox.getWriteableCanMailbox(MessageDictionary.CAR_LIGHT_BASE_CAN_ID + 
//        											ReplicationComputer.computeReplicationId(floor, hallway));
//		
        
		networkCarCaLLOut = CanMailbox.getWriteableCanMailbox(MessageDictionary.CAR_CALL_BASE_CAN_ID +
													 ReplicationComputer.computeReplicationId(floor, hallway));

//        mCarLight = new CustomizedBooleanCanPayloadTranslator(networkCarLightOut);
        mCarCall = new NewBooleanCanPayloadTranslator(networkCarCaLLOut);

//        canInterface.sendTimeTriggered(networkCarLightOut, period);
        canInterface.sendTimeTriggered(networkCarCaLLOut, period);
        //above two used for sending mCarlight and mCarCall

        /*
         * Registration for the DoorClosed message is similar to the mHallLight message
         * 
         * To register for network messages from the smart sensors or other objects
         * defined in elevator modules, use the translators already defined in
         * elevatormodules package.  These translators are specific to one type
         * of message.
         */
        networkDoorClosed = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID 
        					+ ReplicationComputer.computeReplicationId(hallway, Side.LEFT));

        //networkDoorClosedBackLeft = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID 
        //					+ ReplicationComputer.computeReplicationId( hallway, Side.LEFT));
		
		networkAtFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID 
        					+ ReplicationComputer.computeReplicationId(floor, hallway));

        mDoorClosed = new DoorClosedCanPayloadTranslator(networkDoorClosed, hallway , Side.LEFT);
        //mDoorClosedBackLeft = new DoorClosedCanPayloadTranslator(networkDoorClosedFrontLeft, Hallway.BACK, Side.LEFT);
        mAtFloor = new AtFloorCanPayloadTranslator(networkAtFloor, floor ,hallway);
        //register to receive periodic updates to the mailbox via the CAN network
        //the period of updates will be determined by the sender of the message
        canInterface.registerTimeTriggered(networkDoorClosed);
        canInterface.registerTimeTriggered(networkAtFloor);
        //canInterface.registerTimeTriggered(networkDoorClosedBackLeft);

        /* issuing the timer start method with no callback data means a NULL value 
         * will be passed to the callback later.  Use the callback data to distinguish
         * callbacks from multiple calls to timer.start() (e.g. if you have multiple
         * timers.
         */
        timer.start(period);
	}

	public void timerExpired(Object callbackData) {
		State newState = state;
        switch (state) {
        	case STATE_LIGHT_OFF:
        	//state actions for "LIGHT OFF"
        		 localCarLight.set(false);
        		 //mCarLight.set(false);
        		 mCarCall.set(false); 
                 //#transition 'T9.2'
        		 if(localCarCall.pressed() == true){ //&& mAtFloor.getValue() == false
        		 	newState = State.STATE_LIGHT_ON;
        		 }
        		 break;
        	case STATE_LIGHT_ON:
        		 localCarLight.set(true);
        		 //mCarLight.set(true);
        		 mCarCall.set(true);
                 //#transition 'T9.1'
        		 if(mDoorClosed.getValue() == false && mAtFloor.getValue() == true && localCarCall.pressed() != true){
        		 	newState = State.STATE_LIGHT_OFF;
        		 }
        		 break;
            default:
                throw new RuntimeException("State " + state + " was not recognized.");
        }
        
        //log the results of this iteration
        if (state == newState) {
            log("remains in state: ",state);
        } else {
            log("Transition:",state,"->",newState);
        }

        //update the state variable
        state = newState;

        //report the current state
        setState(STATE_KEY,newState.toString());

        //schedule the next iteration of the controller
        //you must do this at the end of the timer callback in order to restart
        //the timer
        timer.start(period);
    }

}