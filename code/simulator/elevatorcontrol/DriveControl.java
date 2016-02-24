/**
 * 18649-Fall-2015
 * Group 3
 * Jiyu Shi(jiyus) ; Shuai Wang(shuaiwa1); Xiaoyu Wang(xiaoyuw); Xiao Guo(xiaog)
 */
package simulator.elevatorcontrol;

import java.util.*;
import jSimPack.SimTime;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.elevatormodules.SafetySensorCanPayloadTranslator;
import simulator.elevatormodules.CarWeightCanPayloadTranslator;
import simulator.elevatormodules.LevelingCanPayloadTranslator;
import simulator.elevatormodules.CarLevelPositionCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.framework.Elevator;
import simulator.framework.Speed;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.DrivePayload;
import simulator.payloads.DrivePayload.WriteableDrivePayload;
import simulator.payloads.DriveSpeedPayload;
import simulator.payloads.DriveSpeedPayload.ReadableDriveSpeedPayload;
import simulator.elevatorcontrol.NewIntegerCanPayloadTranslator;
import simulator.elevatorcontrol.Utility.DoorClosedArray;
import simulator.elevatorcontrol.Utility.AtFloorArray;

/**
 *
 * Author: xiaoyuw
 */
public class DriveControl extends Controller{

    //local physical state
    private ReadableDriveSpeedPayload localDriveSpeed;
    private WriteableDrivePayload localDrive;

    //network interfaces
    //output
    private WriteableCanMailbox networkDriveOut;
    private DriveCommandCanPayloadTranslator mDrive;

    private WriteableCanMailbox networkDriveSpeedOut;
    private NewIntegerCanPayloadTranslator mDriveSpeed;

    //inputs

    private ReadableCanMailbox networkLevelUp;
    private LevelingCanPayloadTranslator mLevelUp;

    private ReadableCanMailbox networkLevelDown;
    private LevelingCanPayloadTranslator mLevelDown;

    private AtFloorArray mAtFloors;

    private DoorClosedArray mDoorClosedFront;
    private DoorClosedArray mDoorClosedBack;

    private ReadableCanMailbox networkEmergencyBrake;
    private SafetySensorCanPayloadTranslator mEmergencyBrake;

    private ReadableCanMailbox networkDesiredFloor;
    private DesiredFloorCanPayloadTranslator mDesiredFloor;

    private ReadableCanMailbox networkCarWeight;
    private CarWeightCanPayloadTranslator mCarWeight;   

    private ReadableCanMailbox networkCarLevelPosition;
    private CarLevelPositionCanPayloadTranslator mCarLevelPosition;

    private ReadableCanMailbox networkCarPositionIndicator;
    private NewIntegerCanPayloadTranslator mCarPositionIndicator;


    //store the period for the controller
    private SimTime period;

    private double SlowSpeed=0.25;
    
     //enumerate states
    private enum State {
        STATE_STOP,
        STATE_LEVEL_UP,
        STATE_LEVEL_DOWN,
        STATE_SLOW_UP,
        STATE_SLOW_DOWN,
        STATE_FAST_UP,
        STATE_FAST_DOWN,
        STATE_EMERGENCY,
    }
    //state variable initialized to the initial state STATE_STOP
    private State state = State.STATE_STOP;

    //constructor
    public DriveControl(SimTime period, boolean verbose){
        super("DriveControl",verbose);
        this.period=period;

        log("Created DriveControl with period = ", period);

        //initialize physical objects
        
        //inputs

        //DriveSpeed
        localDriveSpeed=DriveSpeedPayload.getReadablePayload();
        physicalInterface.registerTimeTriggered(localDriveSpeed);

        //outputs

        //Drive
        localDrive=DrivePayload.getWriteablePayload();
        physicalInterface.sendTimeTriggered(localDrive,period);   


        //initialize network interfaces

        //inputs

        //mCarPositionIndicator
        networkCarPositionIndicator=CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_POSITION_CAN_ID);
        mCarPositionIndicator=new NewIntegerCanPayloadTranslator(networkCarPositionIndicator);
        canInterface.registerTimeTriggered(networkCarPositionIndicator);

        //mCarLevelPosition
        networkCarLevelPosition=CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
        mCarLevelPosition=new CarLevelPositionCanPayloadTranslator(networkCarLevelPosition);
        canInterface.registerTimeTriggered(networkCarLevelPosition);

        //mLevelUp
        networkLevelUp=CanMailbox.getReadableCanMailbox(MessageDictionary.LEVELING_BASE_CAN_ID+ReplicationComputer.computeReplicationId(Direction.UP));
        mLevelUp=new LevelingCanPayloadTranslator(networkLevelUp,Direction.UP);
        canInterface.registerTimeTriggered(networkLevelUp);

        //mLevelDown
        networkLevelDown=CanMailbox.getReadableCanMailbox(MessageDictionary.LEVELING_BASE_CAN_ID+ReplicationComputer.computeReplicationId(Direction.DOWN));
        mLevelDown=new LevelingCanPayloadTranslator(networkLevelDown,Direction.DOWN);
        canInterface.registerTimeTriggered(networkLevelDown);

        //mDesiredFloor
        networkDesiredFloor=CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor=new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
        canInterface.registerTimeTriggered(networkDesiredFloor);

        //mDoorClosed      
        mDoorClosedFront=new DoorClosedArray(Hallway.FRONT,canInterface);
        mDoorClosedBack=new DoorClosedArray(Hallway.BACK,canInterface);
        
        //mAtFloor FH+BH
        mAtFloors = new AtFloorArray(canInterface);

        //mEmergencyBrake
        networkEmergencyBrake=CanMailbox.getReadableCanMailbox(MessageDictionary.EMERGENCY_BRAKE_CAN_ID);
        mEmergencyBrake=new SafetySensorCanPayloadTranslator(networkEmergencyBrake);
        canInterface.registerTimeTriggered(networkEmergencyBrake);

        //mCarWeight
        networkCarWeight=CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_CAN_ID);
        mCarWeight=new CarWeightCanPayloadTranslator(networkCarWeight);
        canInterface.registerTimeTriggered(networkCarWeight);

        //outputs

        //mDrive
        networkDriveOut=CanMailbox.getWriteableCanMailbox(MessageDictionary.DRIVE_COMMAND_CAN_ID);
        mDrive=new DriveCommandCanPayloadTranslator(networkDriveOut);
        canInterface.sendTimeTriggered(networkDriveOut,period);

        //mDriveSpeed
        networkDriveSpeedOut=CanMailbox.getWriteableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
        mDriveSpeed=new NewIntegerCanPayloadTranslator(networkDriveSpeedOut);
        canInterface.sendTimeTriggered(networkDriveSpeedOut,period);

        /* issuing the timer start method with no callback data means a NULL value 
         * will be passed to the callback later.  Use the callback data to distinguish
         * callbacks from multiple calls to timer.start() (e.g. if you have multiple
         * timers.
         */
        timer.start(period);  

    }

     // Timer callback is where the main controller code is executed.
    public void timerExpired(Object callbackData) {

        State newState = state;
        switch(state){
            case STATE_STOP:
            //state actions for STATE_STOP
            localDrive.set(Speed.STOP, Direction.STOP);
            mDrive.set(Speed.STOP, Direction.STOP);
            mDriveSpeed.set((int) (localDriveSpeed.speed()*1000));
            // System.out.println(mAtFloors.getCurrentFloor());

            //#transition 'T6.1'            
            if(localDriveSpeed.speed()*1000==0 && allDoorClosed()==true && mDesiredFloor.getFloor() > mAtFloors.getCurrentFloor()){
                newState=State.STATE_SLOW_UP;
            //#transition 'T6.4'
            }else if(localDriveSpeed.speed()*1000==0 && allDoorClosed()==true && mDesiredFloor.getFloor() < mAtFloors.getCurrentFloor()){
                newState=State.STATE_SLOW_DOWN;
            //#transition 'T6.7'
            }else if(localDriveSpeed.speed()*1000==0 && mLevelDown.getValue()==true && mLevelUp.getValue()==false){
                newState=State.STATE_LEVEL_UP;
            //#transition 'T6.8'
            }else if(localDriveSpeed.speed()*1000==0 && mLevelUp.getValue()==true && mLevelDown.getValue()==false){
                newState=State.STATE_LEVEL_DOWN;
            }

            break;

            case STATE_SLOW_UP:
            //state actions for STATE_SLOW_UP
            localDrive.set(Speed.SLOW, Direction.UP);
            mDrive.set(Speed.SLOW,Direction.UP);
            mDriveSpeed.set((int) (localDriveSpeed.speed()*1000));
            
            //#transition 'T6.13'
            if(localDriveSpeed.speed()==SlowSpeed && mCarPositionIndicator.getValue()<mDesiredFloor.getFloor()){
                newState=State.STATE_FAST_UP;
            //#transition 'T6.2'
            }else if(mAtFloors.getCurrentFloor()==mDesiredFloor.getFloor()){
                newState=State.STATE_LEVEL_UP;
            //#transition 'T6.9'
            }else if(mEmergencyBrake.getValue()==true){
                newState=State.STATE_EMERGENCY;                
            }
            break;

            case STATE_SLOW_DOWN:
            //state actions for STATE_SLOW_DOWN
            localDrive.set(Speed.SLOW, Direction.DOWN);
            mDrive.set(Speed.SLOW,Direction.DOWN);
            mDriveSpeed.set((int) (localDriveSpeed.speed()*1000));            
            
            //#transition 'T6.15'
            if(localDriveSpeed.speed()==SlowSpeed && mCarPositionIndicator.getValue()>mDesiredFloor.getFloor()){
                newState=State.STATE_FAST_DOWN;
            //#transition 'T6.5'
            }else if(mAtFloors.getCurrentFloor()==mDesiredFloor.getFloor()){
                newState=State.STATE_LEVEL_DOWN;
            //#transition 'T6.10'
            }else if(mEmergencyBrake.getValue()==true){
                newState=State.STATE_EMERGENCY;
            }
            break;

            case STATE_LEVEL_UP:
            //state actions for STATE_LEVEL_UP
            localDrive.set(Speed.LEVEL, Direction.UP);
            mDrive.set(Speed.LEVEL, Direction.UP);
            mDriveSpeed.set((int) (localDriveSpeed.speed()*1000));
            //#transition 'T6.3'
            if( mLevelUp.getValue()==true ){
                newState=State.STATE_STOP;
            //#transition 'T6.11'
            }else if(mEmergencyBrake.getValue()==true){
                newState=State.STATE_EMERGENCY;
            }
            break;

            case STATE_LEVEL_DOWN:
            //state actions for STATE_LEVEL_DOWN
            localDrive.set(Speed.LEVEL, Direction.DOWN);
            mDrive.set(Speed.LEVEL, Direction.DOWN);
            mDriveSpeed.set((int) (localDriveSpeed.speed()*1000));

            //#transition 'T6.6'
            if(mLevelDown.getValue()==true){
                newState=State.STATE_STOP;
            //#transition 'T6.12'
            }else if(mEmergencyBrake.getValue()==true){
                newState=State.STATE_EMERGENCY;
            }
            break;

            case STATE_FAST_UP:
            localDrive.set(Speed.FAST, Direction.UP);
            mDrive.set(Speed.FAST, Direction.UP);
            mDriveSpeed.set((int) (localDriveSpeed.speed()*1000));
            
            //#transition 'T6.14'
            if(mCarPositionIndicator.getValue()==mDesiredFloor.getFloor()){
                newState=State.STATE_SLOW_UP;
            //#transition 'T6.18'
            }else if(mEmergencyBrake.getValue()==true){
                newState=State.STATE_EMERGENCY;
            }
            break;

            case STATE_FAST_DOWN:
            localDrive.set(Speed.FAST, Direction.DOWN);
            mDrive.set(Speed.FAST, Direction.DOWN);
            mDriveSpeed.set((int) (localDriveSpeed.speed()*1000));
                        
            //#transition 'T6.16'
            if(mCarPositionIndicator.getValue()==mDesiredFloor.getFloor()){
                newState=State.STATE_SLOW_DOWN;
            //#transition 'T6.17'
            }else if(mEmergencyBrake.getValue()==true ){
                newState=State.STATE_EMERGENCY;
            }
            break;

            case STATE_EMERGENCY:
            //state actions for STATE_EMERGENCY
            localDrive.set(Speed.STOP, Direction.STOP);
            mDrive.set(Speed.STOP, Direction.STOP);
            mDriveSpeed.set((int) (localDriveSpeed.speed()*1000));
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

    //check if all doors are closed, namely check if any door open.
    public boolean allDoorClosed(){
        if(mDoorClosedFront.getBothClosed()==true && mDoorClosedBack.getBothClosed()==true)
            return true;
        else 
            return false;
    }

    //check if the car is leveled
    public boolean isLevel(){
        if(mLevelUp.getValue()==true && mLevelDown.getValue()==true){
            return true;
        }else{
            return false;
        }
    }


}