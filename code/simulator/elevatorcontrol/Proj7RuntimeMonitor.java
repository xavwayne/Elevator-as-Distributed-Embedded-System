/**
 * 18649-Fall-2015
 * Group 3
 * Jiyu Shi(jiyus) ; Shuai Wang(shuaiwa1); Xiaoyu Wang(xiaoyuw); Xiao Guo(xiaog)
 */
package simulator.elevatorcontrol;



import jSimPack.SimTime;
import simulator.framework.DoorCommand;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.Harness;
import simulator.framework.RuntimeMonitor;
import simulator.framework.Side;
import simulator.payloads.CarWeightPayload.ReadableCarWeightPayload;
import simulator.payloads.DoorClosedPayload.ReadableDoorClosedPayload;
import simulator.payloads.DoorMotorPayload.ReadableDoorMotorPayload;
import simulator.payloads.DoorOpenPayload.ReadableDoorOpenPayload;
import simulator.payloads.DriveSpeedPayload.ReadableDriveSpeedPayload;
//what I implement from here starting with gvien sample

import simulator.payloads.DoorReversalPayload.ReadableDoorReversalPayload;
import simulator.payloads.CarCallPayload.ReadableCarCallPayload;
import simulator.payloads.HallCallPayload.ReadableHallCallPayload;
import simulator.elevatorcontrol.Utility;
import simulator.elevatorcontrol.MessageDictionary;
import simulator.payloads.AtFloorPayload.ReadableAtFloorPayload;
import simulator.framework.Direction;

public class Proj7RuntimeMonitor extends RuntimeMonitor {

    DoorStateMachine doorState = new DoorStateMachine();
    WeightStateMachine weightState = new WeightStateMachine();
    boolean hasMoved = false;
    boolean wasOverweight = false;
    int overWeightCount = 0;

    //input mAtFloor    
    //what I add from the sample monitor
    int wasteOpenCount = 0;
    boolean wasWeightChanged = false;
    Stopwatch doorRevesalTime = new Stopwatch();

    CarCallStateMachine carCallState = new CarCallStateMachine();
    HallCallStateMachine hallCallState = new HallCallStateMachine();

    protected int currentFloor = MessageDictionary.NONE;
    //protected int lastStoppedFloor = MessageDictionary.NONE;

    protected String[] summarize() {
        String[] arr = new String[3];
        arr[0] = "Overweight Count = " + overWeightCount ;
        arr[1] = "Wasted Opening Count =" + wasteOpenCount ;
        arr[2] = "Time Reversal Cost = " + doorRevesalTime.getAccumulatedTime() ;       
        return arr;
    }

    public void timerExpired(Object callbackData) {
        //do nothing
    }


    /**************************************************************************
     * high level event methods
     *
     * these are called by the logic in the message receiving methods and the
     * state machines
     **************************************************************************/
    /**
     * Called once when the door starts opening
     * @param hallway which door the event pertains to
     */
    private void doorOpening(Hallway hallway) {
        if (currentFloor != MessageDictionary.NONE){
            if (carCallState.state[currentFloor - 1][hallway.ordinal()] == CarCallState.UNPRESSED &&
                hallCallState.state[currentFloor - 1][hallway.ordinal()][Direction.UP.ordinal()] == HallCallState.UNPRESSED&&
                hallCallState.state[currentFloor -1][hallway.ordinal()][Direction.DOWN.ordinal()] == HallCallState.UNPRESSED){
                    wasteOpenCount++;
                    message("Wasted Open");    
            }
            carCallState.state[currentFloor - 1][hallway.ordinal()] = CarCallState.UNPRESSED;
            hallCallState.state[currentFloor - 1][hallway.ordinal()][Direction.UP.ordinal()] = HallCallState.UNPRESSED;
            hallCallState.state[currentFloor -1][hallway.ordinal()][Direction.DOWN.ordinal()] = HallCallState.UNPRESSED;         
        }
    }

    /**
     * Called once when the door starts closing
     * @param hallway which door the event pertains to
     */
    private void doorClosing(Hallway hallway) {
        //System.out.println(hallway.toString() + " Door Closing");
        /*
        if (!doorState.anyDoorMotorOpening(hallway)){
            if (!wasWeightChanged){
                wasteOpenCount++;
                message("Wasted Open");
            }else{
                wasWeightChanged = false;
               
            }            
        }   
        */
    }

    /**
     * Called once if the door starts opening after it started closing but before
     * it was fully closed.
     * @param hallway which door the event pertains to
     */
    private void doorReopening(Hallway hallway) {
        //System.out.println(hallway.toString() + " Door Reopening");
    }

    /**
     * Called once when the doors close completely
     * @param hallway which door the event pertains to
     */
    private void doorClosed(Hallway hallway) {
        //System.out.println(hallway.toString() + " Door Closed");
        //once all doors are closed, check to see if the car was overweight
        if (!doorState.anyDoorOpen()) {
            doorRevesalTime.stop();
            if (wasOverweight) {
                message("Overweight");
                overWeightCount++;
                wasOverweight = false;
            }
        }
    }

    /**
     * Called once when the doors are fully open
     * @param hallway which door the event pertains to
     */
    private void doorOpened(Hallway hallway) {
        //System.out.println(hallway.toString() + " Door Opened");
    }


    /**
     * Called when the car weight changes
     * @param hallway which door the event pertains to
     */
    private void weightChanged(int newWeight) {
        if (newWeight > Elevator.MaxCarCapacity) {
            wasOverweight = true;
        }
    }

    /**************************************************************************
     * low level message receiving methods
     * 
     * These mostly forward messages to the appropriate state machines
     **************************************************************************/
    @Override
    public void receive(ReadableDoorClosedPayload msg) {
        doorState.receive(msg);
    }

    @Override
    public void receive(ReadableDoorOpenPayload msg) {
        doorState.receive(msg);
    }

    @Override
    public void receive(ReadableDoorMotorPayload msg) {
        doorState.receive(msg);
    }

    @Override
    public void receive(ReadableCarWeightPayload msg) {
        weightState.receive(msg);
    }

    public void receive(ReadableDoorReversalPayload msg) {    
        doorState.receive(msg);
    }

    @Override
    public void receive(ReadableDriveSpeedPayload msg) {
        if (msg.speed() > 0) {
            hasMoved = true;
        }
    }

    public void receive(ReadableHallCallPayload msg) {
        if (msg.getFloor() !=  MessageDictionary.NONE){
            hallCallState.receive(msg);   
        }
        
    }

    public void receive(ReadableCarCallPayload msg) {
        if (msg.getFloor() !=  MessageDictionary.NONE){
            carCallState.receive(msg);            
        }
    }

        @Override
    public void receive(ReadableAtFloorPayload msg) {
        updateCurrentFloor(msg);
    }

    private static enum DoorState {
        CLOSED,
        OPENING,
        OPEN,
        CLOSING
    }

    private static enum CarCallState {
        PRESSED,
        UNPRESSED
    }

    private static enum HallCallState {
        PRESSED,
        UNPRESSED
    }

    private class CarCallStateMachine{
        CarCallState state[][] = new CarCallState[8][2];
        public CarCallStateMachine(){
            for (int i = 0; i < 8; i++){
                for (int j = 0; j < 2; j++){
                    state[i][j] = CarCallState.UNPRESSED;
                }
            }            
        }

        public void receive(ReadableCarCallPayload msg) {
            if (msg.isPressed()){
                state[msg.getFloor() - 1][msg.getHallway().ordinal()] = CarCallState.PRESSED;
                //message("Car call pressed updated " + msg.getFloor() );
            }
        }
    }

    private class HallCallStateMachine{
        HallCallState state[][][] = new HallCallState[8][2][2];

        public HallCallStateMachine(){
            for (int i = 0 ; i < 8; i++){
                for (int j = 0; j < 2; j++){
                    for (int k = 0; k < 2 ; k++){
                        state[i ][j][k] = HallCallState.UNPRESSED;
                    }
                }
            }
        }

        public void receive(ReadableHallCallPayload msg) {
            if (msg.pressed()){
                state[msg.getFloor() - 1][msg.getHallway().ordinal()][msg.getDirection().ordinal()] = HallCallState.PRESSED;
               // message("Hall call pressed updated " + msg.getFloor() );
            }
        }
    }   
    /**
     * Utility class to detect weight changes //was initialized at the start
     */
    private class WeightStateMachine {

        int oldWeight = 0;

        public void receive(ReadableCarWeightPayload msg) {
            if (oldWeight != msg.weight()) {
                weightChanged(msg.weight());   //call another method to put the value wasOverweighted to true.
                wasWeightChanged = true	;		// this means a valid open door instead of a waste		
            }
            oldWeight = msg.weight();
        }
    }

    /**
     * Utility class for keeping track of the door state.//was initialized at the start
     * 
     * Also provides external methods that can be queried to determine the
     * current door state.
     */
    private class DoorStateMachine {

        DoorState state[] = new DoorState[2];

        public DoorStateMachine() {
            state[Hallway.FRONT.ordinal()] = DoorState.CLOSED;
            state[Hallway.BACK.ordinal()] = DoorState.CLOSED;
        }

        public void receive(ReadableDoorClosedPayload msg) {
            updateState(msg.getHallway());
        }

        public void receive(ReadableDoorOpenPayload msg) {
            updateState(msg.getHallway());
        }

        public void receive(ReadableDoorMotorPayload msg) {
            updateState(msg.getHallway());
        }

        //add door reversal here
        public void receive(ReadableDoorReversalPayload msg) {          
            if(msg.isReversing()){
                message("Reversal start which shall not be triggered!");
                doorRevesalTime.start();
            }
        }

        //use car call and hall call to find
        public void receive(ReadableHallCallPayload msg) {
            updateState(msg.getHallway());
        }

        public void receive(ReadableCarCallPayload msg) {
            updateState(msg.getHallway());
        }


        private void updateState(Hallway h) {
            DoorState previousState = state[h.ordinal()];
            DoorState newState = previousState;

            if (allDoorsClosed(h) && allDoorMotorsStopped(h)) {
                newState = DoorState.CLOSED;
            } else if (allDoorsCompletelyOpen(h) && allDoorMotorsStopped(h)) {
                newState = DoorState.OPEN;
                //} else if (anyDoorMotorClosing(h) && anyDoorOpen(h)) {
            } else if (anyDoorMotorClosing(h)) {
                newState = DoorState.CLOSING;
            } else if (anyDoorMotorOpening(h)) {
                newState = DoorState.OPENING;
            }

            if (newState != previousState) {
                switch (newState) {
                    case CLOSED:
                       // message("Door Closed");
                        doorClosed(h);
                        break;
                    case OPEN:
                       // message("Door Opened");
                        doorOpened(h);
                        break;
                    case OPENING:
                       // message("Door OPENING");
                        if (previousState == DoorState.CLOSING) {
                            doorReopening(h);
                        } else {
                            doorOpening(h);
                        }
                        break;
                    case CLOSING:
                       //message("Door Closing");
                        doorClosing(h);
                        break;

                }
            }

            //set the newState
            state[h.ordinal()] = newState;
        }

        //door utility methods
        public boolean allDoorsCompletelyOpen(Hallway h) {
            return doorOpeneds[h.ordinal()][Side.LEFT.ordinal()].isOpen()
                    && doorOpeneds[h.ordinal()][Side.RIGHT.ordinal()].isOpen();
        }

        public boolean anyDoorOpen() {
            return anyDoorOpen(Hallway.FRONT) || anyDoorOpen(Hallway.BACK);

        }

        public boolean anyDoorOpen(Hallway h) {
            return !doorCloseds[h.ordinal()][Side.LEFT.ordinal()].isClosed()
                    || !doorCloseds[h.ordinal()][Side.RIGHT.ordinal()].isClosed();
        }

        public boolean allDoorsClosed(Hallway h) {
            return (doorCloseds[h.ordinal()][Side.LEFT.ordinal()].isClosed()
                    && doorCloseds[h.ordinal()][Side.RIGHT.ordinal()].isClosed());
        }

        public boolean allDoorMotorsStopped(Hallway h) {
            return doorMotors[h.ordinal()][Side.LEFT.ordinal()].command() == DoorCommand.STOP && doorMotors[h.ordinal()][Side.RIGHT.ordinal()].command() == DoorCommand.STOP;
        }

        public boolean anyDoorMotorOpening(Hallway h) {
            return doorMotors[h.ordinal()][Side.LEFT.ordinal()].command() == DoorCommand.OPEN || doorMotors[h.ordinal()][Side.RIGHT.ordinal()].command() == DoorCommand.OPEN;
        }

        //since our door has no close command but nudge , I change this to nudge in order to find closing state to count the numbers.
        public boolean anyDoorMotorClosing(Hallway h) {
            return doorMotors[h.ordinal()][Side.LEFT.ordinal()].command() == DoorCommand.NUDGE || doorMotors[h.ordinal()][Side.RIGHT.ordinal()].command() == DoorCommand.NUDGE;
        }
    }

    //used to get current floor
    private void updateCurrentFloor(ReadableAtFloorPayload lastAtFloor) {
        if (lastAtFloor.getFloor() == currentFloor) {
            //the atFloor message is for the currentfloor, so check both sides to see if they a
            if (!atFloors[lastAtFloor.getFloor()-1][Hallway.BACK.ordinal()].value() && !atFloors[lastAtFloor.getFloor()-1][Hallway.FRONT.ordinal()].value()) {
                //both sides are false, so set to NONE
                currentFloor = MessageDictionary.NONE;
            }
            //otherwise at least one side is true, so leave the current floor as is
        } else {
            if (lastAtFloor.value()) {
                currentFloor = lastAtFloor.getFloor();
                //message("floor updated");
            }
        }
    }

    /**
     * Keep track of time and decide whether to or not to include the last interval
     */
    private class ConditionalStopwatch {

        private boolean isRunning = false;
        private SimTime startTime = null;
        private SimTime accumulatedTime = SimTime.ZERO;

        /**
         * Call to start the stopwatch
         */
        public void start() {
            if (!isRunning) {
                startTime = Harness.getTime();
                isRunning = true;
            }
        }

        /**
         * stop the stopwatch and add the last interval to the accumulated total
         */
        public void commit() {
            if (isRunning) {
                SimTime offset = SimTime.subtract(Harness.getTime(), startTime);
                accumulatedTime = SimTime.add(accumulatedTime, offset);
                startTime = null;
                isRunning = false;
            }
        }

        /**
         * stop the stopwatch and discard the last interval
         */
        public void reset() {
            if (isRunning) {
                startTime = null;
                isRunning = false;
            }
        }

        public SimTime getAccumulatedTime() {
            return accumulatedTime;
        }

        public boolean isRunning() {
            return isRunning;
        }
    }

    /**
     * Keep track of the accumulated time for an event
     */
    private class Stopwatch {

        private boolean isRunning = false;
        private SimTime startTime = null;
        private SimTime accumulatedTime = SimTime.ZERO;

        /**
         * Start the stopwatch
         */
        public void start() {
            if (!isRunning) {
                startTime = Harness.getTime();
                isRunning = true;
            }
        }

        /**
         * Stop the stopwatch and add the interval to the accumulated total
         */
        public void stop() {
            if (isRunning) {
                SimTime offset = SimTime.subtract(Harness.getTime(), startTime);
                accumulatedTime = SimTime.add(accumulatedTime, offset);
                startTime = null;
                isRunning = false;
            }
        }

        public SimTime getAccumulatedTime() {
            return accumulatedTime;
        }

        public boolean isRunning() {
            return isRunning;
        }
    }

    /**
     * Utility class to implement an event detector
     */
    private abstract class EventDetector {

        boolean previousState;

        public EventDetector(boolean initialValue) {
            previousState = initialValue;
        }

        public void updateState(boolean currentState) {
            if (currentState != previousState) {
                previousState = currentState;
                eventOccurred(currentState);
            }
        }

        /**
         * subclasses should overload this to make something happen when the event
         * occurs.
         * @param newState
         */
        public abstract void eventOccurred(boolean newState);
    }
}