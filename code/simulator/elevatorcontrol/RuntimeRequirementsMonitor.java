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

import simulator.payloads.CarLanternPayload.ReadableCarLanternPayload;
import simulator.payloads.DrivePayload.ReadableDrivePayload;

import simulator.payloads.CarLightPayload.ReadableCarLightPayload;
import simulator.payloads.HallLightPayload.ReadableHallLightPayload;
import simulator.framework.Speed;
public class RuntimeRequirementsMonitor extends RuntimeMonitor {

    DoorStateMachine doorState = new DoorStateMachine();
    WeightStateMachine weightState = new WeightStateMachine();
    boolean hasMoved = false;
    boolean wasOverweight = false;
    int overWeightCount = 0;
    PrevCarLanternState prevCarLanternState = PrevCarLanternState.OFF;
    //input mAtFloor    
    //what I add from the sample monitor in Project 7
    int wasteOpenCount = 0;
    //boolean wasWeightChanged = false;
    Stopwatch doorRevesalTime = new Stopwatch();

    CarLightStateMachine carCallState = new CarLightStateMachine();
    HallLightStateMachine hallCallState = new HallLightStateMachine();
    //New added in Projet 8
    DriveSpeedStateMachine driveSpeedState = new DriveSpeedStateMachine();
    CarLanternStateMachine carLanternState = new CarLanternStateMachine();
    DriveStateMachine  driveMachine = new DriveStateMachine();
    protected int currentFloor = MessageDictionary.NONE;
    boolean beyond = false;
    boolean under = false; 
    
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

                // if (doorState.reversal[hallway.ordinal()]){
                //     message("not waste open because reversal triggered"); 
                // }else if (wasOverweight){
                //      message("not waste open because overWeight");       
                // }else{
                    wasteOpenCount++;
                    String h = "";
                    if (hallway.ordinal() == 0) {
                         h = "FRONT";
                    }else{
                         h = "BACK";
                    }
                    warning("R-T.7 Violated: The Car open Doors at Floor "+ currentFloor
                             +" "+ h +" Hallway"+" for which there is no pending call.");    
               // }
  
            }        
        }
                //CHECK r-t8.1

        if (currentFloor != MessageDictionary.NONE){
            for (Hallway h : Hallway.replicationValues){
                for (int i = currentFloor + 1 ; i < 9 ; i ++){
                    if (carCallState.state[i - 1][h.ordinal()] == CarCallState.PRESSED ||
                        hallCallState.state[i - 1][h.ordinal()][Direction.UP.ordinal()] == HallCallState.PRESSED||
                        hallCallState.state[i -1][h.ordinal()][Direction.DOWN.ordinal()] == HallCallState.PRESSED){
                    beyond = true;
                    break;
                    }                
                }
                if (hallCallState.state[currentFloor - 1][h.ordinal()][Direction.UP.ordinal()] == HallCallState.PRESSED)
                    beyond = true;

                for (int j = currentFloor - 1; j > 0; j --){
                    if (carCallState.state[j - 1][h.ordinal()] == CarCallState.PRESSED ||
                        hallCallState.state[j - 1][h.ordinal()][Direction.UP.ordinal()] == HallCallState.PRESSED||
                        hallCallState.state[j -1][h.ordinal()][Direction.DOWN.ordinal()] == HallCallState.PRESSED){
                    under = true;
                    break;
                    }       
                }
                if (hallCallState.state[currentFloor - 1][h.ordinal()][Direction.DOWN.ordinal()] == HallCallState.PRESSED)
                    under = true;                  
            }
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
        // if (currentFloor != MessageDictionary.NONE){
        //     for (Hallway h : Hallway.replicationValues){
        //         for (int i = currentFloor + 1 ; i < 9 ; i ++){
        //             if (carCallState.state[i - 1][h.ordinal()] == CarCallState.PRESSED ||
        //                 hallCallState.state[i - 1][h.ordinal()][Direction.UP.ordinal()] == HallCallState.PRESSED||
        //                 hallCallState.state[i -1][h.ordinal()][Direction.DOWN.ordinal()] == HallCallState.PRESSED){
        //             beyond = true;
        //             break;
        //             }                
        //         }
        //         if (hallCallState.state[currentFloor - 1][h.ordinal()][Direction.UP.ordinal()] == HallCallState.PRESSED)
        //             beyond = true;

        //         for (int j = currentFloor - 1; j > 0; j --){
        //             if (carCallState.state[j - 1][h.ordinal()] == CarCallState.PRESSED ||
        //                 hallCallState.state[j - 1][h.ordinal()][Direction.UP.ordinal()] == HallCallState.PRESSED||
        //                 hallCallState.state[j -1][h.ordinal()][Direction.DOWN.ordinal()] == HallCallState.PRESSED){
        //             under = true;
        //             break;
        //             }   
        //         if (hallCallState.state[currentFloor - 1][h.ordinal()][Direction.DOWN.ordinal()] == HallCallState.PRESSED)
        //             under = true;      
        //         }                
        //     }
        // }
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
        doorState.reversalCount[hallway.ordinal()] = 0;
        //driveSpeedState.countStop = 0;

    }

    /**
     * Called once when the doors are fully open
     * @param hallway which door the event pertains to
     */
    private void doorOpened(Hallway hallway) {


        //System.out.println("beyond = "+ beyond);
        if (carCallState.state[7][Hallway.FRONT.ordinal()] == CarCallState.PRESSED ){
           // System.out.println("8 FRONT CALLED");
        }else{
            //System.out.println("8 FRONT NOT");
        }
        if (beyond && !under){ //pending calls beyond
            if (carLanternState.state[Direction.UP.ordinal()] == CarLanternState.OFF){  
                warning("R-T.8.1 Violated: There is only car call beyond the floor but up lantern light is not ON at floor "
                    +currentFloor);             
            }
        }else if(!beyond && under){//pending calls under
            if (carLanternState.state[Direction.DOWN.ordinal()] == CarLanternState.OFF ){
                warning("R-T.8.1 Violated: There is only car call under the floor but down lantern light is not ON at floor "
                    +currentFloor);             
            }
        }else if(beyond && under){ //pending calls everywhere
            if (carLanternState.state[Direction.UP.ordinal()] == CarLanternState.OFF &&
                carLanternState.state[Direction.DOWN.ordinal()] == CarLanternState.OFF){
                warning("R-T.8.1 Violated: There is car call both beyond and under the floor but no lantern light is ON at floor "
                    +currentFloor);  
            }
        }else{                     //no pending calls

        }

        beyond = false;
        under = false;
        // reversal[Hallway.BACK.ordinal()] = false;
        // reversal[Hallway.FRONT.ordinal()] = false;
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

    private void carMove(){

    }


    private void carStop(){
        if (carCallState.state[currentFloor - 1][0] == CarCallState.UNPRESSED &&
            hallCallState.state[currentFloor - 1][0][Direction.UP.ordinal()] == HallCallState.UNPRESSED&&
            hallCallState.state[currentFloor -1][0][Direction.DOWN.ordinal()] == HallCallState.UNPRESSED &&
            carCallState.state[currentFloor - 1][1] == CarCallState.UNPRESSED &&
            hallCallState.state[currentFloor - 1][1][Direction.UP.ordinal()] == HallCallState.UNPRESSED&&
            hallCallState.state[currentFloor -1][1][Direction.DOWN.ordinal()] == HallCallState.UNPRESSED&&
            driveSpeedState.countStop == 0
            ){

                warning("R-T.6 Violated: The Car Stops at Floor "+ currentFloor+
                             " for which there is no pending call.");   
            }
            driveSpeedState.countStop ++;
            //this varible is used for record how many stops happened in one landing,
            //because passenegers gets in and out will let the drivespped transit from stop to level and 
            //then back to stop
            //but this time car call has been canceled and we don't to consider this as a landing.
        if (!driveMachine.fastSpeedCommand){
                warning("R-T.9 Violated: The Drive has not been commanded to fast speed to the maximum degree"+ 
                   " when arriving at floor " + currentFloor + " ."); 
        }

    }
    //called only once if state change from off to on
    private void lanternOn(Direction direction){
        if (direction == Direction.UP){
            prevCarLanternState = PrevCarLanternState.UP;
        }else if(direction == Direction.DOWN){
            prevCarLanternState = PrevCarLanternState.DOWN;
        }

        if (doorState.state[Hallway.FRONT.ordinal()] != DoorState.OPENING &&  
            doorState.state[Hallway.BACK.ordinal()] != DoorState.OPENING){
            warning("R-T.8.2 Violated: Car Lantern turn on at floor "
                    +currentFloor+" While door is not open"); 
        }
    }
    //called only once if state change from on to off
    private void lanternOff(Direction direction){
        if (doorState.state[Hallway.FRONT.ordinal()]!= DoorState.CLOSED && 
            doorState.state[Hallway.BACK.ordinal()]!= DoorState.CLOSED){
            warning("R-T.8.2 Violated: Car Lantern turn off  at floor "
                    +currentFloor+" While door is not fully closed"); 
        }
    }
    //called once from stop to up
    private void driveCommandUp(){

        boolean downCall = false;
        if (currentFloor != MessageDictionary.NONE){
            for (Hallway h : Hallway.replicationValues){
                for (int j = currentFloor - 1; j > 0; j --){
                    if (carCallState.state[j - 1][h.ordinal()] == CarCallState.PRESSED ||
                        hallCallState.state[j - 1][h.ordinal()][Direction.UP.ordinal()] == HallCallState.PRESSED||
                        hallCallState.state[j -1][h.ordinal()][Direction.DOWN.ordinal()] == HallCallState.PRESSED){
                    downCall = true;
                    break;
                    }   
                }                
            }
        }

        if (prevCarLanternState == PrevCarLanternState.DOWN && downCall){
            warning("R-T.8.3 Violated: Down direction of the car lanterns is lit,"
             + "but the car moves up ");
        }
        prevCarLanternState = PrevCarLanternState.OFF;
        driveSpeedState.countStop = 0;
        driveMachine.fastSpeedCommand = false;
    }
    //called once from stop to down
    private void driveCommandDown(){
        boolean upCall = false;

        if (currentFloor != MessageDictionary.NONE){
            for (Hallway h : Hallway.replicationValues){
                for (int i = currentFloor + 1 ; i < 8 ; i ++){
                    if (carCallState.state[i - 1][h.ordinal()] == CarCallState.PRESSED ||
                        hallCallState.state[i - 1][h.ordinal()][Direction.UP.ordinal()] == HallCallState.PRESSED||
                        hallCallState.state[i -1][h.ordinal()][Direction.DOWN.ordinal()] == HallCallState.PRESSED){
                    upCall = true;
                    break;
                    }                           
                }
            }
        }

        if (prevCarLanternState == PrevCarLanternState.UP && upCall){
            warning("R-T.8.3 Violated: UP direction of the car lanterns is lit, and there is "
             + "but the car moves down ");
        }
        prevCarLanternState = PrevCarLanternState.OFF;
        driveSpeedState.countStop = 0;
        driveMachine.fastSpeedCommand = false;
    }
    //called once from moving to stop
    private void driveCommandStop(){

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
        //check for RT.10
        if (msg.command() == DoorCommand.NUDGE){
            if (doorState.reversalCount[msg.getHallway().ordinal()] == 0){
                warning("R-T.10 Violated: The Car nudge Doors at Floor "+ currentFloor
                             +" "+ msg.getHallway() +" Hallway"+" for which there is no reversal happense before.");
            }
            //message("NUDGE command");
        }
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
        driveSpeedState.receive(msg);
    }

    public void receive(ReadableHallLightPayload msg) {
        if (msg.getFloor() !=  MessageDictionary.NONE){
            hallCallState.receive(msg);   
        }   
    }

    public void receive(ReadableCarLightPayload msg) {
        if (msg.getFloor() !=  MessageDictionary.NONE){
            carCallState.receive(msg);            
        }
    }

    public void receive(ReadableCarCallPayload msg) {
        if (msg.getFloor() !=  MessageDictionary.NONE){
            carCallState.receiveCarCall(msg);            
        }
    }

    public void receive(ReadableHallCallPayload msg) {
        if (msg.getFloor() !=  MessageDictionary.NONE){
            hallCallState.receiveHallCall(msg);            
        }
    }

        
    public void receive(ReadableAtFloorPayload msg) {
        updateCurrentFloor(msg);

    }

    public void receive(ReadableCarLanternPayload msg) {
        carLanternState.receive(msg);
    }

    public void receive(ReadableDrivePayload msg) {
        driveMachine.receive(msg);
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

    private static enum DriveSpeedState {
        STOP,
        MOVING
    }

    private static enum CarLanternState {
        ON,
        OFF
    }

    private static enum PrevCarLanternState {
        UP,
        DOWN,
        OFF
    }

    private static enum DriveDirectionState {
        UP,
        DOWN,
        STOP
    }

    private class DriveStateMachine {
        DriveDirectionState state = DriveDirectionState.STOP;
        boolean fastSpeedCommand = false;
         public void receive(ReadableDrivePayload msg) {
            DriveDirectionState previousState = state;
            DriveDirectionState newState = previousState;
            Direction direction = msg.direction();

            //Check RT-9 in this part.

            if (!fastSpeedCommand && msg.speed() == Speed.FAST){
                fastSpeedCommand = true;
            }

            //this is used for test RT-8 since passenger gets in and out will make the car down and up
            //and that will be count to waste landing, in order to avoid this, I design this if sentence
            // ingonre the level case.
            if (msg.speed() != Speed.LEVEL){
                switch (direction.ordinal()){
                    case 0:                       
                        newState = DriveDirectionState.UP;
                        break;
                    case 1:
                        
                        newState = DriveDirectionState.DOWN;
                        break;
                    case 2:

                        newState = DriveDirectionState.STOP;
                        break;
                }

                if (newState != previousState){
                    switch(newState.ordinal()){   
                        case 0: 
                            //message("Car moves UP~");
                            driveCommandUp();
                            break; 
                        case 1:
                        //DOWN
                            //message("Car moves DOWN~");
                            driveCommandDown(); 
                            break;
                        case 2:
                            //message("Car moves STOP~");
                            driveCommandStop(); 
                            break;
                    }
                }
                state = newState;                
            }

         }
    }


    private class CarLanternStateMachine {
        CarLanternState state[] = new CarLanternState[2];
        public CarLanternStateMachine(){
            state[Direction.UP.ordinal()] = CarLanternState.OFF;   
            state[Direction.DOWN.ordinal()] =  CarLanternState.OFF; //initial state
        }

        public void receive(ReadableCarLanternPayload msg) {
            CarLanternState previousState = state[msg.getDirection().ordinal()];
            CarLanternState newState = previousState;
            if (msg.lighted()){
                newState = CarLanternState.ON;
            }else{
                newState = CarLanternState.OFF;
            }
            if (newState != previousState){
                switch(newState.ordinal()){   
                    case 0: 
                        lanternOn(msg.getDirection());
                        break; 
                    case 1:
                    //DOWN 
                        lanternOff(msg.getDirection());
                        break;

                }
            }
            state[msg.getDirection().ordinal()] = newState;
        }

    }

    private class DriveSpeedStateMachine {
        DriveSpeedState state = DriveSpeedState.STOP;
        int countStop ;
        public DriveSpeedStateMachine(){
            state = DriveSpeedState.STOP;   //initial state
            countStop = 0;
        }


        public void receive(ReadableDriveSpeedPayload msg) {
            DriveSpeedState previousState = state;
            DriveSpeedState newState = previousState;

            switch(msg.direction().ordinal()){   
                    case 2: 
                    //STOP
                        newState = DriveSpeedState.STOP;
                        break; 
                    case 1:
                    //DOWN
                        newState = DriveSpeedState.MOVING;
                        break;
                    case 0: 
                    //UP
                        newState = DriveSpeedState.MOVING;
                        break;
            }
            if (newState != previousState){
                    switch(newState){   
                        case STOP: 
                            carStop();
                            break; 
                        case MOVING:
                        //DOWN
                            carMove();
                            break;

                    }
                }
            state = newState;                
        }
    }
    private class CarLightStateMachine{
        CarCallState state[][] = new CarCallState[8][2];
        public CarLightStateMachine(){
            for (int i = 0; i < 8; i++){
                for (int j = 0; j < 2; j++){
                    state[i][j] = CarCallState.UNPRESSED;
                }
            }            
        }

        public void receive(ReadableCarLightPayload msg) {
            if (msg.lighted()){
                state[msg.getFloor() - 1][msg.getHallway().ordinal()] = CarCallState.PRESSED;
                //message("Car call pressed updated " + msg.getFloor() );
            }else{
                state[msg.getFloor() - 1][msg.getHallway().ordinal()] = CarCallState.UNPRESSED;
            }
        }

        public void receiveCarCall(ReadableCarCallPayload msg) {
            if (msg.pressed()){
                state[msg.getFloor() - 1][msg.getHallway().ordinal()] = CarCallState.PRESSED;
                //message("Car call pressed updated " + msg.getFloor() );
            }
        }

    }

    private class HallLightStateMachine{
        HallCallState state[][][] = new HallCallState[8][2][2];
        public HallLightStateMachine(){
            for (int i = 0 ; i < 8; i++){
                for (int j = 0; j < 2; j++){
                    for (int k = 0; k < 2 ; k++){
                        state[i ][j][k] = HallCallState.UNPRESSED;
                    }
                }
            }          
        }

        public void receive(ReadableHallLightPayload msg) {
            if (msg.lighted()){
                state[msg.getFloor() - 1][msg.getHallway().ordinal()][msg.getDirection().ordinal()] = HallCallState.PRESSED;
               // message("Hall call pressed updated " + msg.getFloor() );
            }else{
                state[msg.getFloor() - 1][msg.getHallway().ordinal()][msg.getDirection().ordinal()] = HallCallState.UNPRESSED;
            }
        }

        public void receiveHallCall(ReadableHallCallPayload msg) {
            if (msg.pressed()){
                state[msg.getFloor() - 1][msg.getHallway().ordinal()][msg.getDirection().ordinal()] = HallCallState.PRESSED;
               // message("Hall call pressed updated " + msg.getFloor() );
            }
        }
    }

    // private class CarCallStateMachine{
    //     CarCallState state[][] = new CarCallState[8][2];
    //     public CarCallStateMachine(){
    //         for (int i = 0; i < 8; i++){
    //             for (int j = 0; j < 2; j++){
    //                 state[i][j] = CarCallState.UNPRESSED;
    //             }
    //         }            
    //     }

    //     public void receive(ReadableCarCallPayload msg) {
    //         if (msg.pressed()){
    //             state[msg.getFloor() - 1][msg.getHallway().ordinal()] = CarCallState.PRESSED;
    //             //message("Car call pressed updated " + msg.getFloor() );
    //         }
    //     }
    // }

    // private class HallCallStateMachine{
    //     HallCallState state[][][] = new HallCallState[8][2][2];

    //     public HallCallStateMachine(){
    //         for (int i = 0 ; i < 8; i++){
    //             for (int j = 0; j < 2; j++){
    //                 for (int k = 0; k < 2 ; k++){
    //                     state[i ][j][k] = HallCallState.UNPRESSED;
    //                 }
    //             }
    //         }
    //     }

    //     public void receive(ReadableHallCallPayload msg) {
    //         if (msg.pressed()){
    //             state[msg.getFloor() - 1][msg.getHallway().ordinal()][msg.getDirection().ordinal()] = HallCallState.PRESSED;
    //            // message("Hall call pressed updated " + msg.getFloor() );
    //         }
    //     }
    // } 

    /**
     * Utility class to detect weight changes //was initialized at the start
     */
    private class WeightStateMachine {

        int oldWeight = 0;

        public void receive(ReadableCarWeightPayload msg) {
            if (oldWeight != msg.weight()) {
                weightChanged(msg.weight());   //call another method to put the value wasOverweighted to true.
               // wasWeightChanged = true	;		// this means a valid open door instead of a waste		
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
        boolean reversal[] = new boolean[2];
        int reversalCount[] = new int[2];

        public DoorStateMachine() {
            state[Hallway.FRONT.ordinal()] = DoorState.CLOSED;
            state[Hallway.BACK.ordinal()] = DoorState.CLOSED;
            reversal[Hallway.FRONT.ordinal()] = false;
            reversal[Hallway.BACK.ordinal()] = false;
            reversalCount[Hallway.FRONT.ordinal()] = 0;
            reversalCount[Hallway.BACK.ordinal()] = 0;
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
            switch(msg.getHallway()){
                case FRONT:
                    if(msg.isReversing()){
                        //message("Front Reversal triggered!");
                        doorRevesalTime.start();
                        reversal[Hallway.FRONT.ordinal()] = true;
                        reversalCount[Hallway.FRONT.ordinal()]++;
                    }else{
                  
                        reversal[Hallway.FRONT.ordinal()] = false;
                    }
                    break;
                case BACK:
                    if(msg.isReversing()){
                        doorRevesalTime.start();
                        reversal[Hallway.BACK.ordinal()] = true;
                        reversalCount[Hallway.BACK.ordinal()] ++;
                    }else{       
                        reversal[Hallway.BACK.ordinal()] = false;
                    }
                    break;
            }          

        }

        //use car call and hall call to find
        // public void receive(ReadableHallCallPayload msg) {
        //     updateState(msg.getHallway());
        // }

        // public void receive(ReadableCarCallPayload msg) {
        //     updateState(msg.getHallway());
        // }


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
                        if (previousState == DoorState.CLOSING ) {
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
            return doorMotors[h.ordinal()][Side.LEFT.ordinal()].command() == DoorCommand.CLOSE 
                || doorMotors[h.ordinal()][Side.RIGHT.ordinal()].command() == DoorCommand.CLOSE
                                ||doorMotors[h.ordinal()][Side.LEFT.ordinal()].command() == DoorCommand.NUDGE 
                                ||doorMotors[h.ordinal()][Side.RIGHT.ordinal()].command() == DoorCommand.NUDGE;
        }

    }

    //used to get current floor
    private void updateCurrentFloor(ReadableAtFloorPayload lastAtFloor) {
        if (lastAtFloor.getFloor() == currentFloor) {
            if (!atFloors[lastAtFloor.getFloor()-1][Hallway.BACK.ordinal()].value() && !atFloors[lastAtFloor.getFloor()-1][Hallway.FRONT.ordinal()].value()) {
                //both sides are false, so set to NONE
                currentFloor = MessageDictionary.NONE;
            }
            //otherwise at least one side is true, so leave the current floor as is
        } else {
            if (lastAtFloor.value()) {
                currentFloor = lastAtFloor.getFloor();
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