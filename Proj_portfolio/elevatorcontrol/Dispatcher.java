/**
 * 18649-Fall-2015
 * Group 3
 * Jiyu Shi(jiyus) ; Shuai Wang(shuaiwa1); Xiaoyu Wang(xiaoyuw); Xiao Guo(xiaog)
 * @author Xiaoyu Wang and Jiyu Shi
 */
package simulator.elevatorcontrol;


import jSimPack.SimTime;
import simulator.elevatorcontrol.DesiredFloorCanPayloadTranslator;
import simulator.elevatorcontrol.Dispatcher;
import simulator.elevatorcontrol.Utility;
import simulator.framework.Controller;
import simulator.framework.Direction;

import simulator.framework.ReplicationComputer;
import simulator.framework.TimeSensitive;
import simulator.framework.Timer;
import simulator.payloads.CANNetwork;
import simulator.elevatorcontrol.NewIntegerCanPayloadTranslator;

import  simulator.framework.Hallway;

import simulator.elevatorcontrol.MessageDictionary;
//those three parts are important to receive all the message !!!

import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import java.util.HashMap;
import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.Iterator;
import java.util.Map;


public class Dispatcher extends Controller implements TimeSensitive {
    static final SimTime DWELL_TIME = new SimTime(3000, SimTime.SimTimeUnit.MILLISECOND);
    static final int totalStop = 4;
    //static final SimTime INTERVAL = new SimTime(100, SimTime.SimTimeUnit.MILLISECOND);

    //private SimTime stopTime;

    private SimTime period;
    private int numFloors;
    private int currentFloor;
    private boolean bothFrontDoorClosed;
    private boolean bothBackDoorClosed;
    private int target;                  //target floor, mdesiredfloor shall be set to this     
    private Hallway targetHallway;             //current hallway
    private Direction targetDirection;  //DesiredDirection always STOP


    private Utility.AtFloorArray atFloorArray;         //input mAtFloor
    private Utility.DoorClosedArray[] doorClosedArray; //input mDoorClosed


    private WriteableCanMailbox networkDesiredDwell;   //used for mDesiredDwell 
    private WriteableCanMailbox networkDesiredFloor;   //used for mDesiredFloor

    private NewIntegerCanPayloadTranslator[] mDesiredDwell;    //output 
    private DesiredFloorCanPayloadTranslator mDesiredFloor; //output

    //added for the fast elevator design
    private ReadableCanMailbox networkCarPositionIndicator;  
    private NewIntegerCanPayloadTranslator mCarPositionIndicator;    //output 
    
    private Utility.CarCallArray carCallArray;
    private Utility.HallCallArray hallCallArray;   
    private HashMap<Integer,Hallway> carCallMessage;
    private HashMap<Integer,Hallway> hallCallUpMessage;
    private HashMap<Integer,Hallway> hallCallDownMessage;
    private int landingFloor;
    private Direction tendency;
    private int countStop ;

    //desendind order comparator for queue 
    private static class PQsort implements Comparator<Integer> {
        public int compare(Integer one, Integer two) {
            return two - one;
        }
    }
    private PQsort pqs = new PQsort();

    private enum State {
        STATE_IDLE,
        STATE_SERVE_BEFORE_ARRIVE,
        STATE_SERVE_AFTER_ARRIVE,
        STATE_COMPUTE,
        STATE_UNSAFE,
        STATE_STOP                                
    }


    private State state = State.STATE_IDLE;  

    public Dispatcher(int numFloors, SimTime period, boolean verbose) {
        
        super("Dispatcher", verbose);
        this.period = period;
        this.numFloors = numFloors;
        this.currentFloor = MessageDictionary.NONE;
        this.target = 1;
        this.targetHallway = Hallway.NONE;
        this.targetDirection = Direction.STOP;
        this.atFloorArray = new Utility.AtFloorArray(this.canInterface);
        this.doorClosedArray = new Utility.DoorClosedArray[2];    //front and back side ,totaly 4 doors.
        this.mDesiredDwell = new NewIntegerCanPayloadTranslator[2];
        
        //add for fast elevator
        this.carCallArray = new Utility.CarCallArray(this.canInterface);
        this.hallCallArray = new Utility.HallCallArray(this.canInterface);
        this.tendency = Direction.UP;
        this.countStop = 0;
        // back and front ,set the dwell
        for (Hallway hallway : Hallway.replicationValues) {
            int hallwayNum = ReplicationComputer.computeReplicationId(hallway);
            this.doorClosedArray[hallwayNum] = new Utility.DoorClosedArray(hallway, this.canInterface);

            networkDesiredDwell = CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_DWELL_BASE_CAN_ID + hallwayNum); //output mDesiredDwell

            //set dwell to each door
            mDesiredDwell[hallwayNum] = new NewIntegerCanPayloadTranslator(networkDesiredDwell);
            mDesiredDwell[hallwayNum].setValue((int)DWELL_TIME.getTruncMilliseconds());   
            canInterface.sendTimeTriggered(networkDesiredDwell, period);
        }


        networkDesiredFloor = CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);

        mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
        mDesiredFloor.setFloor(1);
        mDesiredFloor.setDirection(targetDirection);
        mDesiredFloor.setHallway(targetHallway);

        canInterface.sendTimeTriggered(networkDesiredFloor,period);
        timer.start(period);

        //added for the fast elevator in the constructor
        //a new input added to dispatcher CarPositionIndicator.
        networkCarPositionIndicator = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_POSITION_CAN_ID);
        mCarPositionIndicator = new NewIntegerCanPayloadTranslator(networkCarPositionIndicator);
        canInterface.registerTimeTriggered(networkCarPositionIndicator);

    }
    public void timerExpired(Object object) {

        bothFrontDoorClosed = doorClosedArray[ReplicationComputer.computeReplicationId(Hallway.FRONT)].getBothClosed();
        bothBackDoorClosed = doorClosedArray[ReplicationComputer.computeReplicationId(Hallway.BACK)].getBothClosed(); 
        currentFloor =  mCarPositionIndicator.getValue();
        landingFloor = atFloorArray.getCurrentFloor();
        //add for fast speed elevator
        carCallMessage = carCallArray.getPressedCarCall();
        hallCallUpMessage = hallCallArray.getPressedUpHallCall();
        hallCallDownMessage = hallCallArray.getPressedDownHallCall();
        DesiredFloorObject desiredFloor;

        //System.out.println("current floor = " + currentFloor);
        log("currentFloor: ",currentFloor);
        State newState = state;
        switch (state) {
            case STATE_IDLE:
                 target = currentFloor;
                 mDesiredFloor.setFloor(target);
                 mDesiredFloor.setHallway(Hallway.NONE);
                 mDesiredFloor.setDirection(Direction.STOP);

                 for (Hallway hallway : Hallway.replicationValues) {
                    int hallwayNum = ReplicationComputer.computeReplicationId(hallway);
                    mDesiredDwell[hallwayNum].set((int)DWELL_TIME.getTruncMilliseconds()); 
                 }
                 //#transition 'T11.1'
                 if(!carCallMessage.isEmpty() || !hallCallUpMessage.isEmpty() 
                            || !hallCallDownMessage.isEmpty()) { //&& mAtFloor.getValue() == false
                    newState = State.STATE_COMPUTE;
                 }
                 break;
            case STATE_UNSAFE:
                mDesiredFloor.setFloor(1);
                mDesiredFloor.setHallway(Hallway.NONE);
                mDesiredFloor.setDirection(Direction.STOP);
                for (Hallway hallway : Hallway.replicationValues) {
                    int hallwayNum = ReplicationComputer.computeReplicationId(hallway);
                    mDesiredDwell[hallwayNum].set((int)DWELL_TIME.getTruncMilliseconds()); 
                }
                //#transition 'T11.7'
                if (landingFloor == 1 && (bothFrontDoorClosed && bothBackDoorClosed)){
                    newState = State.STATE_IDLE;
                }
                break;
            case STATE_COMPUTE:
                desiredFloor = calculateDesiredFloor();

                target = desiredFloor.floor;
                targetHallway = desiredFloor.hallway;
                targetDirection = desiredFloor.direction;

                mDesiredFloor.setFloor(target);
                mDesiredFloor.setHallway(targetHallway);
                mDesiredFloor.setDirection(targetDirection);
                //#transition 'T11.2'
                if (currentFloor == target){
                    newState = State.STATE_SERVE_BEFORE_ARRIVE;
                //#transition 'T11.3'  
                }else if((!(bothFrontDoorClosed && bothBackDoorClosed)) && landingFloor == MessageDictionary.NONE){
                    newState = State.STATE_UNSAFE;
                }
                break;
            case STATE_SERVE_BEFORE_ARRIVE:
                //update the target, targetDirection,
                desiredFloor = calculateDesiredFloor();

                if (landingFloor != target){
                    target = desiredFloor.floor;
                    targetHallway = desiredFloor.hallway;
                    targetDirection = desiredFloor.direction;                    
                }else {
                    targetHallway = desiredFloor.hallway;
                    targetDirection = desiredFloor.direction;  
                }

                mDesiredFloor.setFloor(target);
                mDesiredFloor.setHallway(targetHallway);
                mDesiredFloor.setDirection(targetDirection);
                for (Hallway hallway : Hallway.replicationValues) {
                    int hallwayNum = ReplicationComputer.computeReplicationId(hallway);
                    mDesiredDwell[hallwayNum].set((int)DWELL_TIME.getTruncMilliseconds()); 
                }
                //#transition 'T11.4'
                if (!(bothFrontDoorClosed && bothBackDoorClosed) && landingFloor == target){                        // if (!(bothFrontDoorClosed && bothBackDoorClosed) && landingFloor == target){
                    newState =State.STATE_SERVE_AFTER_ARRIVE;                //     newState =State.STATE_SERVE_AFTER_ARRIVE;
                }                                                               // }
                break;
            case STATE_SERVE_AFTER_ARRIVE:

                mDesiredFloor.setFloor(target);
                mDesiredFloor.setHallway(targetHallway);
                mDesiredFloor.setDirection(targetDirection);
                
                for (Hallway hallway : Hallway.replicationValues) {
                    int hallwayNum = ReplicationComputer.computeReplicationId(hallway);
                    mDesiredDwell[hallwayNum].set((int)DWELL_TIME.getTruncMilliseconds()); 
                }

                if (targetDirection != Direction.STOP){
                    tendency = targetDirection;//change the tendency here! important
                }
                
                //#transition 'T11.6'
                if (bothFrontDoorClosed && bothBackDoorClosed && carCallMessage.isEmpty() && hallCallUpMessage.isEmpty() 
                            && hallCallDownMessage.isEmpty()){
                    newState = State.STATE_IDLE;
                //#transition 'T11.5'
                }else if(bothFrontDoorClosed && bothBackDoorClosed &&(!carCallMessage.isEmpty() || !hallCallUpMessage.isEmpty() 
                            || !hallCallDownMessage.isEmpty())){
                    // System.out.println(" want to wait for some minute");
                    // SimTime startTime = Harness.getTime();
                    // System.out.println("start time: "+startTime.toString());
                    // System.out.println("test add: " + SimTime.add(startTime, INTERVAL).toString());
                    // while (SimTime.add(startTime, INTERVAL).isGreaterThan(Harness.getTime())){
                    //     System.out.println(" current time :" + Harness.getTime());
                    // }

                    newState = State.STATE_STOP;

                    //newState =State.STATE_COMPUTE;
                }        
                break;   
            // this state is used for making the dispatcher delay a bit of time to do nothing because 
            
            case STATE_STOP:
                //#transition 'T11.9'
                if (countStop < totalStop){
                    newState = State.STATE_STOP;
                    countStop++;
                }else{
                    //#transition 'T11.8'
                    countStop = 0;
                    newState =State.STATE_COMPUTE;
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


    private class DesiredFloorObject{
        public int floor;
        public Hallway hallway;
        public Direction direction;
        public DesiredFloorObject(int floor,Hallway hallway,Direction direction){
            this.floor = floor;
            this.hallway = hallway;
            this.direction = direction;
        }
        public DesiredFloorObject(){
            this.floor = MessageDictionary.NONE;
            this.hallway = Hallway.NONE;
            this.direction = Direction.STOP;
        }
    }

    public DesiredFloorObject calculateDesiredFloor(){
        DesiredFloorObject calculatedFloor = new DesiredFloorObject();
        int count = 0;
        int secondDesiredFloor = -1;
        //set floor first
        if (target == currentFloor){
            //System.out.println("TARGET = currentfloor");
            if(tendency == Direction.UP){
                //System.out.println("TENDENCY = UP");
                //server up floor first
                //ascending order
                PriorityQueue<Integer> pq1 = new PriorityQueue<Integer>(7);
                PriorityQueue<Integer> pq2 = new PriorityQueue<Integer>(7, pqs);
                PriorityQueue<Integer> pq3 = new PriorityQueue<Integer>(7, pqs);
                PriorityQueue<Integer> pq4 = new PriorityQueue<Integer>(7);

                for (Integer key : carCallMessage.keySet()) {
                   
                    if (key >= currentFloor){
                        pq1.offer(key);
                    }else{
                        pq3.offer(key);
                    }
                }

                for (Integer key : hallCallUpMessage.keySet()) {
                   
                    if (key >= currentFloor && !pq1.contains(key)){
                        pq1.offer(key);
                    }else if(key < currentFloor){
                        pq4.offer(key);
                    }
                }

                for (Integer key : hallCallDownMessage.keySet()) {
                    if (key >= currentFloor){
                        pq2.offer(key);
                    }else if(key < currentFloor && !pq3.contains(key)){
                        pq3.offer(key);
                    }
                }
                  

                //after we build our structure, we got to decide which floor to go, what the next direction is ,
                //which hallway to open.
                while(pq1.size()!= 0 && count!= 2){
                    if (count == 0){
                        calculatedFloor.floor = pq1.poll();                    
                    }else if(count == 1){
                        secondDesiredFloor = pq1.poll();
                        calculatedFloor.direction = Direction.UP;
                    }
                    count ++;
                }
                while(pq2.size()!= 0 && count!= 2){
                    if (count == 0){
                        calculatedFloor.floor = pq2.poll();
                        calculatedFloor.direction = Direction.DOWN;                    
                    }else if(count == 1){
                        secondDesiredFloor = pq2.poll();
                    }
                    count ++;
                }
                while(pq3.size()!= 0 && count!= 2){
                    if (count == 0){
                        calculatedFloor.floor = pq3.poll();
                                   
                    }else if(count == 1){
                        secondDesiredFloor = pq3.poll();
                    }
                    count ++;
                }
                while(pq4.size()!= 0 && count!= 2){
                    if (count == 0){
                        calculatedFloor.floor = pq4.poll();  
                        calculatedFloor.direction = Direction.UP;            
                    }else if(count == 1){
                        secondDesiredFloor = pq4.poll();
                    }
                    count ++;
                }
                //change the tendency
                if (calculatedFloor.floor >= currentFloor){
                    tendency = Direction.UP;
                }else{
                    tendency = Direction.DOWN;
                }

                if (hallCallUpMessage.containsKey(calculatedFloor.floor)){
                        calculatedFloor.direction = Direction.UP;
                    }

                //if the desired floor is hallcall, so we don't need to consider the second desired floor
                if(calculatedFloor.direction == Direction.STOP && count != 2){
                    if(hallCallUpMessage.containsKey(calculatedFloor.floor)){
                        calculatedFloor.direction = Direction.UP;
                    }else if(hallCallDownMessage.containsKey(calculatedFloor.floor)){
                        calculatedFloor.direction = Direction.DOWN;
                    }
                }
                //if the first desired floor is car call and we do have more than one calls
                if(calculatedFloor.direction == Direction.STOP && count == 2){
                    if(calculatedFloor.floor == secondDesiredFloor){
                        if(hallCallDownMessage.containsKey(secondDesiredFloor)){
                            calculatedFloor.direction = Direction.DOWN;
                        }else{ //the only left case that is  hte hall call up
                            calculatedFloor.direction = Direction.UP;
                        }
                    }else if(calculatedFloor.floor > secondDesiredFloor){
                        calculatedFloor.direction = Direction.DOWN;
                    }else{
                        calculatedFloor.direction = Direction.UP;
                    }
                }

                //set hallway
                //car call first
                //System.out.println("TEST here");
                if (carCallMessage.get(calculatedFloor.floor) != null){
                    calculatedFloor.hallway = carCallMessage.get(calculatedFloor.floor);
                }

                //if car call not at both floor
                if (calculatedFloor.hallway != Hallway.BOTH){
                    
                    if (calculatedFloor.direction == Direction.UP){

                        if(hallCallUpMessage.get(calculatedFloor.floor) != null){
                            if (calculatedFloor.hallway == Hallway.NONE){

                                calculatedFloor.hallway = hallCallUpMessage.get(calculatedFloor.floor);
                            }else if(calculatedFloor.hallway != hallCallUpMessage.get(calculatedFloor.floor)){
                                calculatedFloor.hallway = Hallway.BOTH;
                            }
                        }         
                    }else if(calculatedFloor.direction == Direction.DOWN){
                       
                        if(hallCallDownMessage.get(calculatedFloor.floor) != null){
                           
                            if (calculatedFloor.hallway == Hallway.NONE){
                               
                                calculatedFloor.hallway = hallCallDownMessage.get(calculatedFloor.floor);
                            }else if(calculatedFloor.hallway != hallCallDownMessage.get(calculatedFloor.floor)){
                                
                                calculatedFloor.hallway = Hallway.BOTH;
                            }
                        }
                    }
                    //else no direction means only carcall happens on the same floor   
                }

            }else if(tendency == Direction.DOWN){
                //serve down floor first
               

                PriorityQueue<Integer> pq1 = new PriorityQueue<Integer>(7, pqs);
                PriorityQueue<Integer> pq2 = new PriorityQueue<Integer>(7);
                PriorityQueue<Integer> pq3 = new PriorityQueue<Integer>(7);
                PriorityQueue<Integer> pq4 = new PriorityQueue<Integer>(7, pqs);

                for (Integer key : carCallMessage.keySet()) {
                   
                    if (key > currentFloor){
                        pq3.offer(key);
                    }else{
                        pq1.offer(key);
                    }
                }

                for (Integer key : hallCallUpMessage.keySet()) {
                   
                    if (key > currentFloor && !pq3.contains(key)){
                        pq3.offer(key);
                    }else if(key <= currentFloor){
                        pq2.offer(key);
                    }
                }

                for (Integer key : hallCallDownMessage.keySet()) {
                    if (key > currentFloor ){
                        pq4.offer(key);
                    }else if(key <= currentFloor&& !pq1.contains(key)){
                        pq1.offer(key);
                    }
                }
 
                while(pq1.size()!= 0 && count!= 2){
                        if (count == 0){
                            calculatedFloor.floor = pq1.poll();                    
                        }else if(count == 1){
                            secondDesiredFloor = pq1.poll();
                            calculatedFloor.direction =Direction.DOWN;
                        }
                        count ++;
                    }
                while(pq2.size()!= 0 && count!= 2){
                        if (count == 0){
                            calculatedFloor.floor = pq2.poll();
                            calculatedFloor.direction = Direction.UP;                
                        }else if(count == 1){
                            secondDesiredFloor = pq2.poll();
                        }
                        count ++;
                    }
                while(pq3.size()!= 0 && count!= 2){
                        if (count == 0){
                            calculatedFloor.floor = pq3.poll();                    
                        }else if(count == 1){
                            secondDesiredFloor = pq3.poll();
                        }
                        count ++;
                    }
                while(pq4.size()!= 0 && count!= 2){
                        if (count == 0){
                            calculatedFloor.floor = pq4.poll();
                            calculatedFloor.direction = Direction.DOWN;                    
                        }else if(count == 1){
                            secondDesiredFloor = pq4.poll();
                        }
                        count ++;
                }
                //change the tendency
                if (calculatedFloor.floor > currentFloor){
                    tendency = Direction.UP;
                }else{
                    tendency = Direction.DOWN;
                }

                if (hallCallDownMessage.containsKey(calculatedFloor.floor)){
                        calculatedFloor.direction = Direction.DOWN;
                    }

                //if the desired floor is hallcall, so we don't need to consider the second desired floor
                if(calculatedFloor.direction == Direction.STOP && count != 2 ){
                    if(hallCallDownMessage.containsKey(calculatedFloor.floor)){
                        calculatedFloor.direction = Direction.DOWN;
              
                    }else if(hallCallUpMessage.containsKey(calculatedFloor.floor)){
                        calculatedFloor.direction = Direction.UP;                        
                    }
                }
                //if the first desired floor is car call and we do have more than one calls
                if(calculatedFloor.direction == Direction.STOP && count == 2 ){
                    if(calculatedFloor.floor == secondDesiredFloor){
                        if(hallCallUpMessage.containsKey(secondDesiredFloor)){
                            calculatedFloor.direction = Direction.UP;
                        }else{ //the only left case that is  hte hall call DOWN
                            calculatedFloor.direction = Direction.DOWN;
                        }
                    }else if(calculatedFloor.floor > secondDesiredFloor){
                        calculatedFloor.direction = Direction.DOWN;
                    }else{
                        calculatedFloor.direction = Direction.UP;
                    }
                }
                //set hallway
                //car call first
                if (carCallMessage.get(calculatedFloor.floor) != null){
                    calculatedFloor.hallway = carCallMessage.get(calculatedFloor.floor);
                }

                //if car call not at both floor
                if (calculatedFloor.hallway != Hallway.BOTH){
                    if (calculatedFloor.direction == Direction.UP){

                        if(hallCallUpMessage.get(calculatedFloor.floor) != null){
                            if (calculatedFloor.hallway == Hallway.NONE){
                                calculatedFloor.hallway = hallCallUpMessage.get(calculatedFloor.floor);
                            }else if(calculatedFloor.hallway != hallCallUpMessage.get(calculatedFloor.floor)){
                                calculatedFloor.hallway = Hallway.BOTH;
                            }
                        }         
                    }else if(calculatedFloor.direction == Direction.DOWN){
                        if(hallCallDownMessage.get(calculatedFloor.floor) != null){
                            if (calculatedFloor.hallway == Hallway.NONE){
                                calculatedFloor.hallway = hallCallDownMessage.get(calculatedFloor.floor);
                            }else if(calculatedFloor.hallway != hallCallDownMessage.get(calculatedFloor.floor)){
                                calculatedFloor.hallway = Hallway.BOTH;
                            }
                        }
                    }
                    //else no direction means only carcall happens on the same floor   
                }
                //otherwise use the default STOP direction
            
            }
        }else if(target != currentFloor){
            //if target !=current floor but you sill have a car call or hall call in currentfloor
            //you shall ignore it if it is set as the mdesiredfloor because you can not turn around
            //System.out.println("TARGET != currentfloor");
            if(tendency == Direction.UP){
                //server up floor first
                //ascending order

                PriorityQueue<Integer> pq1 = new PriorityQueue<Integer>(7);
                PriorityQueue<Integer> pq2 = new PriorityQueue<Integer>(7, pqs);
                PriorityQueue<Integer> pq3 = new PriorityQueue<Integer>(7, pqs);
                PriorityQueue<Integer> pq4 = new PriorityQueue<Integer>(7);

                for (Integer key : carCallMessage.keySet()) {
                   
                    if (key > currentFloor){
                        pq1.offer(key);
                    }else{
                        pq3.offer(key);
                    }
                }

                for (Integer key : hallCallUpMessage.keySet()) {
                   
                    if (key > currentFloor && !pq1.contains(key)){
                        pq1.offer(key);
                    }else if(key <= currentFloor){
                        pq4.offer(key);
                    }
                }

                for (Integer key : hallCallDownMessage.keySet()) {
                    if (key > currentFloor ){
                        pq2.offer(key);
                    }else if(key <= currentFloor && !pq3.contains(key)){
                        pq3.offer(key);
                    }
                }
                /*
                Iterator it = carCallMessage.entrySet().iterator();
                while (it.hasNext()) {
                    Map.Entry pair = (Map.Entry)it.next();
                    if (pair.getKey() > currentFloor){
                        pq1.offer(pair.getKey());
                    }else{
                        pq3.offer(pair.getKey());
                    }
                        it.remove(); // avoids a ConcurrentModificationException
                    }

                Iterator it = hallCallUpMessage.entrySet().iterator();
                while (it.hasNext()) {
                    Map.Entry pair = (Map.Entry)it.next();
                    if (pair.getKey() > currentFloor && !pq1.contains(pair.getKey())){
                        pq1.offer(pair.getKey());
                    }else if(pair.getKey() <= currentFloor){
                        pq4.offer(pair.getKey());
                    }
                    it.remove(); // avoids a ConcurrentModificationException
                }

                Iterator it = hallCallDownMessage.entrySet().iterator();
                while (it.hasNext()) {
                    Map.Entry pair = (Map.Entry)it.next();
                    if (pair.getKey() > currentFloor){
                        pq2.offer(pair.getKey());
                    }else if(pair.getKey() <= currentFloor && !pq3.contains(pair.getKey())){
                        pq3.offer(pair.getKey());
                    }
                    it.remove(); // avoids a ConcurrentModificationException
                }
                */
                //after we build our structure, we got to decide which floor to go, what the next direction is ,
                //which hallway to open.
                while(pq1.size()!= 0 && count!= 2){
                    if (count == 0){
                        calculatedFloor.floor = pq1.poll();                    
                    }else if(count == 1){
                        secondDesiredFloor = pq1.poll();
                        calculatedFloor.direction = Direction.UP;
                    }
                    count ++;
                }
                while(pq2.size()!= 0 && count!= 2){
                    if (count == 0){
                        calculatedFloor.floor = pq2.poll();
                        calculatedFloor.direction = Direction.DOWN;                    
                    }else if(count == 1){
                        secondDesiredFloor = pq2.poll();
                    }
                    count ++;
                }
                while(pq3.size()!= 0 && count!= 2){
                    if (count == 0){
                        calculatedFloor.floor = pq3.poll();               
                    }else if(count == 1){
                        secondDesiredFloor = pq3.poll();
                    }
                    count ++;
                }
                while(pq4.size()!= 0 && count!= 2){
                    if (count == 0){
                        calculatedFloor.floor = pq4.poll();  
                        calculatedFloor.direction = Direction.UP;            
                    }else if(count == 1){
                        secondDesiredFloor = pq4.poll();
                    }
                    count ++;
                }
                //change the tendency
                if (calculatedFloor.floor >= currentFloor){
                    tendency = Direction.UP;
                }else{
                    tendency = Direction.DOWN;
                }

                if (hallCallUpMessage.containsKey(calculatedFloor.floor)){
                        calculatedFloor.direction = Direction.UP;
                }

                //if the desired floor is hallcall, so we don't need to consider the second desired floor
                if(calculatedFloor.direction == Direction.STOP && count != 2){
                    if(hallCallUpMessage.containsKey(calculatedFloor.floor)){
                        calculatedFloor.direction = Direction.UP;
                    }else if(hallCallDownMessage.containsKey(calculatedFloor.floor)){
                        calculatedFloor.direction = Direction.DOWN;
                    }
                }


                //if the first desired floor is car call and we do have more than one calls
                if(calculatedFloor.direction == Direction.STOP && count==2 ){
                    if(calculatedFloor.floor == secondDesiredFloor){
                        if(hallCallDownMessage.containsKey(secondDesiredFloor)){
                            calculatedFloor.direction = Direction.DOWN;
                        }else{ //the only left case that is  hte hall call up
                            calculatedFloor.direction = Direction.UP;
                        }
                    }else if(calculatedFloor.floor > secondDesiredFloor){
                        calculatedFloor.direction = Direction.DOWN;
                    }else{
                        calculatedFloor.direction = Direction.UP;
                    }
                }

                //set hallway
                //car call first
                if (carCallMessage.get(calculatedFloor.floor) != null){
                    calculatedFloor.hallway = carCallMessage.get(calculatedFloor.floor);
                }

                //if car call not at both floor
                if (calculatedFloor.hallway != Hallway.BOTH){
                    if (calculatedFloor.direction == Direction.UP){

                        if(hallCallUpMessage.get(calculatedFloor.floor) != null){
                            if (calculatedFloor.hallway == Hallway.NONE){
                                calculatedFloor.hallway = hallCallUpMessage.get(calculatedFloor.floor);
                            }else if(calculatedFloor.hallway != hallCallUpMessage.get(calculatedFloor.floor)){
                                calculatedFloor.hallway = Hallway.BOTH;
                            }
                        }         
                    }else if(calculatedFloor.direction == Direction.DOWN){
                        if(hallCallDownMessage.get(calculatedFloor.floor) != null){
                            if (calculatedFloor.hallway == Hallway.NONE){
                                calculatedFloor.hallway = hallCallDownMessage.get(calculatedFloor.floor);
                            }else if(calculatedFloor.hallway != hallCallDownMessage.get(calculatedFloor.floor)){
                                calculatedFloor.hallway = Hallway.BOTH;
                            }
                        }
                    }
                    //else no direction means only carcall happens on the same floor   
                }

            }else if(tendency == Direction.DOWN){
                //serve down floor first
                PriorityQueue<Integer> pq1 = new PriorityQueue<Integer>(7, pqs);
                PriorityQueue<Integer> pq2 = new PriorityQueue<Integer>(7);
                PriorityQueue<Integer> pq3 = new PriorityQueue<Integer>(7);
                PriorityQueue<Integer> pq4 = new PriorityQueue<Integer>(7, pqs);

                for (Integer key : carCallMessage.keySet()) {
                   
                    if (key >= currentFloor){
                        pq3.offer(key);
                    }else{
                        pq1.offer(key);
                    }
                }

                for (Integer key : hallCallUpMessage.keySet()) {
                   
                    if (key >= currentFloor && !pq3.contains(key)){
                        pq3.offer(key);
                    }else if(key < currentFloor){
                        pq2.offer(key);
                    }
                }

                for (Integer key : hallCallDownMessage.keySet()) {
                    if (key >= currentFloor ){
                        pq4.offer(key);
                    }else if(key < currentFloor&& !pq1.contains(key)){
                        pq1.offer(key);
                    }
                }
                /*
                Iterator it = carCallMessage.entrySet().iterator();
                while (it.hasNext()) {
                    Map.Entry pair = (Map.Entry)it.next();
                    if (pair.getKey() >= currentFloor){
                        pq3.offer(pair.getKey());
                    }else{
                        pq1.offer(pair.getKey());
                    }
                    it.remove(); // avoids a ConcurrentModificationException
                }

                Iterator it = hallCallUpMessage.entrySet().iterator();
                while (it.hasNext()) {
                    Map.Entry pair = (Map.Entry)it.next();
                    if (pair.getKey() >= currentFloor && !pq3.contains(pair.getKey())){
                        pq3.offer(pair.getKey());
                    }else if(pair.getKey() < currentFloor){
                        pq2.offer(pair.getKey());
                    }
                    it.remove(); // avoids a ConcurrentModificationException
                }

                Iterator it = hallCallDownMessage.entrySet().iterator();
                while (it.hasNext()) {
                    Map.Entry pair = (Map.Entry)it.next();
                    if (pair.getKey() >= currentFloor){
                        pq4.offer(pair.getKey());
                    }else if(pair.getKey() < currentFloor && !pq1.contains(pair.getKey())){
                        pq1.offer(pair.getKey());
                    }
                    it.remove(); // avoids a ConcurrentModificationException
                }
                */
                while(pq1.size()!= 0 && count!= 2){
                        if (count == 0){
                            calculatedFloor.floor = pq1.poll();                    
                        }else if(count == 1){
                            secondDesiredFloor = pq1.poll();
                            calculatedFloor.direction =Direction.DOWN;
                        }
                        count ++;
                    }
                while(pq2.size()!= 0 && count!= 2){
                        if (count == 0){
                            calculatedFloor.floor = pq2.poll();
                            calculatedFloor.direction = Direction.UP;                
                        }else if(count == 1){
                            secondDesiredFloor = pq2.poll();
                        }
                        count ++;
                    }
                while(pq3.size()!= 0 && count!= 2){
                        if (count == 0){
                            calculatedFloor.floor = pq3.poll();                    
                        }else if(count == 1){
                            secondDesiredFloor = pq3.poll();
                        }
                        count ++;
                    }
                while(pq4.size()!= 0 && count!= 2){
                        if (count == 0){
                            calculatedFloor.floor = pq4.poll();
                            calculatedFloor.direction = Direction.DOWN;                    
                        }else if(count == 1){
                            secondDesiredFloor = pq4.poll();
                        }
                        count ++;
                }
                //change the tendency
                if (calculatedFloor.floor > currentFloor){
                    tendency = Direction.UP;
                }else{
                    tendency = Direction.DOWN;
                }

                if (hallCallDownMessage.containsKey(calculatedFloor.floor)){
                        calculatedFloor.direction = Direction.DOWN;
                    }

                //if the desired floor is hallcall, so we don't need to consider the second desired floor
                if(calculatedFloor.direction == Direction.STOP && count != 2 ){
                    if(hallCallDownMessage.containsKey(calculatedFloor.floor)){
                        calculatedFloor.direction = Direction.DOWN;
              
                    }else if(hallCallUpMessage.containsKey(calculatedFloor.floor)){
                        calculatedFloor.direction = Direction.UP;                        
                    }
                }

                //if the first desired floor is car call and we do have more than one calls
                if(calculatedFloor.direction == Direction.STOP && count == 2 ){
                    if(calculatedFloor.floor == secondDesiredFloor){
                        if(hallCallUpMessage.containsKey(secondDesiredFloor)){
                            calculatedFloor.direction = Direction.UP;
                        }else{ //the only left case that is  hte hall call DOWN
                            calculatedFloor.direction = Direction.DOWN;
                        }
                    }else if(calculatedFloor.floor > secondDesiredFloor){
                        calculatedFloor.direction = Direction.DOWN;
                    }else{
                        calculatedFloor.direction = Direction.UP;
                    }
                }

                //set hallway
                //car call first
                if (carCallMessage.get(calculatedFloor.floor) != null){
                    calculatedFloor.hallway = carCallMessage.get(calculatedFloor.floor);
                }

                //if car call not at both floor
                if (calculatedFloor.hallway != Hallway.BOTH){
                    if (calculatedFloor.direction == Direction.UP){

                        if(hallCallUpMessage.get(calculatedFloor.floor) != null){
                            if (calculatedFloor.hallway == Hallway.NONE){
                                calculatedFloor.hallway = hallCallUpMessage.get(calculatedFloor.floor);
                            }else if(calculatedFloor.hallway != hallCallUpMessage.get(calculatedFloor.floor)){
                                calculatedFloor.hallway = Hallway.BOTH;
                            }
                        }         
                    }else if(calculatedFloor.direction == Direction.DOWN){
                        if(hallCallDownMessage.get(calculatedFloor.floor) != null){
                            if (calculatedFloor.hallway == Hallway.NONE){
                                calculatedFloor.hallway = hallCallDownMessage.get(calculatedFloor.floor);
                            }else if(calculatedFloor.hallway != hallCallDownMessage.get(calculatedFloor.floor)){
                                calculatedFloor.hallway = Hallway.BOTH;
                            }
                        }
                    }
                       //else no direction means only carcall happens on the same floor   
                }
            }
        }
        return calculatedFloor;
    }

}

