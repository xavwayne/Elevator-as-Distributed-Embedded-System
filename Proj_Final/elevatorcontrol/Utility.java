/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package simulator.elevatorcontrol;

import java.util.HashMap;
import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.payloads.CANNetwork;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.Harness;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
//added for fast elevator
import simulator.framework.Direction;
/**
 * This class provides some example utility classes that might be useful in more
 * than one spot.  It is okay to create new classes (or modify the ones given
 * below), but you may not use utility classes in such a way that they constitute
 * a communication channel between controllers.
 *
 * @author justinr2
 */
public class Utility {

    public static class DoorClosedArray {

        HashMap<Integer, DoorClosedCanPayloadTranslator> translatorArray = new HashMap<Integer, DoorClosedCanPayloadTranslator>();
        public final Hallway hallway;

        public DoorClosedArray(Hallway hallway, CANNetwork.CanConnection conn) {
            this.hallway = hallway;
            for (Side s : Side.values()) {
                int index = ReplicationComputer.computeReplicationId(hallway, s);
                ReadableCanMailbox m = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + index);
                DoorClosedCanPayloadTranslator t = new DoorClosedCanPayloadTranslator(m, hallway, s);
                conn.registerTimeTriggered(m);
                translatorArray.put(index, t);
            }
        }

        public boolean getBothClosed() {
            return translatorArray.get(ReplicationComputer.computeReplicationId(hallway, Side.LEFT)).getValue() &&
                    translatorArray.get(ReplicationComputer.computeReplicationId(hallway, Side.RIGHT)).getValue();
        }
    }

    public static class AtFloorArray {

        public HashMap<Integer, AtFloorCanPayloadTranslator> networkAtFloorsTranslators = new HashMap<Integer, AtFloorCanPayloadTranslator>();
        public final int numFloors = Elevator.numFloors;

        public AtFloorArray(CANNetwork.CanConnection conn) {
            for (int i = 0; i < numFloors; i++) {
                int floor = i + 1;
                for (Hallway h : Hallway.replicationValues) {
                    int index = ReplicationComputer.computeReplicationId(floor, h);
                    ReadableCanMailbox m = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + index);
                    AtFloorCanPayloadTranslator t = new AtFloorCanPayloadTranslator(m, floor, h);
                    conn.registerTimeTriggered(m);
                    networkAtFloorsTranslators.put(index, t);
                }
            }
        }
        
        public boolean isAtFloor(int floor, Hallway hallway) {
            return networkAtFloorsTranslators.get(ReplicationComputer.computeReplicationId(floor, hallway)).getValue();
        }

        public int getCurrentFloor() {
            int retval = MessageDictionary.NONE;
            for (int i = 0; i < numFloors; i++) {
                int floor = i + 1;
                for (Hallway h : Hallway.replicationValues) {
                    int index = ReplicationComputer.computeReplicationId(floor, h);
                    AtFloorCanPayloadTranslator t = networkAtFloorsTranslators.get(index);
                    if (t.getValue()) {
                        if (retval == MessageDictionary.NONE) {
                            //this is the first true atFloor
                            retval = floor;
                        } else if (retval != floor) {
                            //found a second floor that is different from the first one
                            throw new RuntimeException("AtFloor is true for more than one floor at " + Harness.getTime());
                        }
                    }
                }
            }
            return retval;
        }
    }

    //the modified part of utility from here , used for the input of Dispatcher
    public static class CarCallArray {
        HashMap<Integer, NewBooleanCanPayloadTranslator> carCallTranslators = new HashMap<Integer, NewBooleanCanPayloadTranslator>();
        public final int numFloors = Elevator.numFloors;
        //Constructor of CarCallArray, used to get all the translators and register all the network message for read
        public CarCallArray(CANNetwork.CanConnection conn){
            for(int i = 0;i < numFloors; i++){
                int floor = i + 1;
                for (Hallway h : Hallway.replicationValues) {
                    int index = ReplicationComputer.computeReplicationId(floor, h);
                    ReadableCanMailbox mCarCall = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_CALL_BASE_CAN_ID + index);
                    NewBooleanCanPayloadTranslator t = new NewBooleanCanPayloadTranslator(mCarCall);
                    conn.registerTimeTriggered(mCarCall);
                    carCallTranslators.put(index, t);
                }
            }
        }
        //return method,Integer is the floor parameter and 
        public  HashMap<Integer,Hallway> getPressedCarCall(){
            HashMap<Integer,Hallway> result = new HashMap<Integer,Hallway>();
            for (int i = 0; i < numFloors; i++) {
                int floor = i + 1;
                for (Hallway h : Hallway.replicationValues) {
                    int index = ReplicationComputer.computeReplicationId(floor, h);
                    NewBooleanCanPayloadTranslator t = carCallTranslators.get(index); 
                    if (result.containsKey(floor) && t.getValue()){
                        result.put(floor,Hallway.BOTH); 
                    }else if (t.getValue()){
                        result.put(floor, h);
                    }
                }
            }
            return result;
        }

        public boolean checkCarCall(int floor, Hallway hallway){
            return carCallTranslators.get(ReplicationComputer.computeReplicationId(floor, hallway)).getValue();
        }

    }
    public static class HallCallArray{
        HashMap<Integer, NewBooleanCanPayloadTranslator> hallCallTranslators = new HashMap<Integer, NewBooleanCanPayloadTranslator>();
        public final int numFloors = Elevator.numFloors;
        public HallCallArray(CANNetwork.CanConnection conn){
            for(int i = 0;i < numFloors; i++){
                int floor = i + 1;
                for (Hallway h : Hallway.replicationValues) {
                    for (Direction d: Direction.replicationValues){
                        int index = ReplicationComputer.computeReplicationId(floor, h, d);
                        ReadableCanMailbox mHallCall = CanMailbox.getReadableCanMailbox(MessageDictionary.HALL_CALL_BASE_CAN_ID + index);
                        NewBooleanCanPayloadTranslator t = new NewBooleanCanPayloadTranslator(mHallCall);
                        conn.registerTimeTriggered(mHallCall);
                        hallCallTranslators.put(index, t);                 
                    }
                }
            }
        }

        public  HashMap<Integer,Hallway> getPressedUpHallCall(){
            HashMap<Integer,Hallway> result = new HashMap<Integer,Hallway>();
            for (int i = 0; i < numFloors; i++) {
                int floor = i + 1;
                for (Hallway h : Hallway.replicationValues) {
                    int index = ReplicationComputer.computeReplicationId(floor, h ,Direction.UP);
                    NewBooleanCanPayloadTranslator t = hallCallTranslators.get(index); 
                    if (result.containsKey(floor) && t.getValue()){
                        result.put(floor,Hallway.BOTH); 
                    }else if (t.getValue()){
                        result.put(floor, h);
                    }
                }
            }
            return result;
        }

        public  HashMap<Integer,Hallway> getPressedDownHallCall(){
            HashMap<Integer,Hallway> result = new HashMap<Integer,Hallway>();
            for (int i = 0; i < numFloors; i++) {
                int floor = i + 1;
                for (Hallway h : Hallway.replicationValues) {
                    int index = ReplicationComputer.computeReplicationId(floor, h ,Direction.DOWN);
                    NewBooleanCanPayloadTranslator t = hallCallTranslators.get(index); 
                    if (result.containsKey(floor) && t.getValue()){
                        result.put(floor,Hallway.BOTH); 
                    }else if (t.getValue()){
                        result.put(floor, h);
                    }
                }
            }
            return result;
        }

        public boolean checkHallCall(int floor, Hallway hallway, Direction direction){
            return hallCallTranslators.get(ReplicationComputer.computeReplicationId(floor,hallway,direction)).getValue();
        }
    }

}
