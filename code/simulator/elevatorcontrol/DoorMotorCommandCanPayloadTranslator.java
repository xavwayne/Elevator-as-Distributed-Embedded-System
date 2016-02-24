/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package simulator.elevatorcontrol;

import java.util.BitSet;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

/**
 * This is a new CAN payload translator for DoorMotor Command messages. It
 * takes one command value and uses 1 byte to send this message.
 * 
 * @author Xiao Guo
 * 
 */
public class DoorMotorCommandCanPayloadTranslator extends CanPayloadTranslator {

    /**
     * Constructor for use with WriteableCanMailbox objects
     * @param payload
     */
    public DoorMotorCommandCanPayloadTranslator(WriteableCanMailbox payload) {
        super(payload, 1);
    }

    /**
     * Constructor for use with ReadableCanMailbox objects
     * @param payload
     */

    public DoorMotorCommandCanPayloadTranslator(ReadableCanMailbox payload) {
        super(payload, 1);
    }

    
    //required for reflection
    public void set(int value) {
        setValue(value);
    }
    
    public void setValue(int value) {
        BitSet b = new BitSet();
        addIntToBitset(b, value, 0, 3);
        setMessagePayload(b, getByteSize());
    }
    
    public int getValue() {
        return getIntFromBitset(getMessagePayload(), 0, 3);
    }
    
    @Override
    public String payloadToString() {
        return "0x" + Integer.toString(getValue(),16);
    }
}
