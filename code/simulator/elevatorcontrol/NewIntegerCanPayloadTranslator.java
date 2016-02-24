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
 * This class takes a single integer value and translates it to a
 * 4 byte CanMailbox
 * 
 * @author Xiao Guo
 */
public class NewIntegerCanPayloadTranslator extends CanPayloadTranslator {

    /**
     * Constructor for use with WriteableCanMailbox objects
     * @param payload
     */
    public NewIntegerCanPayloadTranslator(WriteableCanMailbox payload) {
        super(payload, 4);
    }

    /**
     * Constructor for use with ReadableCanMailbox objects
     * @param payload
     */

    public NewIntegerCanPayloadTranslator(ReadableCanMailbox payload) {
        super(payload, 4);
    }

    
    //required for reflection
    public void set(int value) {
        setValue(value);
    }
    
    public void setValue(int value) {
        BitSet b = new BitSet();
        addIntToBitset(b, value, 0, 32);
        setMessagePayload(b, getByteSize());
    }
    
    public int getValue() {
        return getIntFromBitset(getMessagePayload(), 0, 32);
    }
    
    @Override
    public String payloadToString() {
        return "0x" + Integer.toString(getValue(),16);
    }
}
