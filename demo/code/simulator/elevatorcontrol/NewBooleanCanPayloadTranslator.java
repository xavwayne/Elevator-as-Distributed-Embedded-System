package simulator.elevatorcontrol;

import java.util.BitSet;

import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

/**
 * This is a new CAN payload translator for boolean messages.  It
 * takes one boolean value and uses 1 byte to send this message.
 * 
 * @author Xiao Guo
 * 
 */
public class NewBooleanCanPayloadTranslator extends CanPayloadTranslator {

	String name;

	public NewBooleanCanPayloadTranslator(WriteableCanMailbox payload) {
        super(payload, 1);
    }
    
	public NewBooleanCanPayloadTranslator(ReadableCanMailbox payload) {
        super(payload, 1);
    }

    //required for reflection
    public void set(boolean value) {
        setValue(value);
    }

    public void setValue(boolean value) {
        BitSet b = getMessagePayload();
        b.set(7, value);
        setMessagePayload(b, getByteSize());
    }

    public boolean getValue() {
        return getMessagePayload().get(7);
    }

    @Override
    public String payloadToString() {
        return name + " = " + getValue();
    }

}
