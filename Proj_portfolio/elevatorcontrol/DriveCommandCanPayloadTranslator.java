package simulator.elevatorcontrol;

import java.util.BitSet;
import simulator.framework.Direction;
import simulator.framework.Speed;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

/**
 * Can payload translator for the drive command, which includes a speed value and a direction.
 * Only one byte is used to send this message
 * 
 * @author Xiao Guo
 */
public class DriveCommandCanPayloadTranslator extends CanPayloadTranslator {

    public DriveCommandCanPayloadTranslator(WriteableCanMailbox p) {
        super(p, 1, MessageDictionary.DRIVE_COMMAND_CAN_ID);
    }
    
    public DriveCommandCanPayloadTranslator(ReadableCanMailbox p) {
        super(p, 1, MessageDictionary.DRIVE_COMMAND_CAN_ID);
    }

    /**
     * This method is required for setting values by reflection in the
     * MessageInjector.  The order of parameters in .mf files should match the
     * signature of this method.
     * All translators must have a set() method with the signature that contains
     * all the parameter values.
     *
     * @param speed
     * @param dir
     */
    public void set(Speed speed, Direction dir) {
        setSpeed(speed);
        setDirection(dir);
    }
    
    public void setSpeed(Speed speed) {
        BitSet b = getMessagePayload();
        addUnsignedIntToBitset(b, speed.ordinal(), 0, 2);
        setMessagePayload(b, getByteSize());
    }

    public Speed getSpeed() {
        int val = getUnsignedIntFromBitset(getMessagePayload(), 0, 2);
        for (Speed s : Speed.values()) {
            if (s.ordinal() == val) {
                return s;
            }
        }
        throw new RuntimeException("Unrecognized Speed Value " + val);
    }

    public void setDirection(Direction dir) {
        BitSet b = getMessagePayload();
        addUnsignedIntToBitset(b, dir.ordinal(), 2, 2);
        setMessagePayload(b, getByteSize());
    }

    public Direction getDirection() {
        int val = getUnsignedIntFromBitset(getMessagePayload(), 2, 2);
        for (Direction d : Direction.values()) {
            if (d.ordinal() == val) {
                return d;
            }
        }
        throw new RuntimeException("Unrecognized Direction Value " + val);
    }

    @Override
    public String payloadToString() {
        return "DriveCommand:  speed=" + getSpeed() + " direction=" + getDirection();
    }
}
