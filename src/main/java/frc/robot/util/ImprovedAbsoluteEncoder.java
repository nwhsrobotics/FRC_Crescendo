package frc.robot.util;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ImprovedAbsoluteEncoder extends DutyCycleEncoder {
    private double offset = 0;

    /**
     * Create a new absolute encoder by DIO channel.
     *
     * <p>
     * <p>
     * The offset is set to 0 by default.
     *
     * @param channel - DIO channel to connect over.
     */
    public ImprovedAbsoluteEncoder(int channel) {
        super(channel);
    }

    /**
     * Create a new absolute encoder
     *
     * @param channel
     * @param offset
     */
    public ImprovedAbsoluteEncoder(int channel, double offset) {
        super(channel);
        this.offset = offset;
    }

    @Override
    public double getAbsolutePosition() {
        return super.getAbsolutePosition();
    }
}
