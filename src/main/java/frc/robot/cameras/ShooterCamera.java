package frc.robot.cameras;

import frc.robot.subsystems.lowlight.Camera;

public class ShooterCamera implements Camera {
    @Override
    public int getIndex() {
        return 0;
    }

    @Override
    public int getQuality() {
        return 75;
    }
}
