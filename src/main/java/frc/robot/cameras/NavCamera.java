package frc.robot.cameras;

import frc.robot.subsystems.lowlight.Camera;

public class NavCamera implements Camera {
    @Override
    public int getIndex() {
        return 1;
    }

    @Override
    public int getQuality() {
        return 75;
    }
}
