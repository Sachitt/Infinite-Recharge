package frc.robot.drivetrain;

import frc.robot.Constants;
import frc.robot.limelight.*;

class Alignment {
    private LimeLight limeLight;

    Alignment() {
        limeLight = new LimeLight();
    }

    public boolean targetFound() {
        limeLight.selectPipeline(Constants.PORT_PIPELINE);

        if (limeLight.getTv() == 0) {
            return false;
        }
        return true;
    }

    public double getError() {
        if (targetFound()) {
            return -limeLight.getTx();
        }
        return -1;
    }
}