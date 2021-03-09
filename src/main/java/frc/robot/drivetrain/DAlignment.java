package frc.robot.drivetrain;

import frc.robot.Constants;
import frc.robot.limelight.*;

class Alignment {
    private LimeLight limeLight;
    private LemonTorch lemonTorch;

    Alignment() {
        limeLight = new LimeLight();
        lemonTorch = new LemonTorch();
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

    public boolean targetFoundLT() {
        limeLight.selectPipeline(Constants.PORT_PIPELINE_LT);

        if (lemonTorch.getTv() == 0) {
            return false;
        }
        return true;
    }

    public double getErrorLT() {
        if (targetFoundLT()) {
            return -lemonTorch.getTx();
        }
        return -1;
    }


}