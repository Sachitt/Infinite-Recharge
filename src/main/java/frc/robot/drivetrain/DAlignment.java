package frc.robot.drivetrain;

import frc.robot.Constants;
import frc.robot.limelight.*;

class DAlignment {
    private LimeLight limeLight;
    private LemonTorch lemonTorch;

    DAlignment() {
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
        lemonTorch.selectPipeline(Constants.PORT_PIPELINE_LT);

        if (lemonTorch.getTv() == 0) {
            System.out.println("TV NOT FOUND");
            return false;
        }
        System.out.println("IDK");
        return true;
    }

    public double getErrorLT() {
        if (targetFoundLT()) {
            return -lemonTorch.getTx();
        }
        return -1;
    }

    public String determinePath() {
        
        return "hi";
    }


}