package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public enum FieldPosition{
    Speaker(Constants.FieldConstants.TEAM_RED_SPEAKER, Constants.FieldConstants.TEAM_BLUE_SPEAKER),
    Amp(Constants.FieldConstants.TEAM_RED_AMP, Constants.FieldConstants.TEAM_BLUE_AMP)
    ;

    private Translation2d red;
    private Translation2d blue;


    private FieldPosition(Translation2d red, Translation2d blue){
        this.red=red;        this.blue=blue;
    }

    public Translation2d getPose(){
        if(DriverStationUtils.getCurrentAlliance() == Alliance.Red)
            return red;
        else 
            return blue;
    }
  }

