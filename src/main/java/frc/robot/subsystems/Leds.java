package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Leds extends SubsystemBase{
    private CANdle led;
    private CANdleConfiguration config = new CANdleConfiguration();

    private String gameData = "";
    private boolean blueWonAuto = false;
    private double matchTime = 0;
    private double shiftTime = 0;
    private boolean ourHubActive = true;

    public Leds() {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void displayMatchInfo() {
        matchTime = DriverStation.getMatchTime();

        gameData = DriverStation.getGameSpecificMessage();

        if (gameData.length() > 0) {
            if (gameData.length() > 0) {
                if (gameData.charAt(0) == 'R') blueWonAuto = false;
                if (gameData.charAt(0) == 'B') blueWonAuto = true;
            }
            if (matchTime > 130) {
                shiftTime = matchTime - 130;
                ourHubActive = true;
            } else if (matchTime > 105) {
                shiftTime = matchTime - 105;
                ourHubActive = !(isBlue() ^ blueWonAuto);
            } else if (matchTime > 80) {
                shiftTime = matchTime - 80;
                ourHubActive = !(isBlue() ^ blueWonAuto);
            } else if (matchTime > 55) {
                shiftTime = matchTime - 55;
                ourHubActive = !(isBlue() ^ blueWonAuto);
            } else if (matchTime > 30) {
                shiftTime = matchTime - 30;
                ourHubActive = !(isBlue() ^ blueWonAuto);
            } else {
                shiftTime = matchTime;
                ourHubActive = true;
            }

            SmartDashboard.putString ("# who won auto", gameData.length() < 1 ? "No Data" : blueWonAuto ? "Blue" : "Red");
            SmartDashboard.putNumber("# Shift Time", shiftTime);
            SmartDashboard.putBoolean("# our hub active", ourHubActive);
        }

    }

    private boolean isBlue() {
        return DriverStation.getAlliance().get() != DriverStation.Alliance.Red;
    }

}
