package frc.Mechanisms.drivetrain;

import com.kauailabs.navx.frc.AHRS;

public class GyroIONavX implements GyroIO 
{
    private final AHRS navX;
  
    public GyroIONavX() {
        navX = new AHRS();
        navX.setAngleAdjustment(-navX.getYaw());
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) 
    {
      inputs.gyroAngle = navX.getAngle();
      inputs.gyroYaw = navX.getYaw();
      inputs.gyroRoll = navX.getRoll();
    }

    @Override
    public void resetNavXIO()
    {
        navX.reset();
    }

    @Override
    public void setAngleAdjustmentIO(double gyroYaw) 
    {
        navX.setAngleAdjustment(gyroYaw);
    }


}

