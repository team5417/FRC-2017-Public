package org.usfirst.frc.team5417.robot;

//import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;

public class GearShift {

	public static class GearSelection {
		public final int mode;
		
		public GearSelection(int mode) {
			this.mode = mode;
		}
	}
	public static final GearSelection kLowGear = new GearSelection(2);
	public static final GearSelection kHighGear = new GearSelection(3);
	
	private GearSelection currentGear = kLowGear;
	
	Solenoid shiftSolenoid1;
	
	public GearShift(Solenoid shiftSolenoid1) {
		this.shiftSolenoid1 = shiftSolenoid1;
	}

	public GearSelection getCurrentGear() {
		return currentGear;
	}

	public void initGearShift() {
		shiftSolenoid1.set(false);
		SmartDashboard.putString("GearShift", "LowGear");
	}
	
	public void gearShiftHigh(final double leftMotorPower, final double rightMotorPower) {
		// don't switch if the motors are moving (safety)
		if (leftMotorPower == 0 && rightMotorPower == 0) {
			// switch to high gear if we're in low gear right now
			if (this.getCurrentGear() == kLowGear) { 
				shiftSolenoid1.set(true);
				currentGear = kHighGear;
			}
		}
	}
	public void gearShiftLow(final double leftMotorPower, final double rightMotorPower) {
		if (leftMotorPower == 0 && rightMotorPower == 0) {
			
			// otherwise, switch to low gear if we're in high gear right now
			if (this.getCurrentGear() == kHighGear) {
				shiftSolenoid1.set(false);
				currentGear = kLowGear;
			}

		}

	}
}
