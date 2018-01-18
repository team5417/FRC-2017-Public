package org.usfirst.frc.team5417.robot;

import edu.wpi.first.wpilibj.PIDOutput;

public class UpdatablePIDOutput implements PIDOutput{

	private double value;
	@Override
	public void pidWrite(double output) {
		this.value = output;
		
	}
	
	public double getValue() {
		return this.value;
	}
}
