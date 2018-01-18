package org.usfirst.frc.team5417.robot;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class UpdatablePIDSource implements PIDSource {

	private double value = 0;
	private PIDSourceType type = PIDSourceType.kDisplacement;
	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		type = pidSource;
		
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		
		return type;
	}

	@Override
	public double pidGet() {

		return value;
	}
	
	public void update(double value) {
		this.value = value;
	}

}
