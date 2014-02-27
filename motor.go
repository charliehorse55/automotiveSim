package automotiveSim


import (
	"time"
	"fmt"
)

type MotorPerformance struct {
	Torque float64
	Power float64
}

type Motor struct {
	Peak MotorPerformance
	Continuous MotorPerformance
    MaxShaftSpeed float64
	Efficiency float64
}

type motorState struct {
	// timeAtPeak time.Duration
	motor *Motor
}

func (m *Motor)Init() error {
	if m.Peak.Torque < m.Continuous.Torque {
		return fmt.Errorf("Peak torque must not be less than continuous torque")
	}
	if m.Peak.Power < m.Continuous.Power {
		return fmt.Errorf("Peak power must not be less than continuous power")
	}
	if m.Continuous.Torque <= 0 {
		return fmt.Errorf("Continuous torque must be positive")
	}
	if m.Continuous.Power <= 0 {
		return fmt.Errorf("Continuous power must be positive")
	}
	
	if m.MaxShaftSpeed <= 0 {
		return fmt.Errorf("Maximum shaft speed must be positive")
	}
	
	if m.Efficiency <= 0 || m.Efficiency > 1 {
		return fmt.Errorf("Motor efficiency must be on the range (0,1]")
	}
	
	return nil
}

func NewMotorState(m *Motor) *motorState {
	return &motorState{ motor:m}
}


func (s *motorState)MaxTorque(shaftSpeed float64, duration time.Duration) (float64, string) {
	if s.motor.Peak.Torque * shaftSpeed > s.motor.Peak.Power { //TODO add code for overheating
		return s.motor.Peak.Power / shaftSpeed, "Power"
	}
	return s.motor.Peak.Torque, "Torque"
}

func (s *motorState)Operate(shaftSpeed, torque float64, duration time.Duration) float64 {	
	//energy transfer direction
	power := shaftSpeed * torque 
	if power > 0 {
		power /= s.motor.Efficiency
	} else {
		power *= s.motor.Efficiency
	}
	return power
}
