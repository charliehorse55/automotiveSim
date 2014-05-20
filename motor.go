package automotiveSim


import (
	"time"
	"fmt"
	"math"
)

type MotorPerformance struct {
	Torque float64
	Power float64
}

type Motor struct {
	Name string
	Peak MotorPerformance
	Continuous MotorPerformance
    MaxShaftSpeed float64
	Efficiency float64
}

type motorState struct {
	// timeAtPeak time.Duration
	motor *Motor
	power Power
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
	return &motorState{motor:m, power:make(Power)}
}

func (s *motorState)powerUse(shaftSpeed, torque float64) (mechanical, loss float64) {
	mechanical = shaftSpeed * torque
	total := et(mechanical, s.motor.Efficiency)
	loss = math.Abs(total) * (1 - s.motor.Efficiency)
	return
}

func (s *motorState)CanOperate(sim *SimulatorState, shaftSpeed, torque float64, duration time.Duration) (float64, error) {
	// TODO add code for overheating
	if math.Abs(shaftSpeed) > s.motor.MaxShaftSpeed {
		return 0, fmt.Errorf("Maximum Shaft speed")
	}
	if math.Abs(torque) > s.motor.Peak.Torque {
		return 0, fmt.Errorf("Maximum Torque")
	}
	if math.Abs(torque * shaftSpeed) > s.motor.Peak.Power {
		return 0, fmt.Errorf("Maximum Power")
	}
	mech, loss := s.powerUse(shaftSpeed, torque)
	return mech + loss, nil
}

func (s *motorState)Operate(sim *SimulatorState, shaftSpeed, torque float64, duration time.Duration) float64 {
	mech, loss := s.powerUse(shaftSpeed, torque)
	s.power["Losses"] = loss
	s.power["Mechanical"] = mech
	return mech + loss
}
