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

func (m *Motor)powerUse(shaftSpeed, torque float64) (mechanical, loss float64) {
	mechanical = shaftSpeed * torque
	total := et(mechanical, m.Efficiency)
	loss = math.Abs(total) * (1 - m.Efficiency)
	return
}

func (m *Motor)MaxTorque(sim *SimulatorState, shaftSpeed) (float64, error) {
	shaftSpeed = math.Abs(shaftSpeed)
	if (shaftSpeed > m.MaxShaftSpeed) {
		return 0, fmt.Errorf("Maximum shaft speed")
	}
	if((m.Peak.Torque * shaftSpeed) > m.Peak.Power) {
		return m.Peak.Power/shaftSpeed, fmt.Errorf("Maximum power")
	}
	return m.Peak.Torque, fmt.Errorf("Maximum torque")
}

func (m *Motor)Operate(sim *SimulatorState, shaftSpeed, torque float64) Power {
	mech, loss := m.powerUse(shaftSpeed, torque)
	s.power["Losses"] = loss
	s.power["Mechanical"] = mech
	return mech + loss
}
