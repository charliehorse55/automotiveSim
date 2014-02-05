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
	timeAtPeak time.Duration
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
	return &motorState{ motor:m, timeAtPeak:0.0}
}

func (s *motorState)CanOperate(shaftSpeed, torque, busVoltage float64, duration time.Duration) error {
	shaftSpeed = math.Abs(shaftSpeed)
	torque = math.Abs(torque)
	
	if shaftSpeed > s.motor.MaxShaftSpeed {
		return fmt.Errorf("Motor Angular Velocity")
	}
	
	if torque > s.motor.Peak.Torque {
		return fmt.Errorf("Motor Torque")
	}
	
	power := shaftSpeed * torque
	if power > s.motor.Peak.Power {
		return fmt.Errorf("Motor Power")
	}
	
	if torque > s.motor.Continuous.Torque || power > s.motor.Continuous.Power {
		if s.timeAtPeak > (time.Second * 20) {
			return fmt.Errorf("Motor Temperature")
		}
	}

	return nil
}

// just calculate power required by an approximation based on a single efficiency value for now
func (s *motorState)PowerAtTorque(shaftSpeed, torque, duration time.Duration) float64 {
	return (shaftSpeed * torque) / s.motor.Efficiency
}

func (s *motorState)Operate(shaftSpeed, torque, busVoltage float64, duration time.Duration) {
	shaftSpeed = math.Abs(shaftSpeed)
	torque = math.Abs(torque)
	
	overload := 0.0
	if torque > s.motor.Continuous.Torque {
		overload = (torque - s.motor.Continuous.Torque)/(s.motor.Peak.Torque - s.motor.Continuous.Torque)
	}
	
	power := torque * shaftSpeed
	if power > s.motor.Continuous.Power {
		poveload := (power - s.motor.Continuous.Power)/(s.motor.Peak.Power - s.motor.Continuous.Power)
		if poveload > overload {
			overload = poveload
		}
	}
	
	if overload == 0.0 {
		s.timeAtPeak -= duration / 4
	}
	
	s.timeAtPeak += time.Duration(float64(duration) * overload)
}
