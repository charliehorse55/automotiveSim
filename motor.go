package automotiveSim


import (
	"time"
	"errors"
)

type MotorPerformance struct {
	Torque float64
	Power float64
}

type Motor struct {
	Peak MotorPerformance
	Continuous MotorPerformance
    MaxShaftSpeed float64
}

type motorState struct {
	timeAtPeak time.Duration
	motor *Motor
}

func (m *Motor)Init() bool {
	return true //TODO actually check that everything has correct values
}

func newMotorState(m *motor) *motorState {
	return &motorState{ motor:m, timeAtPeak:0.0}
}

func (s *motorState)CanOperate(shaftSpeed, torque float64) error {
	if shaftSpeed > s.motor.MaxShaftSpeed {
		return errors.New("Motor Angular Speed")
	}
	
	if torque > s.motor.Peak.Torque {
		return errors.New("Motor Torque")
	}
	
	power := shaftSpeed * torque
	if torque > s.motor.Peak.Power {
		return errors.New("Motor Power")
	}
	
	if torque > s.motor.Continuous.Torque || power > s.motor.Continuous.Power {
		if s.TimeAtPeak > (time.Second * 20) {
			return errors.New("Motor Temperature")
		}
	}

	return nil
}

func (s *motorState)Operate(torque, shaftSpeed float64, duration time.Duration) {
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
	
	s.timeAtPeak += time.Duration(float64(duration) * overload)
}
