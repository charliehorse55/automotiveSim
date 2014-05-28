package automotiveSim

import (
	"fmt"
	"time"
)
	
const (
	gravity = 9.81
)

type SimulatorState struct {
    Vehicle *Vehicle
    battery *batteryState
	body *bodyState
	
	Time time.Duration
    Speed float64
    Distance float64
    Interval time.Duration
	Power Power
	BusVoltage float64
}

func InitSimulation(vehicle *Vehicle) (*SimulatorState, error) {
    var state SimulatorState
    state.Vehicle = vehicle
	
	//initialize the map of power use
	state.Power = make(Power)
	
	state.battery = NewBatteryState(&vehicle.Battery)
	state.BusVoltage = state.battery.pack.NominalVoltage
	state.Power["Battery"] = state.battery.power
	
	state.body = NewBodyState(&vehicle.Body)
	state.Power["Body"] = state.body.power

	//1ms default interval 
    state.Interval = 1 * time.Millisecond	
		
	//check that the vehicle can actually move
	accel, err := state.FindOperatingPoint(1)
	if accel <= 0 {
		return nil, fmt.Errorf("Vehicle can not move: %v", err)
	}
	
    return &state, nil
}

func (state *SimulatorState)CanOperate(accel float64) error {
    vehicle := state.Vehicle
	
	powerUse := 0.0
	
	tractionPower, err := state.body.CanOperate(state, accel, state.Interval)
	if err != nil {
		return err
	}
	powerUse += tractionPower
	powerUse += vehicle.Accessory
	
	err = state.battery.CanOperate(state, powerUse, state.Interval)
	if err != nil {
		return err
	}
		
	return nil
}

func (state *SimulatorState)Operate(accel float64) {
	power := state.body.Operate(state, accel, state.Interval)
	power += state.Vehicle.Accessory
	state.Power["Accessory"] = state.Vehicle.Accessory
	
	state.BusVoltage = state.battery.Operate(state, power, state.Interval)
	
	
	interval := state.Interval.Seconds()
    state.Distance += state.Speed * interval
    state.Speed += accel * interval
    state.Time += state.Interval
}

func (state *SimulatorState)FindOperatingPoint(targetAccel float64) (float64, error) {
	err := state.CanOperate(targetAccel) 
	if err == nil {
		return targetAccel, nil
	}
	
	// TODO fix for cases where CanOperateAtPoint(0) == false
	guess := targetAccel/2
	step := targetAccel/4
	lastKnownGood := 0.0
	var lastErr error
	for step > 0.001 {
		err := state.CanOperate(guess) 
		if err != nil {
			guess -= step
			lastErr = err
		} else {
			lastKnownGood = guess
			guess += step
		}
		step /= 2
	}
	return lastKnownGood, lastErr
}

func (state *SimulatorState)Tick(targetAccel float64) (float64, error) {    
	accel, err := state.FindOperatingPoint(targetAccel)
	state.Operate(accel)
	return accel, err
}







