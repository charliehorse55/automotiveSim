package automotiveSim

import (
	"math"
	"fmt"
	"time"
)
	
const (
	gravity = 9.81
)

type PowerUse map[string]float64

type SimulatorState struct {
    Vehicle *Vehicle
    battery *batteryState
	body *bodyState
	
    Time time.Duration
    Speed float64
    Distance float64
    Interval time.Duration
    PowerUses PowerUse
	BusVoltage float64
}

func InitSimulation(vehicle *Vehicle) (*SimulatorState, error) {
    var state SimulatorState
    state.Vehicle = vehicle
	
	//initialize the states
	state.battery = NewBatteryState(&vehicle.Battery)
	state.body = NewBodyState(&vehicle.Body)

	// 1ms default interval 
    state.Interval = 1 * time.Millisecond;
	
	//initialize the map of power use
	state.PowerUses = make(PowerUse)
	
	//check that the vehicle isn't totally broken
	accel, err := state.findOperatingPoint(1)
	if accel <= 0 {
		return nil, fmt.Errorf("Vehicle can not move: %v", err)
	}
	
    return &state, nil
}

func (state *SimulatorState)CanOperateAtPoint(accel float64) error {
    vehicle := state.Vehicle
	
	powerUse := 0.0
	
	tractionPower, err := state.body.CanOperate(accel, state.Interval)
	if err != nil {
		return err
	}
	powerUse += tractionPower
	powerUse += vehicle.Accessory
		
	
		
	return true
}

func (state *SimulatorState)OperateAtPoint(accel float64) {
	
}

func (state *SimulatorState)findOperatingPoint(targetAccel float64) (float64, error) {
	if state.CanOperateAtPoint(targetAccel) {
		return targetAccel, nil
	}
	
	// TODO fix for cases where CanOperateAtPoint(0) == false
	guess := targetAccel/2
	step := targetAccel/4
	lastKnownGood := 0.0
	var lastErr error
	for step > 0.001 {
		err := state.CanOperateAtPoint(guess) 
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
    vehicle := state.Vehicle
    interval := state.Interval.Seconds()
		 
	accel, err := state.findOperatingPoint(targetAccel)
	
	rollingForce := vehicle.RollingDrag(state.Speed)
	aeroForce 	 :=	vehicle.AeroDrag(state.Speed)
	accelForce   := accel * vehicle.Weight
		            
    //calculate battery draw
	force := aeroForce + rollingForce + accelForce
	idealPower := (force * state.Speed) + vehicle.Accessory
	power := state.powerAtMotorForce(force)
    amps := vehicle.Battery.ampsAtPower(math.Abs(power))
    	
    state.Distance += state.Speed * interval
    state.Speed += accel * interval
    state.Time += state.Interval
        
    return accel, err
}

func (state *SimulatorState)TotalPowerUse() (total float64) {
	total := 0.0
	for _,power := range state.PowerUse {
		total += power
	}
	return total
}






