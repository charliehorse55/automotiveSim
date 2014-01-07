package automotiveSim

import (
	"math"
	"fmt"
	"time"
	"errors"
)
	
const (
	ROLLING = iota
	ACCESORY
	INEFF
	ACCELERATION
	AERO
)

//human readable descriptions for each of the powerdraw causes
var PowerDrawSources []string = []string{"Rolling Resistance", "Accessory", "Inefficiencies", "Acceleration", "Aerodynamic",}

//length of this array must be equal to number of elements in the string array above.
// no way that I am aware of to do this automatically in golang
type VehiclePowerUse [5]float64



type SimulatorState struct {
    Vehicle *Vehicle
    
    Time time.Duration
    Speed float64
    Distance float64
    Coulombs float64
    Interval time.Duration
    PowerUse VehiclePowerUse
}

func InitSimulation(vehicle *Vehicle) (*SimulatorState, error) {
    var state SimulatorState
    state.Vehicle = vehicle
    state.Interval = 1 * time.Millisecond; // 1ms default interval
	if state.findOperatingPoint(1) <= 0 {
		return nil, fmt.Errorf("Vehicle can not move")
	}
	
    return &state, nil
}

// the total power (as drawn from the main bus) required to produce a force f from the motors at the wheels
func (state *SimulatorState)powerAtMotorForce(f float64) float64 {
	vehicle := state.Vehicle
	power := f * state.Speed
	
	if f < 0 {
		//regen energy flows in the opposite direction, so the losses reduce the amount of energy reclaimed
		power *= vehicle.ElectricalEff * vehicle.DrivetrainEff
	} else {
		//when accelerating, losses just increase the amount of power drawn from the battery
		power /= vehicle.ElectricalEff * vehicle.DrivetrainEff
	}
    power += vehicle.Accessory
	return power
}

func (state *SimulatorState)CanOperateAtPoint(accel float64) (bool, error) {
    vehicle := state.Vehicle
	
	//check that the tires can support the force put onto them
	tireForce := (accel * vehicle.Weight) + vehicle.AeroDrag(state.Speed)
	if math.Abs(tireForce) > (vehicle.Tires.Friction * 9.81 * vehicle.Weight) {
		return false, errors.New("Tire friction")
	}
	
	//check that the motors can provide the required torque (or we are braking)
	//not Abs of motorForce as negative values indicate regen
	//any extra needed braking force can be added by the actual brakes
	motorForce := tireForce + vehicle.RollingDrag(state.Speed)
	if motorForce > vehicle.PeakForce(state.Speed) {
		return false, errors.New("Motor Torque")
	}
	
	//calculate battery power draw
	power := state.powerAtMotorForce(motorForce)
	
	//if the battery can't handle maximum regen, the brakes can pick up the slack
	if math.Abs(power) > vehicle.Battery.maxPower() && motorForce > 0 {
		state.limitingFactor = "Battery Power"
		return false
	}
	
	return true
}

func (state *SimulatorState)findOperatingPoint(targetAccel float64) float64 {
	state.limitingFactor = "No limiting factor"
	if state.CanOperateAtPoint(targetAccel) {
		return targetAccel
	}
	
	// TODO fix for cases where CanOperateAtPoint(0) == false
	guess := targetAccel/2
	step := targetAccel/4
	lastKnownGood := 0.0
	for step > 0.001 {
		if state.CanOperateAtPoint(guess) {
			lastKnownGood = guess
			guess += step
		} else {
			guess -= step
		}
		step /= 2
	}
	return lastKnownGood
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
    state.Coulombs += amps * interval
    
    // attribute the power loss to each component
    state.PowerUse[AERO] = aeroForce*state.Speed
    state.PowerUse[ROLLING] = rollingForce*state.Speed
  	state.PowerUse[ACCESORY] = vehicle.Accessory
    state.PowerUse[ACCELERATION] = accelForce*state.Speed
	state.PowerUse[INEFF] = math.Abs((amps * vehicle.Battery.NominalVoltage) - idealPower)
	
    state.Distance += state.Speed * interval
    state.Speed += accel * interval
    state.Time += state.Interval
        
    return accel, err
}

func (state *SimulatorState)TotalPowerUse() (total float64) {
	for _,power := range state.PowerUse {
		total += power
	}
	return
}






