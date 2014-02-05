package automotiveSim

import (
	"math"
	"fmt"
	"time"
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
    battery *batteryState
	powertrain *powertrainState
	
    Time time.Duration
    Speed float64
    Distance float64
    Interval time.Duration
    PowerUse VehiclePowerUse
}

func InitSimulation(vehicle *Vehicle) (*SimulatorState, error) {
    var state SimulatorState
    state.Vehicle = vehicle
	
	//initialize the states
	state.battery = NewBatteryState(vehicle.Battery)
	state.powertrain = NewPowertrainState(vehicle.Powertrain)
	
	// 1ms default interval 
    state.Interval = 1 * time.Millisecond;
	
	//check that the vehicle isn't totally broken
	if state.findOperatingPoint(1) <= 0 {
		return nil, fmt.Errorf("Vehicle can not move")
	}
	
    return &state, nil
}

func (state *SimulatorState)CanOperateAtPoint(accel float64) error {
    vehicle := state.Vehicle
		
	//check that the tires can support the force put onto them
	tireForce := (accel * vehicle.Weight) + vehicle.AeroDrag(state.Speed)
	if math.Abs(tireForce) > (vehicle.Tires.Friction * 9.81 * vehicle.Weight) {
		return fmt.Errorf("Tire friction")
	}
	
	//the total force that is needed to be provided by the powertrain
	totalForce := tireForce + vehicle.RollingDrag(state.Speed)
	
	//use the total force to estimate power draw
	//the calculate the bus voltage by asking the battery about voltage sag given power
	powerEstimate := vehicle.Accessory + state.powertrain.PowerAtTorque(state.Speed, totalForce, state.Interval)
	
	
	
	
	motorForce := (totalForce * vehicle.Tires.Radius)/(vehicle.Gearing * vehicle.DrivetrainEff)
		
	//just split torque evenly for now
	shaftSpeed := vehicle.ShaftSpeed(state.Speed)
	for _,motor := range state.motors {
		err := motor.canOperate(shaftSpeed, motorForce/len(state.motors),  busVoltage, state.Interval)
		if err != nil {
			return err
		}
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

func (state *SimulatorState)OperateAtPoint(accel float64) {
	
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






