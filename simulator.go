package automotiveSim

import "math"

const (
	AERO = iota
	ROLLING
	ACCESORY
	ACCELERATION
)

//human readable descriptions for each of the powerdraw causes
var PowerDrawSources []string = []string{"Aerodynamic", "Rolling Resistance", "Accessory", "Acceleration"}

//length of this array must be equal to number of elements in the string array above.
// no way that I am aware of to do this automatically in golang
type VehiclePowerUse [4]float64

type SimulatorState struct {
    Vehicle *Vehicle
    
    Time float64
    Speed float64
    Distance float64
    Coulombs float64
    Interval float64
    Acceleration float64
    PowerUse VehiclePowerUse
    topMotorSpeed float64
}


func InitSimulation(vehicle *Vehicle) *SimulatorState {
    var state SimulatorState
    state.Vehicle = vehicle
    state.Interval = 0.001; // 1ms default interval
    state.topMotorSpeed = vehicle.MotorShaftSpeedLimit()
    return &state
}

func (state *SimulatorState)CanOperateAtPoint(accel float64) bool {
    vehicle := state.Vehicle
    	
	//make sure we are not accelerating over the max motor RPM
    if state.Speed + (accel * state.Interval) > state.topMotorSpeed {
        return false
    }
	
	//check that the tires can support the force put onto them
	tireForce := (accel * vehicle.Weight) + vehicle.AeroDrag(state.Speed)
	if math.Abs(tireForce) > (vehicle.Tires.Friction * 9.81 * vehicle.Weight) {
		return false
	}
	
	//check that the motors can provide the required torque
	motorForce := tireForce + vehicle.RollingDrag(state.Speed)
	if motorForce > vehicle.PeakForce(state.Speed) {
		return false
	}
	
	//calculate battery power draw
    power := (motorForce * state.Speed)/(vehicle.ElectricalEff * vehicle.DrivetrainEff)
    power += vehicle.Accessory
	
	if power > vehicle.Battery.maxPower() {
		return false
	}
	
	return true
}

func (state *SimulatorState)findOperatingPoint(targetAccel float64) float64 {
	if state.CanOperateAtPoint(targetAccel) {
		return targetAccel
	}
	
	guess := targetAccel/2
	step := targetAccel/4
	for {
		if state.CanOperateAtPoint(guess) {
			if step < 0.01 {
				return guess
			}
			guess += step
		} else {
			guess -= step
		}
		step /= 2
	}
}

func (state *SimulatorState)Tick(targetAccel float64) {    
    vehicle := state.Vehicle
         
	//find operating point here
	accel := state.findOperatingPoint(targetAccel)
	    
	rollingForce := vehicle.RollingDrag(state.Speed)
	aeroForce 	 :=	vehicle.AeroDrag(state.Speed)
	accelForce   := accel * vehicle.Weight
		
    // calculate the source of the power loss
    state.PowerUse[AERO] = aeroForce*state.Speed
    state.PowerUse[ROLLING] = rollingForce*state.Speed
    state.PowerUse[ACCESORY] = vehicle.Accessory
    state.PowerUse[ACCELERATION] = accelForce*state.Speed
            
    //calculate battery draw
	force := aeroForce + rollingForce + accelForce
    power := (force * state.Speed)/(vehicle.ElectricalEff * vehicle.DrivetrainEff)
    power += vehicle.Accessory
    amps := vehicle.Battery.ampsAtPower(power)
    state.Coulombs += amps * state.Interval
    
    state.Distance += state.Speed * state.Interval
    state.Speed += accel * state.Interval
    state.Time += state.Interval
    
    state.Acceleration = accel
    
    return
}

func (state *SimulatorState)TotalPowerUse() (total float64) {
	for _,power := range state.PowerUse {
		total += power
	}
	return
}






