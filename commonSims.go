package automotiveSim

import "math"
import "fmt"

const quarterMile = 402.33600

type Schedule struct {
	Name string
    Interval float64
    Speeds []float64
}

func (state *SimulatorState)energyUsed() float64 {
    return state.Coulombs * state.Vehicle.Battery.NominalVoltage
}

type ScheduleResult struct {
	Power []float64
	Distance float64
	
}

func (input *Schedule)Run(vehicle *Vehicle) (*ScheduleResult, error) {
    sim := InitSimulation(vehicle)
    
	numTicks := int(input.Interval/sim.Interval)
	var result ScheduleResult
	result.Power = make([]float64, len(input.Speeds))
    for i,newSpeed := range input.Speeds {
        accel := (newSpeed - sim.Speed)/input.Interval
		var powerUsage float64
        for i := 0; i < numTicks; i++ {
            sim.Tick(accel);
            if math.Abs(sim.Acceleration - accel) > 0.01 {
				return nil, fmt.Errorf("Vehicle failed to accelerate at %5.2fm/s (only %5.2f)", accel, sim.Acceleration)
            }
			if sim.Coulombs > vehicle.Battery.Coulomb {
				return nil, fmt.Errorf("Battery depleted before end of test")
			}
			powerUsage += sim.TotalPowerUse()
        }
		result.Power[i] = powerUsage/float64(numTicks)
    }
	result.Distance = sim.Distance
    return &result, nil
}

type AccelProfile struct {
	TopSpeed float64
	Accel100 float64
	QuarterMile float64
	AccelTop float64
	PeakAccel float64
}

func (vehicle *Vehicle)RunAccelerationProfile() (AccelProfile, error) {
	sim := InitSimulation(vehicle)
	
	var result AccelProfile
	for result.TopSpeed == 0 || result.QuarterMile == 0 {
		sim.Tick(10000)
		if sim.Acceleration > result.PeakAccel {
			result.PeakAccel = sim.Acceleration
		}
		
		if sim.Speed > (100/3.6) && result.Accel100 == 0 {
			result.Accel100 = sim.Time
		}
		
		if sim.Distance > quarterMile && result.QuarterMile == 0 {
			result.QuarterMile = sim.Time
		}
		
		//have we hit topspeed
		if sim.Acceleration < 0.01  && result.TopSpeed == 0{
			result.TopSpeed = sim.Speed
			result.AccelTop = sim.Time
			if sim.Speed < (100/3.6) {
				result.Accel100 = math.NaN()
			}
		}
	}
	return result, nil
}

type PowerDraw struct {
	Sources []string
	Magnitude []VehiclePowerUse
}

func (vehicle *Vehicle)PowerAtSpeeds(speeds []float64) (*PowerDraw, error) {
    sim := InitSimulation(vehicle)
	
	var power PowerDraw
	power.Sources = PowerDrawSources
	power.Magnitude = make([]VehiclePowerUse, len(speeds))
	for i,speed := range speeds {
		sim.Speed = speed
		sim.Tick(0)
		if math.Abs(sim.Acceleration) > 0.01 {
			return nil, fmt.Errorf("Vehicle can not maintain speed %5.2f", speed)
		} 
		power.Magnitude[i] = sim.PowerUse
	}
	return &power, nil
}




