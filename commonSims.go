package automotiveSim

import "math"
import "fmt"

type SimulatorRun struct {
    Interval float64
    Speeds []float64
}

func (state *SimulatorState)energyUsed() float64 {
    return state.Coulombs * state.Vehicle.Battery.NominalVoltage
}

func (vehicle *Vehicle)RunInput(input *SimulatorRun) ([]float64, error) {
    sim := InitSimulation(vehicle)
    
	numTicks := int(input.Interval/sim.Interval)
	Power := make([]float64, len(input.Speeds))
    for i,newSpeed := range input.Speeds {
        accel := (newSpeed - sim.Speed)/input.Interval
		var powerUsage float64
        for i := 0; i < numTicks; i++ {
            sim.Tick(accel);
            if math.Abs(sim.Acceleration - accel) > 0.01 {
				return nil, fmt.Errorf("Vehicle failed to accelerate at %5.2fm/s (only %5.2f)", accel, sim.Acceleration)
            }
			if sim.Coulombs > vehicle.Battery.Coulomb {
				return nil, fmt.Errorf("Battery depleted before test completion")
			}
			powerUsage += sim.TotalPowerUse()
        }
		Power[i] = powerUsage/float64(numTicks)
    }
    return Power, nil
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




