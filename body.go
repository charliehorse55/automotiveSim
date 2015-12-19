package automotiveSim


import (
	"time"
	"fmt"
	"math"
)

type Wheelset struct {
	Name string
	Drive *Drive
	WeightDistribution float64 //percentage of weight supported by this drives tire(s)
	Tires Tire
}

type Drive struct {
	Motor Motor
	Gearing float64
	Efficiency float64
}

type Body struct {
	Wheelsets []Wheelset
    Weight float64
    CdA float64
}

func (b *Body)Init() error {
	if b.Weight <= 0 {
		return fmt.Errorf("Vehicle must have positive weight")
	}
	
	if b.CdA < 0 {
		return fmt.Errorf("Vehicle must not have negative drag area")
	}
	
	totalWeightDist := 0.0
	drivenCount := 0
	for _,w := range b.Wheelsets {
		if w.Drive != nil {
			err := w.Drive.Motor.Init()
			if err != nil {
				return fmt.Errorf("%s: %v", w.Drive.Motor.Name, err)
			}
			if w.Drive.Gearing == 0 {
				return fmt.Errorf("%s: gearing must not be zero", w.Name)
			}
			if w.Drive.Efficiency <= 0 || w.Drive.Efficiency > 1 {
				return fmt.Errorf("%s: Mechanical drive efficiency must be on the range (0,1]", w.Name)
			}
			drivenCount++
		}
		
		err := w.Tires.Init()
		if err != nil {
			return err
		}
		totalWeightDist += drive.WeightDistribution
	}
	if drivenCount == 0 {
		return fmt.Errorf("Vehicle requires at least one driven wheelset")
	}
		
	if math.Abs(totalWeightDist - 1.0) >= 0.0001 {
		return fmt.Errorf("Vehicle weight distribution does not sum to 1.0 (%6.4f)", totalWeightDist)
	}
	return nil
}

func (b *Body)findWheelsetForces(sim *SimulatorState, accel float64) ([]float64, error) {
	//first find the total force required by the rest of the car
	totalForce := s.body.Weight * accel
	totalForce += s.AeroDrag(sim)
		
	//find the total range of force the wheelsets are collectively able to produce
	totalFmax := 0.0
	FmaxLimits := make([]error, len(b.Wheelsets))
	Fmax := make([]float64, len(b.Wheelsets))
	totalFmin := 0.0
	FminLimits := make([]error, len(b.Wheelsets))
	Fmin := make([]float64, len(b.Wheelsets))
	for i,w := range b.Wheelsets {
		Fmax[i], FmaxLimits[i] := w.Fmax(sim)
		maxAvailableF += Fmax[i]
		Fmin[i], FminLimits[i] := w.Fmin(sim)
		minAvailableF += Fmin[i]
	}
	
	if(totalForce < totalFmin) {
		errorStr := ""
		for _,reason := range FminLimits {
			errorStr += reason + "\n"
		}
		return nil, fmt.Errorf("Min wheelset force:\n%s", errorStr)
	} else if (totalForce > totalFmax) {
		errorStr := ""
		for _,reason := range FmaxLimits {
			errorStr += reason + "\n"
		}
		return nil, fmt.Errorf("Max wheelset force:\n%s", errorStr)
	}
	
	//for now, balance the torque from each wheelset by driving them at the same % of their max
	throttle := (totalForce - totalFmin)/(totalFmax - totalFmin)
	
	//reuse Fmax array for final output torques
	for _,w := range b.Wheelsets {
		Fmax[i] := (throttle * (Fmax[i] - Fmin[i])) + Fmin[i]
	}
	return Fmax, nil
}

func (b *Body)CanOperate(sim *SimulatorState, accel float64) (float64, error) {
	forces, err := b.findWheelsetForces(sim, accel)
	if err != nil {
		return 0, err
	}
	
	totalPower := 0.0
	for i,w := range b.Wheelsets {
		power, err := w.CanOperate(sim, forces[i])
		if err != nil {
			return 0, fmt.Errorf("%s: %v", w.Name, err)	
		}
		totalPower += power
	}
	return totalPower, nil
}

func (b *Body)Operate(sim *SimulatorState, accel float64) float64 {
	forces, err := b.findWheelsetForces(sim, accel)
	if err != nil {
		panic("Got body error on operate")
	}
	
	totalPower := 0.0
	for i,w := range b.Wheelsets {
		totalPower += w.Operate(sim, forces[i])
	}
	return totalPower
}

func (b *Body)RollingDrag(sim *SimulatorState) float64 {
	total := 0.0
	for _,w := range b.Wheelsets {
		supportedWeight := w.WeightDistribution * b.Weight
		total += supportedWeight * gravity * w.Tires.RollingResistance
	}	
	return total
}

func (b *Body)AeroDrag(sim *SimulatorState) float64 {
    return 0.5 * b.CdA * sim.Speed * sim.Speed * airDensity(sim.Vehicle.Ambient.Temperature, sim.Vehicle.Ambient.Pressure)
}



