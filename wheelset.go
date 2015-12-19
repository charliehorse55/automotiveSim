package automotiveSim


import (
	"time"
	"fmt"
	"math"
)

func (w *Wheelset)Fmax(sim *SimulatorState) (float64, error) {
	maxF := 0.0
	limit := nil
	if(w.Drive != nil) {
		shaftRatio := w.Drive.Gearing/w.Drive.Tires.Radius
		
		maxTorque := 0.0
		//careful not to use := here and redefine limit (and why we define maxTorque above)
		maxTorque, limit = w.Drive.Motor.MaxTorque(sim, sim.Speed * shaftRatio)
		maxF += maxTorque * w.Drive.Efficiency * shaftRatio
	} else {
		limit = fmt.Errorf("Freewheel")
	}
	
	forceOnWheel := w.WeightDistribution * sim.Body.Weight * gravity
	maxF -= forceOnWheel * w.Drive.Tires.RollingResistance
	
	tireGrip := forceOnWheel * w.Drive.Tires.Grip
	
	if(math.Abs(maxF) > tireGrip) {
		return math.Copysign(tireGrip, maxF), fmt.Errorf("Tire grip")
	}
	return maxF, limit
}

func (w *Wheelset)Fmin(sim *SimulatorState) (float64, error) {
	
}

func (w *Wheelset)CanOperate(sim *SimulatorState, force float64) (float64, error) {

	//tire grip
	for _,d := range s.drives {
		grip := d.drive.Tires.Grip * d.drive.WeightDistribution * s.body.Weight * gravity
		if splitForce > grip {
			return 0, fmt.Errorf("Tire grip on %s", d.drive.Name)
		}
	}
	
	//motors need to overcome rolling resistance as well
	totalForce += s.RollingDrag(sim)
	
	splitForce = totalForce / float64(len(s.drives))
	totalPower := 0.0
	for _,d := range s.drives {
		//motor calculations
		shaftTorque := (splitForce * d.drive.Tires.Radius) / (d.drive.Efficiency * d.drive.Gearing)
		motorPower, err := d.motor.CanOperate(sim, shaftSpeed, shaftTorque, duration)
		if err != nil {
			return 0, fmt.Errorf("%s: %s: %v", d.drive.Name, d.drive.Motor.Name, err)	
		}
		totalPower += motorPower
	}
	return totalPower, nil
}

func (w *Wheelset)Operate(sim *SimulatorState, force float64) (float64) {
	totalForce := s.body.Weight * accel
	totalForce += s.RollingDrag(sim)
	totalForce += s.AeroDrag(sim)
	//TODO optimize torque split between drives
	splitForce := totalForce / float64(len(s.drives))
	totalPower := 0.0
	for _,d := range s.drives {
		//motor calculations
		shaftSpeed := (sim.Speed / d.drive.Tires.Radius) * d.drive.Gearing
		shaftTorque := (splitForce * d.drive.Tires.Radius) / (d.drive.Gearing)
		if shaftSpeed != 0 {
			shaftTorque = et(shaftTorque*shaftSpeed, d.drive.Efficiency)/shaftSpeed
		}
		loss := math.Abs(shaftSpeed * shaftTorque) * (1 - d.drive.Efficiency)
		motorPower := d.motor.Operate(sim, shaftSpeed, shaftTorque, duration)
		totalPower += motorPower
		d.power["Gear friction"] = loss
	}
	return totalPower
}

func (s *bodyState)RollingDrag(sim *SimulatorState) float64 {
	total := 0.0
	for _,d := range s.drives {
		supportedWeight := d.drive.WeightDistribution * s.body.Weight
		total += supportedWeight * gravity * d.drive.Tires.RollingResistance
	}
	
	//add the freewheeling resistance
	for _,f := range s.body.Freewheels {
		supportedWeight := f.WeightDistribution * s.body.Weight
		total += supportedWeight * gravity * f.Tires.RollingResistance
	}
	
	return total
}

func (s *bodyState)AeroDrag(sim *SimulatorState) float64 {
    return 0.5 * s.body.CdA * sim.Speed * sim.Speed * airDensity(sim.Vehicle.Ambient.Temperature, sim.Vehicle.Ambient.Pressure) //aero
}



