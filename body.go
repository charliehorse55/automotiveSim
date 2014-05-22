package automotiveSim


import (
	"time"
	"fmt"
	"math"
)

type Drive struct {
	Name string
	Motor Motor
	Gearing float64
	Efficiency float64
	WeightDistribution float64 //percentage of weight supported by this drives tire(s)
	Tires Tire
}

type driveState struct {
	drive *Drive
	power Power
	motor *motorState 
}

type Body struct {
	Drives []Drive
	Freewheels []Drive
    Weight float64
    CdA float64
}

type bodyState struct {
	power Power
	body *Body
	drives []*driveState
}

func (b *Body)Init() error {
	if b.Weight <= 0 {
		return fmt.Errorf("Vehicle must have positive weight")
	}
	
	if b.CdA < 0 {
		return fmt.Errorf("Vehicle must not have negative drag area")
	}

	if len(b.Drives) == 0 {
		return fmt.Errorf("Vehicle requires at least one driven wheelset")
	}
	
	totalWeightDist := 0.0
	weight, err := initDrives(b.Drives, true)
	if err != nil {
		return err
	}
	totalWeightDist += weight
	
	weight, err = initDrives(b.Freewheels, false)
	if err != nil {
		return err
	}
	totalWeightDist += weight
	
	if math.Abs(totalWeightDist - 1.0) > 0.0001 {
		return fmt.Errorf("Vehicle weight distribution does not sum to 1.0 (%6.4f)", totalWeightDist)
	}
		
	return nil
}

func initDrives(drives []Drive, driven bool) (float64, error) {
	totalWeightDist := 0.0
	for _,drive := range drives {
		if driven {
			err := drive.Motor.Init()
			if err != nil {
				return 0.0, fmt.Errorf("%s: %v", drive.Motor.Name, err)
			}
			if drive.Gearing == 0 {
				return 0.0, fmt.Errorf("%s: gearing must not be zero", drive.Name)
			}
		
			if drive.Efficiency <= 0 || drive.Efficiency > 1 {
				return 0.0, fmt.Errorf("%s: Mechanical drive efficiency must be on the range (0,1]", drive.Name)
			}
		}
		
		err := drive.Tires.Init()
		if err != nil {
			return 0.0, err
		}
		
		if drive.WeightDistribution <= 0 || drive.WeightDistribution > 1 {
			return 0.0, fmt.Errorf("%s: Mechanical efficiency must be on the range (0,1]", drive.Name)
		}
		totalWeightDist += drive.WeightDistribution
	}
	return totalWeightDist, nil
}

func NewDriveState(d *Drive) *driveState {
	state := &driveState{drive:d, power:make(Power), motor:NewMotorState(&d.Motor)}
	state.power[d.Motor.Name] = state.motor.power
	return state
}

func NewBodyState(b *Body) *bodyState {
	state := &bodyState{body:b}
	state.power = make(Power)
	state.drives = make([]*driveState, len(b.Drives))
	for i := range b.Drives {
		state.drives[i] = NewDriveState(&b.Drives[i])
		state.power[b.Drives[i].Name] = state.drives[i].power
	}
	return state
}

func (s *bodyState)CanOperate(sim *SimulatorState, accel float64, duration time.Duration) (float64, error) {
	totalForce := s.body.Weight * accel
	totalForce += s.AeroDrag(sim)
	splitForce := totalForce / float64(len(s.drives))
	
	//TODO optimize torque split between drives

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
		shaftSpeed := (sim.Speed / d.drive.Tires.Radius) * d.drive.Gearing
		shaftTorque := (splitForce * d.drive.Tires.Radius) / (d.drive.Efficiency * d.drive.Gearing)
		motorPower, err := d.motor.CanOperate(sim, shaftSpeed, shaftTorque, duration)
		if err != nil {
			return 0, fmt.Errorf("%s: %s: %v", d.drive.Name, d.drive.Motor.Name, err)	
		}
		totalPower += motorPower
	}
	return totalPower, nil
}

func (s *bodyState)Operate(sim *SimulatorState, accel float64, duration time.Duration) (float64) {
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
		shaftTorque = et(shaftTorque, d.drive.Efficiency)
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
    return 0.5 * s.body.CdA * sim.Speed * sim.Speed * airDensity(sim.ExternalTemp) //aero
}



