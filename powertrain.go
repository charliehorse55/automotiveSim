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
	WeightDistribution float64 //amount of weight supported by this drives tire(s)
	Tires Tire
}

type driveState struct {
	drive *Drive
	motor *motorState 
}

type Body struct {
	Drives []Drive
    Weight float64
    CdA float64
}

type bodyState struct {
	body *Body
	drives []driveState
}

func (b *Body)Init() error {
	if b.Weight < 0 {
		return fmt.Errorf("Vehicle must not have negative weight")
	}
	
	if b.CdA < 0 {
		return fmt.Errorf("Vehicle must not have negative drag area")
	}

	if len(b.Drives) == 0 {
		return fmt.Errorf("Vehicle requires at least one motor")
	}
	
	for _,drive := range b.Drives {
		err := drive.Motor.Init()
		if err != nil {
			return err
		}
		
		err = drive.Tires.Init()
		if err != nil {
			return err
		}
			
		if drive.Gearing == 0 {
			return fmt.Errorf("Drive gearing must not be zero")
		}
		
		if drive.Efficiency <= 0 || drive.Efficiency > 1 {
			return fmt.Errorf("Mechanical drive efficiency must be on the range (0,1]")
		}
	}
		
	return nil
}

func NewBodyState(b *Body) *bodyState {
	state := &bodyState{body:b}
	state.drives = make([]driveState, len(b.Drives))
	for i := range b.Drives {
		state.drives[i] = driveState{drive:&b.Drives[i], motor:NewMotorState(&b.Drives[i].Motor)}
	}
	return state
}

func (b *bodyState)CanOperate(accel float64, duration time.Duration) (float64, error) {
	
}

func (b *bodyState)Operate(accel float64, duration time.Duration) (map[string]float64) {

}

func (b *bodyState)MaxForce(vehicleSpeed float64) float64 {
	total := 0.0
	for _,d := range b.drives {
		shaftSpeed := (vehicleSpeed / (2 * math.Pi * d.drive.Tires.Radius)) * d.drive.Gearing
		supportedWeight := d.drive.WeightDistribution * b.body.Weight
		grip := d.drive.Tires.Grip * d.drive.WeightDistribution * b.body.Weight
		force := (d.motor.MaxTorque(shaftSpeed) * d.drive.Efficiency)
		if math.Abs(force) > grip {
			force = math.Copysign(grip, force)
		}
		total += force
	}
	return total
}

func (b *bodyState)RollingDrag() float64 {
	total := 0.0
	for _,d := range b.drives {
		supportedWeight := d.drive.WeightDistribution * b.body.Weight
		total += supportedWeight * gravity * d.drive.Tires.RollingResistance
	}
	return total
}


