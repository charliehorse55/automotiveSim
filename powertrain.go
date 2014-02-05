package automotiveSim


import (
	"time"
	"fmt"
)


type Drive struct {
	Motor Motor
	Gearing float64
	Efficiency float64
	Tires Tire
}

type driveState struct {
	drive *Drive
	motor *motorState 
}

type Powertrain struct {
	Drives []Drive
}

type powertrainState struct {
	drives []driveState
}

func (p *Powertrain)Init() error {
	if len(p.Drives) == 0 {
		return fmt.Errorf("Vehicle requires at least one motor")
	}
	
	for _,drive := range p.Drives {
		err := drive.Motor.Init()
		if err != nil {
			return err
		}
		err := drive.Tires.Init()
		if err != nil {
			return err
		}
		
		if drive.Gearing == 0 {
			return fmt.Errorf("Drive gearing must not be zero")
		}
	}
	
	return nil
}

func NewPowertrainState(p *Powertrain) *powertrainState {
	drives := make([]driveState, len(p.Drives))
	for i := range p.Drives {
		drives[i] = driveState{drive:p.Drives[i], NewMotorState(p.Drives[i])}
	}
	return &powertrainState{drives:drives}
}

func (p *powertrainState)CanOperate(vehicleSpeed, force, busVoltage float64, duration time.Duration) error {
	
}

// just calculate power required by an approximation based on a single efficiency value for now
func (p *powertrainState)PowerAtTorque(vehicleSpeed, force, duration time.Duration) float64 {
	
}

func (p *powertrainState)Operate(vehicleSpeed, force, busVoltage float64, duration time.Duration) {
	
}
