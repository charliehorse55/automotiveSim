package automotiveSim


import (
	"time"
	"fmt"
)

type Vehicle struct {
    Accessory float64
    Battery BatteryPack
	Body Body
	
	//environmental conditions TODO pull this into it's own data structure
    ExternalTemp float64
}


type driveState struct {
	drive *Drive
	motor *motorState 
}


func (v *Vehicle)Init() error {
	initFuncs := []func() error {
		v.Battery.Init,
 	}
	
	if len(v.Drives) == 0 {
		return fmt.Errorf("Vehicle requires at least one motor")
	}
	
	for _,drive := range v.Drives {
		initFuncs = append(initFuncs, drive.Motor.Init, drive.Tires.Init)
				
		if drive.Gearing == 0 {
			return fmt.Errorf("Drive gearing must not be zero")
		}
		
		if drive.Efficiency <= 0 || drive.Efficiency > 1 {
			return fmt.Errorf("Mechanical drive efficiency must be on the range (0,1]")
		}
	}
	
	for _,function := range initFuncs {
		err := function()
		if err != nil {
			return err
		}
	}
	
	return nil
}
