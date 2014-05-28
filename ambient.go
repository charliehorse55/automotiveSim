package automotiveSim


import (
	"fmt"
)

type Ambient struct {
	Temperature float64
	Pressure float64
}

func (a *Ambient)Init() error {
	if a.Temperature <= 0 {
		return fmt.Errorf("Temperature must be above absolute zero")
	}
	if a.Pressure < 0 {
		return fmt.Errorf("Pressure can not be negative")
	}
	
	return nil
}
