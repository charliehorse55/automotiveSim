package automotiveSim


import (
	"fmt"
)

type Tire struct {
    Grip float64
    RollingResistance float64
    Radius float64
}

func (t *Tire)Init() error {
	if t.Grip <= 0 {
		return fmt.Errorf("Tire grip must be positive")
	}
	if t.RollingResistance < 0 {
		return fmt.Errorf("Tire rolling resistance must not be negative")
	}
	if t.Radius <= 0 {
		return fmt.Errorf("Tire radius must be positive")
	}
	
	return nil
}
