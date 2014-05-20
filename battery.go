package automotiveSim


import (
	"time"
	"math"
	"fmt"
)

type BatteryPack struct {
    NominalVoltage float64
    InternalResistance float64
    Coulomb float64
	MaxCurrent float64
    Series uint
    Parallel uint
    CellVoltage float64
    CellResistance float64
    CellCoulomb float64
	CellMaxCurrent float64
}

type batteryState struct {
	pack *BatteryPack 
	
	power Power
	coulombsUsed float64
}


func (b *BatteryPack)Init() error {
	if b.CellCoulomb <= 0 {
		return fmt.Errorf("Battery cells must store a positive amount of charge")
	}
	
	if b.CellResistance < 0 {
		return fmt.Errorf("Battery cells can not have negative resistance")
	}
	
	if b.CellVoltage <= 0 {
		return fmt.Errorf("Battery cells must have positive nominal voltage")
	}
	
	if b.CellMaxCurrent <= 0 {
		return fmt.Errorf("Battery cells must have positive maximum current")
	}
	
	if b.Series == 0 {
		return fmt.Errorf("Battery pack must have at least one cell in series")
	}	
	
	if b.Parallel == 0 {
		return fmt.Errorf("Battery pack must have at least one cell in parallel")
	}
	
    b.NominalVoltage = b.CellVoltage * float64(b.Series)
    b.Coulomb = b.CellCoulomb * float64(b.Parallel)
    b.InternalResistance = (b.CellResistance*float64(b.Series)) / float64(b.Parallel)
	b.MaxCurrent = b.CellMaxCurrent * float64(b.Parallel)
	return nil
}

func NewBatteryState(b *BatteryPack) *batteryState {
	return &batteryState{pack:b, power:make(Power)}
}

func (s *batteryState)CanOperate(sim *SimulatorState, power float64, duration time.Duration) error {
	amp := s.AmpsAtPower(power)
	if math.IsNaN(amp) || math.Abs(amp) > s.pack.MaxCurrent {
		return fmt.Errorf("Exceeds max pack current")
	}
	coulomb := amp * duration.Seconds()
	if (coulomb + s.coulombsUsed) > s.pack.Coulomb {
		return fmt.Errorf("Battery Energy depleted")
	}
	
	return nil
}


func (s *batteryState)Operate(sim *SimulatorState, power float64, duration time.Duration) float64 {
	amp := s.AmpsAtPower(power)
	s.coulombsUsed += amp * duration.Seconds()
	s.power["Internal Resistance"] = (amp*s.pack.NominalVoltage) - power
	return power/amp
}

func (s *batteryState)VoltageAtPower(power float64) float64  {
	diff := math.Sqrt(s.pack.NominalVoltage*s.pack.NominalVoltage - 4*power*s.pack.InternalResistance)
	return (s.pack.NominalVoltage + diff)/2
}

func (s *batteryState)AmpsAtPower(power float64) float64 {
	return power/s.VoltageAtPower(power)
}




