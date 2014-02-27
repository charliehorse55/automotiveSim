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
	CellCRate float64
}

type batteryState struct {
	coulombsUsed float64
	pack *BatteryPack 	
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
	
	if b.CellCRate <= 0 {
		return fmt.Errorf("Battery cells must have positive C rate")
	}
	
	if b.Series == 0 {
		return fmt.Errorf("Battery pack must have at least one cell in series")
	}	
	if b.Parallel == 0 {
		return fmt.Errorf("Battery pack must have at least one cell in parallel")
	}
	
    b.NominalVoltage = b.CellVoltage * float64(b.Series)
    b.Coulomb = b.CellCoulomb * float64(b.Parallel)
    b.InternalResistance = b.CellResistance*float64(b.Series) / float64(b.Parallel)
	b.MaxCurrent = b.CellCRate * b.Series
	
	return nil
}

func NewBatteryState(b *BatteryPack) *batteryState {
	return &batteryState{pack:b}
}

func (b *batteryState)CanOperate(power float64, duration time.Duration) error {
	amp := b.AmpsAtPower(power)
	if math.IsNan(amp) || math.Abs(amp) > b.pack.MaxCurrent {
		return fmt.Errorf("Exceeds C Rate")
	}
	coulomb := amp * duration.Seconds()
	if (coulomb + b.coulombsUsed) > b.pack.Coulomb {
		return fmt.Errorf("Battery Energy depleted")
	}
	
	return nil
}


func (b *batteryState)Operate(power float64, duration time.Duration) float64 {
	amp := b.AmpsAtPower(power)
	b.coulombsUsed += amp * duration.Seconds()
	return power/amp
}

func newBatteryState(b *BatteryPack) *batteryState {
	return &batteryState{pack:b}
}

func (b *batteryState)VoltageAtPower(power float64) float64  {
	diff := math.Sqrt(b.pack.NominalVoltage*b.pack.NominalVoltage - 4*power*b.pack.InternalResistance)
	return (b.pack.NominalVoltage + diff)/2
}

func (b *batteryState)AmpsAtPower(power float64) float64 {
	return power/b.VoltageAtPower(power)
}




