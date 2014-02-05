package automotiveSim


import (
	// "time"
	"math"
	"fmt"
)

type BatteryPack struct {
    NominalVoltage float64
    InternalResistance float64
    Coulomb float64
    Series uint
    Parallel uint
    CellVoltage float64
    CellResistance float64
    CellCoulomb float64
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
	
	if b.Series == 0 {
		return fmt.Errorf("Battery pack must have at least one cell in series")
	}	
	if b.Parallel == 0 {
		return fmt.Errorf("Battery pack must have at least one cell in parallel")
	}
	
    b.NominalVoltage = b.CellVoltage * float64(b.Series)
    b.Coulomb = b.CellCoulomb * float64(b.Parallel)
    b.InternalResistance = b.CellResistance*float64(b.Series) / float64(b.Parallel)
	
	return nil
}

func NewBatteryState(b *BatteryPack) *batteryState {
	return &batteryState{pack:b}
}

func (b *batteryState)CanOperate(current float64, duration time.Duration) error {
	

	return nil
}


func (b *batteryState)Operate(current float64, duration time.Duration) {
	

}

func newBatteryState(b *BatteryPack) *batteryState {
	return &batteryState{pack:b}
}


func (pack *batteryState)AmpsAtPower(power float64) float64  {
	absPow := math.Abs(power)
    amps := (2 * absPow) / (pack.NominalVoltage + math.Sqrt(pack.NominalVoltage*pack.NominalVoltage - 4*absPow*pack.InternalResistance))
	return math.Copysign(amps, power)
}

func (b *batteryState)VoltageForPower(power float64) float64 {
	return power/b.AmpsAtPower(power)
}


func (pack *batteryState)MaxPower() float64 {
    return pack.NominalVoltage*pack.NominalVoltage/(4*pack.InternalResistance)
}




