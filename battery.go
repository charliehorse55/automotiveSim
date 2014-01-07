package automotiveSim


import (
	// "time"
	// "errors"
	"math"
)

type Battery struct {
	Voltage float64
	Resistance float64
	Coulomb float64
}

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
	coulombs  float64
	pack *BatteryPack 	
}


func (b *BatteryPack)Init() bool {
	if b.CellCoulomb <= 0 {
		return false
	}
	
	if b.CellResistance < 0 {
		return false
	}
	
	if b.CellVoltage <= 0 {
		return false
	}
	
	if b.Series == 0 || b.Parallel == 0 {
		return false
	}	
	
    b.NominalVoltage = b.CellVoltage * float64(b.Series)
    b.Coulomb = b.CellCoulomb * float64(b.Parallel)
    b.InternalResistance = b.CellResistance*float64(b.Series) / float64(b.Parallel)
	
	return true
}

func newBatteryState(b *BatteryPack) *batteryState {
	return &batteryState{pack:b,  }
}

func (pack *BatteryPack)ampsAtPower(power float64) float64  {
	absPow := math.Abs(power)
    amps := (2 * absPow) / (pack.NominalVoltage  + math.Sqrt(pack.NominalVoltage*pack.NominalVoltage - 4*absPow*pack.InternalResistance))
	return math.Copysign(amps, power)

}

func (pack *BatteryPack)maxPower() float64 {
    return pack.NominalVoltage*pack.NominalVoltage/(4*pack.InternalResistance)
}
