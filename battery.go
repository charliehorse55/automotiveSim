package automotiveSim


import (
	"time"
	"math"
	"fmt"
)

type Battery struct {
	Component
    NominalVoltage float64
    Resistance float64
    Coulomb float64
	MaxCurrent float64
	ChargerEfficency float64
	
	//state
	coulombsUsed float64
}

func (b *Battery)Init() error {
	if b.Coulomb <= 0 {
		return fmt.Errorf("Battery must store a positive amount of charge")
	}
	
	if b.Resistance < 0 {
		return fmt.Errorf("Battery can not have negative resistance")
	}
	
	if b.NominalVoltage <= 0 {
		return fmt.Errorf("Battery must have positive nominal voltage")
	}
	
	if b.MaxCurrent <= 0 {
		return fmt.Errorf("Battery must have positive maximum current")
	}
	
	if b.ChargerEfficency <= 0 || b.ChargerEfficency > 1 {
		return fmt.Errorf("Charger efficiency must be on the range (0,1]")
	}
	b.Power = make(Power)
	
	return nil
}

func (b *Battery)CanOperate(sim *SimulatorState, power float64) error {
	amp := b.AmpsAtPower(power)
	if math.IsNaN(amp) || math.Abs(amp) > b.MaxCurrent {
		return fmt.Errorf("Exceeds max pack current")
	}
	coulomb := amp * sim.Interval.Seconds()
	if (coulomb + b.coulombsUsed) > b.Coulomb {
		return fmt.Errorf("Battery Energy depleted")
	}
	return nil
}

func (b *Battery)Operate(sim *SimulatorState, power float64) float64 {
	amp := b.AmpsAtPower(power)
	time := sim.Interval.Seconds()
	b.coulombsUsed += amp * time
	totalUsed := (amp*b.NominalVoltage)
	b.Power["Internal Resistance"] = totalUsed - power
	sim.Resources["Electricity"] += (totalUsed * time) / b.ChargerEfficency
	return power/amp
}

func (b *Battery)VoltageAtPower(power float64) float64  {
	diff := math.Sqrt(b.NominalVoltage*b.NominalVoltage - 4*power*b.Resistance)
	return (b.NominalVoltage + diff)/2
}

func (b *Battery)AmpsAtPower(power float64) float64 {
	return power/b.VoltageAtPower(power)
}

func (b *Battery)StateOfCharge() float64 {
	return 1.0 - (b.coulombsUsed/b.Coulomb)
}




