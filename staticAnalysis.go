package automotiveSim

import "math"


func airDensity(temperature float64) float64 {
    return (100000/((temperature+273.15)*287.058)) //ideal gas law
}

func (vehicle *Vehicle)Drag(speed float64) float64 {
	return vehicle.RollingDrag(speed) + vehicle.AeroDrag(speed)
}

func (vehicle *Vehicle)AeroDrag(speed float64) float64 {
    return 0.5 * vehicle.CdA * speed * speed * airDensity(vehicle.ExternalTemp) //aero
}

func (vehicle *Vehicle)RollingDrag(speed float64) float64 {
	return vehicle.Weight * 9.81 * vehicle.Tires.RollingResistance
}

func (vehicle *Vehicle)PowerUse(speed float64) float64 {
    return (vehicle.Drag(speed) * speed)/(vehicle.ElectricalEff * vehicle.DrivetrainEff) + vehicle.Accessory
}

