package automotiveSim


func airDensity(temperature float64) float64 {
    return (100000/((temperature)*287.058)) //ideal gas law
}

//calculates energy transfer based on direction of energy flow
//and system efficiency 
func et(amount, efficiency float64) float64 {
	if(amount > 0) {
		return amount / efficiency
	} else {
		return amount * efficiency
	}
}

