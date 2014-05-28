package automotiveSim


func airDensity(T, p float64) float64 {
	M := 0.0289644 // kg/mol, (dry air)
	R := 8.31447 //J/(mol·K) (ideal gas constant)
	return (p*M)/(T*R)
}

//calculates energy transfer based on direction of energy flow
//and system efficiency 
//assumes positive power = energy towards wheels
func et(amount, efficiency float64) float64 {
	if(amount > 0) {
		return amount / efficiency
	} else {
		return amount * efficiency
	}
}



