package automotiveSim


import (
)

type Power map[string]interface{}

func (p Power)Total() float64 {
	sum := 0.0
	for _, val := range p {
		switch t := val.(type) {
			case float64:
				sum += t
				
			case Power:
				sum += t.Total()
		}
	}
	return sum
}

func (p Power)Copy() Power {
	result := make(Power)
	for key, val := range p {
		switch t := val.(type) {
			case float64:
				result[key] = t
				
			case Power:
				result[key] = t.Copy()
		}
	}
	return result
}

