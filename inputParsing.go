package automotiveSim

import "encoding/json"
import "fmt"

type Tire struct {
    Friction float64
    RollingResistance float64
    Radius float64
}

type Vehicle struct {
	Powertrain Powertrain
	
    Weight float64
    CdA float64
	
    Accessory float64
    Battery BatteryPack
	
	//environmental conditions TODO pull this into it's own data structure
    ExternalTemp float64
	
}

func Parse(vehicleJSON interface{}) (*Vehicle, error) {    
    var data []byte
    switch t := vehicleJSON.(type) {
        default:
            return nil, fmt.Errorf("Unexpected type: %T", t)
        
        case []byte:
            data = t
        
        case string:
            data = []byte(t)
    }
    
    var vehicle Vehicle
    err := json.Unmarshal(data, &vehicle)
    if err != nil {
        return nil, err
    }
	    
	initFuncs := []func() error {
		vehicle.Battery.Init, 
		vehicle.Powertrain.Init,
	}
	
	for _,function := range initFuncs {
		err := function()
		if err != nil {
			return nil, err
		}
	}
    
    return &vehicle, nil
}

