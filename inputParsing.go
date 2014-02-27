package automotiveSim

import (
	"encoding/json"
)


func Parse(vehicleJSON []byte) (*Vehicle, error) {        
    var vehicle Vehicle
    err := json.Unmarshal(vehicleJSON, &vehicle)
    if err != nil {
        return nil, err
    }
	   
	err = vehicle.Init()	
	if err != nil {
		return nil, err
	}
    
    return &vehicle, nil
}

