package automotiveSim

import "encoding/json"
import "fmt"


type Tire struct {
    Friction float64
    RollingResistance float64
    Radius float64
}

type Vehicle struct {
    Motors []Motor
    Weight float64
    CdA float64
    Accessory float64
    ExternalTemp float64
    Tires Tire
    Battery BatteryPack
    ElectricalEff float64
    DrivetrainEff float64
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
  
    err = vehicle.Battery.Init()
    if err != nil {
    }
	
    return &vehicle, nil
}

