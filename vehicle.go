package automotiveSim


type Vehicle struct {
    Accessory float64
    Battery BatteryPack
	Body Body
}


func (v *Vehicle)Init() error {
	initFuncs := []func() error {
		v.Battery.Init,
		v.Body.Init,
 	}
	
	
	for _,function := range initFuncs {
		err := function()
		if err != nil {
			return err
		}
	}
	
	return nil
}
