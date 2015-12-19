package automotiveSim



type Vehicle struct {
    Accessory float64
    Battery Battery
	Body Body
	Ambient Ambient
}


func (v *Vehicle)Init() error {
	initFuncs := []func() error {
		v.Battery.Init,
		v.Body.Init,
		v.Ambient.Init,
 	}
	
	
	for _,function := range initFuncs {
		err := function()
		if err != nil {
			return err
		}
	}
	
	return nil
}
