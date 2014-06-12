package automotiveSim

import (
	"math"
	"time"
	"fmt"
)

const (
	kph100 = 100 / 3.6
	quarterMile = 402.33600 //quarter mile in meters
	causeFilter = 100 //number of simulation intervals
)

type Schedule struct {
	Name string	
    Interval time.Duration
    Speeds []float64
}

func (sim *SimulatorState)Run(input *Schedule) (error) {	
    for i,newSpeed := range input.Speeds {
        accel := (newSpeed - sim.Speed)/input.Interval.Seconds()
		target := input.Interval * time.Duration(i)
        for sim.Time < target {
            currAccel, err := sim.Tick(accel);
            if err != nil {
				return fmt.Errorf("Vehicle failed to accelerate at %5.2fm/s (only %5.2f) (%v)", accel, currAccel, err)
            }
        }
    }
    return nil
}

type LimitingReason struct {
	Reason string
	Start time.Duration
}

type AccelProfile struct {
	TopSpeed float64
	Accel100 float64
	AccelTop float64
	QuarterMile float64
	PeakAccel float64
	Limits []LimitingReason
	Profile []float64
}

func (vehicle *Vehicle)RunAccelerationProfile() (AccelProfile, error) {
	sim, err := InitSimulation(vehicle)
    if err != nil {
    	return AccelProfile{}, err
    }
	
	var result AccelProfile

	speedInterval := time.Millisecond * 10
	var currTime time.Duration
	lastReason := ""
	for result.TopSpeed == 0 || result.QuarterMile == 0 {
		//attempt to accelerate at 1,000 m/s^2
		//it's a binary search, so it only slows things down log(n)
		//so start with a huge n. This gurantees we are always
		//accelerating at maximum speed
		currAccel, err := sim.Tick(1000)
		currReason := err.Error()
		
		if currReason != lastReason {
			result.Limits = append(result.Limits, LimitingReason{Reason:currReason, Start:sim.Time})
		}
		lastReason = currReason
		
		if currAccel > result.PeakAccel {
			result.PeakAccel = currAccel
		}
		
		if sim.Speed > kph100 && result.Accel100 == 0 {
			result.Accel100 = sim.Time.Seconds()
		}
		
		if sim.Distance > quarterMile && result.QuarterMile == 0 {
			result.QuarterMile = sim.Time.Seconds()
		}
		
		//have we hit topspeed
		if currAccel < 0.05  && result.TopSpeed == 0 {
			result.TopSpeed = sim.Speed
			result.AccelTop = sim.Time.Seconds()
			if sim.Speed < kph100 {
				result.Accel100 = math.NaN()
			}
		}
		currTime += sim.Interval
		if currTime > speedInterval {
			result.Profile = append(result.Profile, sim.Speed)
			currTime -= speedInterval
		}
	}
	//clean up transistions
	pos := len(result.Limits) - 1
	lastStart := time.Hour * 1000000
	for i := pos; i >= 0; i-- {
		timeSinceLast := lastStart - result.Limits[i].Start
		if (timeSinceLast >= (sim.Interval * causeFilter)) {
			lastStart = result.Limits[i].Start
			result.Limits[pos] = result.Limits[i]
			pos--
		}
	}
	pos++
	copy(result.Limits[:len(result.Limits) - pos], result.Limits[pos:])
	result.Limits = result.Limits[:len(result.Limits) - pos]
	return result, nil
}

func (vehicle *Vehicle)EfficiencyAtSpeeds(speeds []float64) (map[string][]float64, error) {
	sim, err := InitSimulation(vehicle)
    if err != nil {
    	return nil, err
    }
	
	eff := make(map[string][]float64)
	causes := []string{"Aerodynamics", "Rolling Resistance", "Accessory", "Losses"}
	for _,cause := range causes {
		eff[cause] = make([]float64, len(speeds))
	}
	
	for i,speed := range speeds {
		sim.Speed = speed
		currAccel, err := sim.Tick(0)
		if math.Abs(currAccel) > 0.01 {
			return nil, fmt.Errorf("Vehicle can not maintain speed %5.2f: %v", speed, err)
		}
		//copy the map
		total := sim.Power.Total()/speed
		aero := sim.Body.AeroDrag(sim)
		tire := sim.Body.RollingDrag(sim)
		accessory := sim.Power["Accessory"].(float64)/speed
		eff["Accessory"][i] = accessory
		eff["Aerodynamics"][i] = aero
		eff["Rolling Resistance"][i] = tire
		eff["Losses"][i] = total - (accessory + aero + tire)
	}
	return eff, nil
}




