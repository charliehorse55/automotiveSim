package automotiveSim

import (
	"math"
	"time"
	"fmt"
)

const (
	kph100 = 100 / 3.6
	quarterMile = 402.33600 //quarter mile in meters
)

type Schedule struct {
    Interval time.Duration
    Speeds []float64
}

type ScheduleResult struct {
	Power []float64
	Distance float64
}

func (input *Schedule)Run(vehicle *Vehicle) (*ScheduleResult, error) {
    sim, err := InitSimulation(vehicle)
    if err != nil {
    	return nil, err
    }
	
	var result ScheduleResult
	result.Power = make([]float64, len(input.Speeds))
    for i,newSpeed := range input.Speeds {
        accel := (newSpeed - sim.Speed)/input.Interval.Seconds()
		var powerUsage float64
		target := input.Interval * time.Duration(i)
		numTicks := 0
        for sim.Time < target {
            currAccel, err := sim.Tick(accel);
            if err != nil {
				return nil, fmt.Errorf("Vehicle failed to accelerate at %5.2fm/s (only %5.2f) (%v)", accel, currAccel, err)
            }
			powerUsage += sim.TotalPowerUse()
			numTicks++
        }
		result.Power[i] = powerUsage/float64(numTicks)
    }
	result.Distance = sim.Distance
    return &result, nil
}

type LimitingReason struct {
	Reason string
	Duration time.Duration
}

type AccelProfile struct {
	TopSpeed float64
	Accel100 float64
	QuarterMile float64
	AccelTop float64
	PeakAccel float64
	Limits []LimitingReason
}

func (vehicle *Vehicle)RunAccelerationProfile() (AccelProfile, error) {
	sim, err := InitSimulation(vehicle)
    if err != nil {
    	return AccelProfile{}, err
    }
	
	var result AccelProfile
	
	reasonIndex := 0
	for result.TopSpeed == 0 || result.QuarterMile == 0 {
		currAccel, err := sim.Tick(10000) //attempt to accelerate at 10,000 m/s^2
		currReason := err.Error()
		
		if result.Limits == nil {
			result.Limits = make([]LimitingReason, 1)
			result.Limits[0].Reason = currReason
		}
		
		if currReason == result.Limits[reasonIndex].Reason {
			result.Limits[reasonIndex].Duration += sim.Interval
		} else {
			result.Limits = append(result.Limits, LimitingReason{Reason:currReason, Duration:sim.Interval})
			reasonIndex++
		}
		
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
	}
	return result, nil
}

type PowerDraw struct {
	Sources []string
	Speeds []float64
	Magnitude []PowerUse
}

func (vehicle *Vehicle)PowerAtSpeeds(speeds []float64) (*PowerDraw, error) {
	sim, err := InitSimulation(vehicle)
    if err != nil {
    	return nil, err
    }
	
	var power PowerDraw
	power.Speeds = speeds
	power.Magnitude = make([]PowerUse, len(speeds))
	for i,speed := range speeds {
		sim.Speed = speed
		currAccel, err := sim.Tick(0)
		if math.Abs(currAccel) > 0.01 {
			return nil, fmt.Errorf("Vehicle can not maintain speed %5.2f: %v", speed, err)
		}
		//copy the map
		power.Magnitude[i] = make(PowerUse)
		for key,value := range sim.PowerUses {
			power.Magnitude[i][key] = value
		}
	}
	return &power, nil
}




