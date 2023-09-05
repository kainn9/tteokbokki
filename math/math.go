package math

// Clamp clamps a value between a minimum and maximum value.
func Clamp(v, min, max float64) float64 {

	if min > max {
		return v
	}

	if min == max {
		return min
	}

	if v > max {
		return max
	}

	if v < min {
		return min
	}

	return v
}
