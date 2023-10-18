package tBokiSliceHelper

// Reverses the order of the elements in a slice.
// Returns a new slice without mutating the original.
func Invert[T any](slice []T) []T {

	for i, j := 0, len(slice)-1; i < j; i, j = i+1, j-1 {
		slice[i], slice[j] = slice[j], slice[i]
	}

	return slice
}

// Removes the last element from a slice and returns it.
// Mutates the original slice.
func Pop[T any](slice *[]T) *T {
	if len(*slice) == 0 {
		return nil
	}
	popped := (*slice)[len(*slice)-1]
	*slice = (*slice)[:len(*slice)-1]
	return &popped
}

// Returns the "scalar product" of two slices.
func ScalarProduct(sliceA, sliceB []float64) float64 {
	sum := 0.0

	for i := range sliceA {
		sum += sliceA[i] * sliceB[i]
	}

	return sum
}

func MultiplySliceByFloat(v []float64, n float64) []float64 {
	result := make([]float64, len(v))
	for i, val := range v {
		result[i] = val * n
	}
	return result
}

func AddSlices(v []float64, n []float64) []float64 {
	if len(v) == 0 {
		return n
	}
	if len(n) == 0 {
		return v
	}

	result := make([]float64, len(v))

	for i := range v {
		result[i] = v[i] + n[i]
	}

	return result
}

func SubtractSlices(v []float64, n []float64) []float64 {
	if len(v) == 0 {
		return n
	}
	if len(n) == 0 {
		return v
	}

	result := make([]float64, len(v))

	for i := range v {
		result[i] = v[i] - n[i]
	}

	return result
}
