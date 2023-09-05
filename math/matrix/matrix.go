package matrix

import (
	"fmt"
	"log"
	"math"

	"github.com/kainn9/tteokbokki/sliceHelper"
)

// MatRC is a matrix represented by rows and columns.
// Example 3x3 matrix:
/*
	[       0: 1: 2:
		0: [1, 2, 3],
		1: [4, 5, 6],
		2: [7, 8, 9],
	]
*/
// first index is the row, second index is the column.
type MatRC struct {
	RowCount, ColCount int
	Rows               *[]*[]float64
}

type MatRCCalcError struct {
	msg string
}

func (e *MatRCCalcError) Error() string {
	return e.msg
}

// New empty matrix.
func NewMatRC(rowCount, colCount int) MatRC {

	rows := make([]*[]float64, rowCount)

	for i := range rows {
		col := make([]float64, colCount)
		rows[i] = &col
	}

	matRC := MatRC{rowCount, colCount, &rows}

	return matRC
}

// Flips the rows and columns of a matrix.
func (matRC MatRC) Transpose() MatRC {
	result := NewMatRC(matRC.ColCount, matRC.RowCount)

	for i := 0; i < matRC.RowCount; i++ {
		for j := 0; j < matRC.ColCount; j++ {
			(*(*result.Rows)[j])[i] = (*(*matRC.Rows)[i])[j]
		}
	}

	return result
}

func (matA MatRC) Multiply(matB MatRC) (MatRC, error) {

	if matA.ColCount != matB.RowCount {
		errorString := fmt.Sprintf("error, matA.ColCount != maB.RowCount, %d != %d", matA.ColCount, matB.RowCount)
		return MatRC{}, &MatRCCalcError{msg: errorString}

	}

	transposedB := matB.Transpose()

	result := NewMatRC(matA.RowCount, matB.ColCount)

	for i := 0; i < matA.RowCount; i++ {
		for j := 0; j < matB.ColCount; j++ {
			resultValue := sliceHelper.ScalarProduct(*(*matA.Rows)[i], *(*transposedB.Rows)[j])
			(*(*result.Rows)[i])[j] = resultValue
		}
	}

	return result, nil
}

// Slice Length must be equal to the number of columns in the matrix.
// Returns new slice of the "ScalarProduct" of each col and the slice.
func (mat MatRC) MultiplyBySlice(slice []float64) ([]float64, error) {
	if mat.ColCount != len(slice) {
		errorString := fmt.Sprintf("error mat.ColCount != len(slice), %d != %d", mat.ColCount, len(slice))
		return []float64{}, &MatRCCalcError{msg: errorString}
	}

	result := make([]float64, mat.RowCount)

	for i := range result {
		result[i] = sliceHelper.ScalarProduct(*(*mat.Rows)[i], slice)
	}

	return result, nil
}

// SolveGaussSeidel performs the Gauss-Seidel method to solve a linear system of equations
// represented by the matrix 'mat' and the right-hand side vector 'slice'.
// It iteratively updates the solution vector 'x' until convergence.
// The function returns the computed solution vector 'x'.
func (mat MatRC) SolveGaussSeidel(slice []float64) []float64 {
	// Get the size of the system (number of equations)
	n := len(slice)

	// Initialize the solution vector 'x' with zeros
	x := make([]float64, n)

	// Iterate through each equation in the system
	for i := 0; i < n; i++ {

		if (*(*mat.Rows)[i])[i] == 0.0 {
			continue
		}

		// Calculate the change in the solution for the current equation
		// dx = (b[i] - dot product of the row and the current solution) / diagonal element
		dx := (slice[i] / (*(*mat.Rows)[i])[i]) - (sliceHelper.ScalarProduct(*(*mat.Rows)[i], x) / (*(*mat.Rows)[i])[i])

		// Check if dx is NaN (Not-a-Number); if so, continue to the next equation
		if math.IsNaN(dx) {
			continue
		}

		// Update the solution vector 'x' with the calculated change
		x[i] += dx
	}

	return x
}

// Prints the matrix.
func (mat *MatRC) Print() {
	log.Println("Rows/Columns:", mat.RowCount, mat.ColCount)
	for i := range *mat.Rows {
		fmt.Println(*(*mat.Rows)[i])
	}

}
