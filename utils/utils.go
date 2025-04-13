package utils

import (
	"bufio"
	"encoding/csv"
	"fmt"
	"math"
	"os"
	"strconv"

	"github.com/tuneinsight/lattigo/v6/ring"
)

// Maps a float to interval (-T/2, T/2).
// Input
// - s : float
// Output
// - sOut: float
func SignFloat(s float64, T uint64) float64 {
	Tf := float64(T)
	sOut := s - math.Floor((s+Tf/2.0)/Tf)*Tf
	return sOut
}

// Takes average of a vector v
// Input
// - v : m x 1
// Output
// - avg: scalar
func Average(v []float64) float64 {
	avg := float64(0)
	for i := 0; i < len(v); i++ {
		avg += v[i]
	}
	avg = avg / float64(len(v))
	return avg
}

// Multiply a scalar to a matrix
// Input
// - s : scalar float
// - M : m x n float matrix
// Output
// - MOut: m x n float matrix
func ScalMatMult(s float64, M [][]float64) [][]float64 {
	row := len(M)
	col := len(M[0])
	MOut := make([][]float64, row)
	for r := 0; r < row; r++ {
		MOut[r] = make([]float64, col)
		for c := 0; c < col; c++ {
			MOut[r][c] = s * M[r][c]
		}
	}
	return MOut
}

// Multiply a matrix to a vector
// Input
// - M : m x n float matrix
// - v : n x 1 float vector
// Output
// - vOut: m x l float vector
func MatVecMult(M [][]float64, v []float64) []float64 {
	row := len(M)
	col := len(M[0])
	colv := len(v)

	if col != colv {
		panic(fmt.Errorf("Matrix dimension don't match"))
	}

	vOut := make([]float64, row)
	for r := 0; r < row; r++ {
		tmp := 0.0
		for k := 0; k < col; k++ {
			tmp = tmp + M[r][k]*v[k]
		}
		vOut[r] = tmp
	}
	return vOut
}

// Mod q of float matrix
// Each component of MOut belongs to [0, q).
// Input
// - M : m x n float matrix
// Output
// - MOut: m x n uint64 matrix
func ModMatFloat(M [][]float64, q uint64) [][]uint64 {
	qf := float64(q)
	row := len(M)
	col := len(M[0])
	MOut := make([][]uint64, row)
	for r := 0; r < row; r++ {
		MOut[r] = make([]uint64, col)
		for c := 0; c < col; c++ {
			// MOut[r][c] = uint64(M[r][c] - math.Floor(M[r][c]/qf)*qf)
			MOut[r][c] = uint64(M[r][c]) - uint64(math.Floor(M[r][c]/qf))*q
		}
	}
	return MOut
}

// Convert a matrix to a vector
// The matrix must have 1 column
// Input
// - M : m x 1 float matrix
// Output
// - vOut : m x 1 float vector
func MatToVec(M [][]float64) []float64 {
	row := len(M)
	col := len(M[0])
	if col > 1 {
		panic(fmt.Errorf("input is not a column vector"))
	}

	vOut := make([]float64, row)
	for r := 0; r < row; r++ {
		vOut[r] = M[r][0]
	}

	return vOut
}

// Convert a matrix to a string
// Input
// - M : m x n float matrix
// Output
// - strOut : m x n string matrix
func MatToStr(M [][]float64) [][]string {
	row := len(M)
	col := len(M[0])
	strOut := make([][]string, row)
	for r := 0; r < row; r++ {
		strOut[r] = make([]string, col)
		for c := 0; c < col; c++ {
			strOut[r][c] = strconv.FormatFloat(M[r][c], 'f', -1, 64)
		}
	}
	return strOut
}

// Add a vector to a vector
// Input
// - v1 : m x 1 float vector
// - v2 : m x 1 float vector
// Output
// - vOut: m x 1 float vector
func VecAdd(v1 []float64, v2 []float64) []float64 {
	row := len(v1)
	vOut := make([]float64, row)
	for r := 0; r < row; r++ {
		vOut[r] = v1[r] + v2[r]
	}
	return vOut
}

// Subtract a vector to a vector
// Input
// - v1 : m x 1 float vector
// - v2 : m x 1 float vector
// Output
// - vOut: m x 1 float vector
func VecSub(v1 []float64, v2 []float64) []float64 {
	row := len(v1)
	vOut := make([]float64, row)
	for r := 0; r < row; r++ {
		vOut[r] = v1[r] - v2[r]
	}
	return vOut
}

// Multiply a scalar to a vector
// Input
// - s : scalar float
// - v : m x 1 float vector
// Output
// - vOut: m x 1 float vector
func ScalVecMult(s float64, v []float64) []float64 {
	row := len(v)

	vOut := make([]float64, row)
	for r := 0; r < row; r++ {
		vOut[r] = s * v[r]
	}
	return vOut
}

func ScalVecMultInt(s int64, v []int64) []int64 {
	row := len(v)

	vOut := make([]int64, row)
	for r := 0; r < row; r++ {
		vOut[r] = s * v[r]
	}
	return vOut
}

// Inner sum of vector elements (mod q)
// Input
// - v : m x 1 uint64 vector
// Output
// - sum: scalar uint64
func VecSumUint(v []uint64, T uint64, bredparams [2]uint64) uint64 {
	row := len(v)
	sum := uint64(0)
	for r := 0; r < row; r++ {
		sum += v[r]
	}
	sum = ring.BRedAdd(sum, T, bredparams)
	return sum
}

// Mod q of float vector
// Each component of vOut belongs to [0, q).
// Input
// - v : m x 1 float vector
// Output
// - vOut: m x 1 uint64 vector
func ModVecFloat(v []float64, q uint64) []uint64 {
	qf := float64(q)
	row := len(v)
	vOut := make([]uint64, row)
	for r := 0; r < row; r++ {
		// vOut[r] = uint64(v[r] - math.Floor(v[r]/qf)*qf)
		vOut[r] = uint64(v[r]) - uint64(math.Floor(v[r]/qf))*q
	}
	return vOut
}

func ModVec(v []int64, q uint64) []uint64 {
	qf := float64(q)
	row := len(v)
	vOut := make([]uint64, row)
	for r := 0; r < row; r++ {
		// vOut[r] = uint64(v[r] - math.Floor(v[r]/qf)*qf)
		vOut[r] = uint64(v[r] - int64(math.Floor(float64(v[r])/qf))*int64(q))
	}
	return vOut
}

// Duplicate + zero padding
// length h vector n times
// Length of a must be less than or equal to h
// Input
// - v : float vector
// Output
// - v: nh x 1 float vector
func VecDuplicate(v []float64, n int, h int) []float64 {
	row := len(v)
	if row > h {
		panic(fmt.Errorf("length of a is greater than h"))
	}
	if row < h {
		for k := 0; k < h-row; k++ {
			v = append(v, 0)
		}
	}
	for i := 1; i < n; i++ {
		for j := 0; j < row; j++ {
			v = append(v, v[j])
		}
		if row < h {
			for k := 0; k < h-row; k++ {
				v = append(v, 0)
			}
		}
	}
	return v
}

// 2-norm of a vector
// Input
// - v : n x 1 float vector
// Output
// - s: scalar float
func Vec2Norm(v []float64) float64 {
	tmp := 0.0
	for i := range v {
		tmp = tmp + v[i]*v[i]
	}
	return math.Sqrt(tmp)
}

// Rounding of a vector
// Input
// - v : n x 1 float vector
// Output
// - v : n x 1 float vector
//
//	func RoundVec(v []float64) []float64 {
//		row := len(v)
//		vOut := make([]float64, row)
//		for r := 0; r < row; r++ {
//			vOut[r] = math.Round(v[r])
//		}
//		return vOut
//	}
func RoundVec(v []float64) []int64 {
	row := len(v)
	vOut := make([]int64, row)
	for r := 0; r < row; r++ {
		vOut[r] = int64(math.Round(v[r]))
	}
	return vOut
}

// Convert a vector to a string
// Input
// - v : n x 1 float vector
// Output
// - strOut : n x 1 string matrix
func VecToStr(v []float64) [][]string {
	row := len(v)
	strOut := make([][]string, row)
	for r := 0; r < row; r++ {
		strOut[r] = make([]string, 1)
		strOut[r][0] = strconv.FormatFloat(v[r], 'f', -1, 64)
	}
	return strOut
}

func DataExport(data [][]float64, fileName string) {
	file, err := os.Create(fileName)
	if err != nil {
		panic(err)
	}
	wr := csv.NewWriter(bufio.NewWriter(file))
	wr.WriteAll(MatToStr(data))
}
