package graphslam

import (
	"errors"
	"math"
)

// 本文件实现最小二乘所需的稠密线性代数：正定矩阵 Cholesky 分解与前代/回代。
// 规模为 3N×3N（N 个 SE(2) 节点），演示规模下稠密求解足够，不依赖任何第三方库。

// cholesky 对对称正定矩阵 A（行主序 n×n）做 A = L·Lᵀ 分解，返回下三角 L。
func cholesky(A []float64, n int) ([]float64, error) {
	L := make([]float64, n*n)
	for j := 0; j < n; j++ {
		for i := j; i < n; i++ {
			s := A[i*n+j]
			for k := 0; k < j; k++ {
				s -= L[i*n+k] * L[j*n+k]
			}
			if i == j {
				if s <= 1e-12 {
					return nil, errors.New("graphslam: matrix is not positive definite")
				}
				L[i*n+j] = math.Sqrt(s)
			} else {
				L[i*n+j] = s / L[j*n+j]
			}
		}
	}
	return L, nil
}

// solveSPD 求解 A x = b，要求 A 对称正定；通过 Cholesky 分解完成。
func solveSPD(A []float64, b []float64, n int) ([]float64, error) {
	L, err := cholesky(A, n)
	if err != nil {
		return nil, err
	}
	x := make([]float64, n)

	// 前代：L y = b
	y := make([]float64, n)
	for i := 0; i < n; i++ {
		s := b[i]
		for k := 0; k < i; k++ {
			s -= L[i*n+k] * y[k]
		}
		y[i] = s / L[i*n+i]
	}
	// 回代：Lᵀ x = y
	for i := n - 1; i >= 0; i-- {
		s := y[i]
		for k := i + 1; k < n; k++ {
			s -= L[k*n+i] * x[k]
		}
		x[i] = s / L[i*n+i]
	}
	return x, nil
}
