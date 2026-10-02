package main

import (
	"errors"
	"math"
)

// choleskySolve 求解对称正定方程组 A x = b。
// A 为 n×n 行主序稠密矩阵。仅使用 Go 标准库实现 Cholesky 分解：
//
//	A = L Lᵀ，先前代 L y = b，再回代 Lᵀ x = y。
//
// 位姿图规模通常只有几十个节点（每节点 3 个自由度），稠密求解足够。
func choleskySolve(A []float64, n int, b []float64) ([]float64, error) {
	L := make([]float64, n*n)

	for j := 0; j < n; j++ {
		for i := j; i < n; i++ {
			sum := A[i*n+j]
			for k := 0; k < j; k++ {
				sum -= L[i*n+k] * L[j*n+k]
			}
			if i == j {
				if sum <= 1e-12 {
					// 节点自由度未被约束（如未固定首节点）会走到这里
					return nil, errors.New("matrix is not positive definite")
				}
				L[i*n+j] = math.Sqrt(sum)
			} else {
				L[i*n+j] = sum / L[j*n+j]
			}
		}
	}

	// 前代 L y = b
	y := make([]float64, n)
	for i := 0; i < n; i++ {
		sum := b[i]
		for k := 0; k < i; k++ {
			sum -= L[i*n+k] * y[k]
		}
		y[i] = sum / L[i*n+i]
	}

	// 回代 Lᵀ x = y
	x := make([]float64, n)
	for i := n - 1; i >= 0; i-- {
		sum := y[i]
		for k := i + 1; k < n; k++ {
			sum -= L[k*n+i] * x[k]
		}
		x[i] = sum / L[i*n+i]
	}
	return x, nil
}
