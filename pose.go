package main

import "math"

// Pose 表示一个 2D 位姿 (x, y, θ)，θ 以弧度表示，约定存储范围为 (-π, π]。
type Pose struct {
	X     float64 `json:"x"`
	Y     float64 `json:"y"`
	Theta float64 `json:"theta"`
}

// wrapAngle 把任意角度规范化到 (-π, π]。
// 位姿图优化中角度是流形上的量：残差角度必须先做环绕规范化，
// 否则在 ±π 边界处误差会从 +π 跳到 -π，导致梯度方向完全相反。
func wrapAngle(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

// Compose 位姿复合 a ⊕ b：在世界系中先做 a 再做 b。
func Compose(a, b Pose) Pose {
	c, s := math.Cos(a.Theta), math.Sin(a.Theta)
	return Pose{
		X:     a.X + c*b.X - s*b.Y,
		Y:     a.Y + s*b.X + c*b.Y,
		Theta: wrapAngle(a.Theta + b.Theta),
	}
}

// Invert 位姿求逆 a⁻¹。
func Invert(a Pose) Pose {
	c, s := math.Cos(a.Theta), math.Sin(a.Theta)
	return Pose{
		X:     -c*a.X - s*a.Y,
		Y:     s*a.X - c*a.Y,
		Theta: wrapAngle(-a.Theta),
	}
}
