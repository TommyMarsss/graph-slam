// Package graphslam 实现了一个仅依赖 Go 标准库的简化 2D 位姿图优化系统：
// 自研 Levenberg-Marquardt 求解器、错误回环定位、局部最优检测以及静态 HTML 报告。
package graphslam

import "math"

// Pose 是世界坐标系下的 SE(2) 位姿：x, y 为平移，Theta 为弧度制航向角。
type Pose struct {
	X, Y, Theta float64
}

// Measurement 是一条相对位姿约束（边）的测量值 z_ij，
// 表示在节点 i 的机体坐标系下观察到的节点 j：
//
//	z_dx, z_dy = R(-theta_i) * (p_j - p_i)
//	z_dtheta   = Wrap(theta_j - theta_i)
type Measurement struct {
	Dx, Dy, Dtheta float64
}

// Mat3 是行主序的 3x3 信息矩阵（协方差之逆）。
type Mat3 [9]float64

// Tau 是角度环绕的周期（2*pi），单独命名以突出环绕处理逻辑。
const Tau = 2 * math.Pi

// WrapAngle 把任意角度归一化到 (-pi, pi]。
//
// 位姿图优化中角度误差必须经过环绕归一化：例如真实误差只有 0.05rad，
// 未归一化的差值可能是 2*pi-0.05，若直接参与二次代价与梯度，
// 优化器会沿错误方向拉动整整一圈。归一化后残差位于最短弧上，
// 雅可比在该分支内仍是普通线性导数（-1/+1），方向因此正确。
func WrapAngle(a float64) float64 {
	a = math.Mod(a, Tau)
	if a > math.Pi {
		a -= Tau
	} else if a <= -math.Pi {
		// 归一化到 (-pi, pi]，让 +pi 与 -pi 不会被当成两个不同的点。
		a += Tau
	}
	return a
}

// Relative 返回从位姿 a 到位姿 b 的相对位姿测量。
func Relative(a, b Pose) Measurement {
	dx, dy := b.X-a.X, b.Y-a.Y
	c, s := math.Cos(a.Theta), math.Sin(a.Theta)
	return Measurement{
		Dx:     c*dx + s*dy,
		Dy:     -s*dx + c*dy,
		Dtheta: WrapAngle(b.Theta - a.Theta),
	}
}

// Compose 由位姿 a 与相对测量 m 合成下一个位姿（a ⊕ m），用于里程计链式初值。
func Compose(a Pose, m Measurement) Pose {
	c, s := math.Cos(a.Theta), math.Sin(a.Theta)
	return Pose{
		X:     a.X + c*m.Dx - s*m.Dy,
		Y:     a.Y + s*m.Dx + c*m.Dy,
		Theta: WrapAngle(a.Theta + m.Dtheta),
	}
}

// Inverse 求相对位姿测量的逆（j -> i）。
func (m Measurement) Inverse() Measurement {
	c, s := math.Cos(m.Dtheta), math.Sin(m.Dtheta)
	return Measurement{
		Dx:     -c*m.Dx - s*m.Dy,
		Dy:     s*m.Dx - c*m.Dy,
		Dtheta: WrapAngle(-m.Dtheta),
	}
}

// DiagInfo 由三个分量的测量标准差生成对角信息矩阵 diag(1/s^2)。
func DiagInfo(sigmaXY, sigmaTheta float64) Mat3 {
	px := 1.0 / (sigmaXY * sigmaXY)
	pt := 1.0 / (sigmaTheta * sigmaTheta)
	return Mat3{px, 0, 0, 0, px, 0, 0, 0, pt}
}
