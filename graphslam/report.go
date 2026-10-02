package graphslam

import (
	"encoding/json"
	"os"
	"strings"
	"time"
)

// 以下是报告的数据模型：序列化为 JSON 后内嵌进单个静态 HTML，
// 页面端仅用原生 JavaScript 渲染 SVG，不引用任何第三方资源。

// FlatPose 是位姿的紧凑 JSON 表示 [x, y, theta]。
type FlatPose [3]float64

// ReportEdge 描述报告中的一条边及其错误标记。
type ReportEdge struct {
	I        int     `json:"i"`
	J        int     `json:"j"`
	Kind     string  `json:"kind"` // "odom" | "loop"
	Bad      bool    `json:"bad"`
	Residual float64 `json:"residual"`
}

// ReportIssue 是一处几何不可信问题。
type ReportIssue struct {
	Kind   string  `json:"kind"`
	Detail string  `json:"detail"`
	A      int     `json:"a"`
	B      int     `json:"b"`
	Value  float64 `json:"value"`
}

// StartSummary 是多起点优化中一个起点的摘要。
type StartSummary struct {
	Name      string  `json:"name"`
	Cost      float64 `json:"cost"`
	Plausible bool    `json:"plausible"`
	Issues    int     `json:"issues"`
	Chosen    bool    `json:"chosen"`
}

// ReportVerdict 是一条回环边的错误回环诊断结论。
type ReportVerdict struct {
	Edge        int      `json:"edge"`
	I           int      `json:"i"`
	J           int      `json:"j"`
	Residual    float64  `json:"residual"`
	OtherMax    float64  `json:"otherMax"`
	Improvement float64  `json:"improvement"`
	Suspicious  bool     `json:"suspicious"`
	Reasons     []string `json:"reasons"`
}

// ScenarioReport 是报告中的一个场景块。
type ScenarioReport struct {
	Name        string          `json:"name"`
	Description string          `json:"description"`
	Initial     []FlatPose      `json:"initial"`
	Optimized   []FlatPose      `json:"optimized"`
	Trapped     []FlatPose      `json:"trapped,omitempty"` // 坏初值直接优化后被困住的解（仅陷阱场景）
	Truth       []FlatPose      `json:"truth"`
	Edges       []ReportEdge    `json:"edges"`
	CostHistory []float64       `json:"costHistory"`
	Chi2History []float64       `json:"chi2History"`
	Converged   bool            `json:"converged"`
	StopReason  string          `json:"stopReason"`
	Iterations  int             `json:"iterations"`
	FinalChi2   float64         `json:"finalChi2"`
	Geometry    *GeometryReport `json:"geometry,omitempty"`
	Verdicts    []ReportVerdict `json:"verdicts,omitempty"`
	MultiStart  []StartSummary  `json:"multiStart,omitempty"`
}

// GeometryReport 是局部最优几何检测的 JSON 表示。
type GeometryReport struct {
	Suspicious bool          `json:"suspicious"`
	Outcome    string        `json:"outcome"`
	Chi2PerDOF float64       `json:"chi2PerDof"`
	MedianStep float64       `json:"medianStep"`
	Issues     []ReportIssue `json:"issues"`
}

// Report 是整页数据。
type Report struct {
	GeneratedAt string           `json:"generatedAt"`
	Scenarios   []ScenarioReport `json:"scenarios"`
}

// PosesToFlat 把 []Pose 转为序列化用的紧凑数组。
func PosesToFlat(ps []Pose) []FlatPose {
	out := make([]FlatPose, len(ps))
	for i, p := range ps {
		out[i] = FlatPose{p.X, p.Y, p.Theta}
	}
	return out
}

// EdgesToReport 导出边及错误标记/残差。
func EdgesToReport(g *Graph, bad map[int]bool, residuals []EdgeResidual) []ReportEdge {
	out := make([]ReportEdge, len(g.Edges))
	for k, e := range g.Edges {
		kind := "odom"
		if e.Kind == Loop {
			kind = "loop"
		}
		out[k] = ReportEdge{
			I: e.I, J: e.J, Kind: kind, Bad: bad[k],
			Residual: residuals[k].Normalized,
		}
	}
	return out
}

// GeometryToReport 转换几何检测结果。
func GeometryToReport(r *LocalOptReport) *GeometryReport {
	if r == nil {
		return nil
	}
	issues := make([]ReportIssue, 0, len(r.Issues))
	for _, is := range r.Issues {
		issues = append(issues, ReportIssue{
			Kind: is.Kind, Detail: is.Detail, A: is.A, B: is.B, Value: is.Value,
		})
	}
	return &GeometryReport{
		Suspicious: r.Suspicious, Outcome: r.Outcome, Chi2PerDOF: r.Chi2PerDOF,
		MedianStep: r.MedianStep, Issues: issues,
	}
}

// VerdictsToReport 转换错误回环诊断结果。
func VerdictsToReport(r *BadLoopReport) []ReportVerdict {
	out := make([]ReportVerdict, 0, len(r.Verdicts))
	for _, v := range r.Verdicts {
		reasons := v.Reasons
		if reasons == nil {
			reasons = []string{}
		}
		out = append(out, ReportVerdict{
			Edge: v.Edge, I: v.I, J: v.J,
			Residual: v.FinalResidual, OtherMax: v.MaxOtherLoopResidual,
			Improvement: v.LeaveOneOutImprovement,
			Suspicious:  v.Suspicious, Reasons: reasons,
		})
	}
	return out
}

// NewReport 创建带时间戳的空报告。
func NewReport() *Report {
	return &Report{GeneratedAt: time.Now().Format("2006-01-02 15:04:05")}
}

// WriteHTML 把报告写入单个自包含 HTML 文件（无外部依赖、无需服务器）。
func (r *Report) WriteHTML(path string) error {
	data, err := json.Marshal(r)
	if err != nil {
		return err
	}
	// 防止 JSON 中的 "</script>" 提前闭合脚本标签。
	safe := escapeForScript(string(data))
	// @BT@ 是 Go 源码中 JS 模板字面量反引号的占位符（反引号会终结原始字符串）。
	html := strings.ReplaceAll(reportHTMLPrefix, "@BT@", "`")
	// 数据注入到 <script> 内部的锚点，保证 JSON 不会落到脚本标签外。
	html = strings.Replace(html, "/*@DATA@*/null", safe, 1)
	return os.WriteFile(path, []byte(html), 0o644)
}

func escapeForScript(s string) string {
	out := make([]byte, 0, len(s))
	for i := 0; i < len(s); i++ {
		if s[i] == '<' && i+1 < len(s) && s[i+1] == '/' {
			out = append(out, '<', '\\', '/')
			i++
			continue
		}
		out = append(out, s[i])
	}
	return string(out)
}
