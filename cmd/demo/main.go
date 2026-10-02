// Command demo 运行三组位姿图优化场景并生成单一静态 HTML 报告 report.html：
//  1. 正确回环：展示漂移被消除、误差单调下降；
//  2. 注入错误回环：展示轨迹被扭曲并自动定位错误约束；
//  3. 坏初值陷阱：展示几何检测识别局部最优、多起点优化恢复可信解。
//
// 用法：go run ./cmd/demo [输出路径]，默认输出 report.html。
package main

import (
	"fmt"
	"os"

	"github.com/TommyMarsss/graph-slam/graphslam"
)

func main() {
	out := "report.html"
	if len(os.Args) > 1 {
		out = os.Args[1]
	}

	cfg := graphslam.DefaultConfig()
	badCfg := graphslam.DefaultBadLoopConfig()
	report := graphslam.NewReport()

	// 场景 1：正确回环。
	clean := graphslam.LawnmowerScenario(4, 8, 2.0, 7)
	cleanRes := graphslam.Optimize(clean.Graph, clean.Initial, cfg)
	cleanGeo0 := graphslam.DetectLocalOpt(clean.Graph, cleanRes, clean.KnownRevisits)
	cleanGeo := graphslam.ClassifyOutcome(cleanRes, cleanGeo0)
	cleanVerdicts := graphslam.LocateBadLoops(clean.Graph, cleanRes.Poses, cleanRes, badCfg)
	fmt.Printf("[正确回环] 迭代 %d 次，χ²: %.4f -> %.4f，结局=%s，几何可疑=%v，错误回环=%v\n",
		cleanRes.Iterations, cleanRes.Chi2History[0], cleanRes.FinalChi2,
		cleanGeo.Outcome, cleanGeo.Suspicious, cleanVerdicts.BadEdges)

	report.Scenarios = append(report.Scenarios, graphslam.ScenarioReport{
		Name:        clean.Name,
		Description: clean.Description,
		Initial:     graphslam.PosesToFlat(clean.Initial),
		Optimized:   graphslam.PosesToFlat(cleanRes.Poses),
		Truth:       graphslam.PosesToFlat(clean.Truth),
		Edges:       graphslam.EdgesToReport(clean.Graph, nil, cleanRes.Residuals),
		CostHistory: cleanRes.CostHistory,
		Chi2History: cleanRes.Chi2History,
		Converged:   cleanRes.Converged,
		StopReason:  cleanRes.StopReason,
		Iterations:  cleanRes.Iterations,
		FinalChi2:   cleanRes.FinalChi2,
		Geometry:    graphslam.GeometryToReport(cleanGeo),
		Verdicts:    graphslam.VerdictsToReport(cleanVerdicts),
	})

	// 场景 2：注入错误回环。
	inj := graphslam.InjectedLoopScenario(clean)
	injRes := graphslam.Optimize(inj.Graph, inj.Initial, cfg)
	injGeo := graphslam.ClassifyOutcome(injRes,
		graphslam.DetectLocalOpt(inj.Graph, injRes, inj.KnownRevisits))
	injVerdicts := graphslam.LocateBadLoops(inj.Graph, injRes.Poses, injRes, badCfg)
	badSet := map[int]bool{}
	for _, e := range injVerdicts.BadEdges {
		badSet[e] = true
	}
	fmt.Printf("[错误回环] χ²: %.4f -> %.4f，几何可疑=%v，注入边=%v，定位结果=%v\n",
		injRes.Chi2History[0], injRes.FinalChi2, injGeo.Suspicious,
		inj.InjectedBadLoops, injVerdicts.BadEdges)

	report.Scenarios = append(report.Scenarios, graphslam.ScenarioReport{
		Name:        inj.Name,
		Description: inj.Description,
		Initial:     graphslam.PosesToFlat(inj.Initial),
		Optimized:   graphslam.PosesToFlat(injRes.Poses),
		Truth:       graphslam.PosesToFlat(inj.Truth),
		Edges:       graphslam.EdgesToReport(inj.Graph, badSet, injRes.Residuals),
		CostHistory: injRes.CostHistory,
		Chi2History: injRes.Chi2History,
		Converged:   injRes.Converged,
		StopReason:  injRes.StopReason,
		Iterations:  injRes.Iterations,
		FinalChi2:   injRes.FinalChi2,
		Geometry:    graphslam.GeometryToReport(injGeo),
		Verdicts:    graphslam.VerdictsToReport(injVerdicts),
	})

	// 场景 3：坏初值陷阱（8 字形初值）+ 多起点恢复。
	trap := graphslam.TrapRingScenario(30, 5.0, 7)
	startPoses := graphslam.TrapInitials(trap.Graph, trap.Initial)
	ms := graphslam.OptimizeMultiStart(trap.Graph, trap.KnownRevisits, startPoses, cfg)
	best := ms.Starts[ms.Best]
	fmt.Printf("[坏初值陷阱] 多起点 %d 个，选中=%q（代价 %.4f，几何可疑=%v）\n",
		len(ms.Starts), best.Name, best.Result.FinalCost, best.Geometry.Suspicious)
	for i, st := range ms.Starts {
		fmt.Printf("   起点 %d %q: 代价=%.4f 几何问题=%d\n", i, st.Name, st.Result.FinalCost, len(st.Geometry.Issues))
	}

	jsStarts := make([]graphslam.StartSummary, 0, len(ms.Starts))
	for i, st := range ms.Starts {
		jsStarts = append(jsStarts, graphslam.StartSummary{
			Name: st.Name, Cost: st.Result.FinalCost, Plausible: st.Plausible,
			Issues: len(st.Geometry.Issues), Chosen: i == ms.Best,
		})
	}
	report.Scenarios = append(report.Scenarios, graphslam.ScenarioReport{
		Name:        trap.Name,
		Description: trap.Description,
		Initial:     graphslam.PosesToFlat(trap.Initial),
		Optimized:   graphslam.PosesToFlat(best.Result.Poses),
		Truth:       graphslam.PosesToFlat(trap.Truth),
		Edges:       graphslam.EdgesToReport(trap.Graph, nil, best.Result.Residuals),
		CostHistory: best.Result.CostHistory,
		Chi2History: best.Result.Chi2History,
		Converged:   best.Result.Converged,
		StopReason:  best.Result.StopReason,
		Iterations:  best.Result.Iterations,
		FinalChi2:   best.Result.FinalChi2,
		Geometry:    graphslam.GeometryToReport(best.Geometry),
		MultiStart:  jsStarts,
	})

	if err := report.WriteHTML(out); err != nil {
		fmt.Fprintln(os.Stderr, "写出报告失败:", err)
		os.Exit(1)
	}
	fmt.Println("报告已生成:", out)
}
