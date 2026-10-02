package main

import (
	"flag"
	"fmt"
	"math"
	"os"
)

// ScenarioRun 收集一个场景的完整运行结果，供报告使用。
type ScenarioRun struct {
	Scenario  Scenario
	Initial   []Pose
	Optimized OptimizeResult
	Diagnosis Diagnosis
}

func runAll() []ScenarioRun {
	var runs []ScenarioRun
	for _, s := range allScenarios() {
		opt := optimize(s.Graph, s.Initial, DefaultLMConfig())
		d := diagnose(s.Graph, s.Initial, opt)
		runs = append(runs, ScenarioRun{
			Scenario: s, Initial: s.Initial, Optimized: opt, Diagnosis: d,
		})
	}
	return runs
}

func main() {
	out := flag.String("out", "report.html", "静态 HTML 报告输出路径")
	flag.Parse()

	runs := runAll()
	for _, r := range runs {
		d := r.Diagnosis
		fmt.Fprintf(os.Stdout, "=== %s ===\n", r.Scenario.Graph.Name)
		fmt.Fprintf(os.Stdout, "  LM: status=%s iterations=%d finalCost=%.6g\n",
			r.Optimized.Status, r.Optimized.Iter, d.FinalCost)
		fmt.Fprintf(os.Stdout, "  诊断: %s", d.Status)
		if d.CulpritID != "" {
			fmt.Fprintf(os.Stdout, " (culprit=%s)", d.CulpritID)
		}
		fmt.Fprintln(os.Stdout)
		fmt.Fprintf(os.Stdout, "  里程计最大残差: 平移 %.4fm (%.1f%% 边长), 角度 %.4frad; 自交 %d 处\n",
			d.MaxOdomPosErr, 100*d.MaxOdomErrRatio, d.MaxOdomAngErr, d.SelfIntersections)
		maxDev := 0.0
		for i := range r.Scenario.Truth {
			if dev := math.Hypot(r.Optimized.Nodes[i].X-r.Scenario.Truth[i].X,
				r.Optimized.Nodes[i].Y-r.Scenario.Truth[i].Y); dev > maxDev {
				maxDev = dev
			}
		}
		fmt.Fprintf(os.Stdout, "  与真值最大偏差: %.4fm\n", maxDev)
		for _, n := range d.Notes {
			fmt.Fprintf(os.Stdout, "    · %s\n", n)
		}
	}

	if err := writeHTMLReport(*out, runs); err != nil {
		fmt.Fprintf(os.Stderr, "写报告失败: %v\n", err)
		os.Exit(1)
	}
	fmt.Fprintf(os.Stdout, "\n静态报告已生成: %s\n", *out)
}
