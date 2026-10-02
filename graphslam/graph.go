package graphslam

// EdgeKind 区分里程计边与回环边；回环边是错误回环检测的对象。
type EdgeKind int

const (
	Odom EdgeKind = iota
	Loop
)

// Edge 是位姿图中的一条相对位姿约束：节点 I 与节点 J 之间的测量 M 及其信息矩阵 Info。
type Edge struct {
	I, J int
	Kind EdgeKind
	M    Measurement
	Info Mat3
}

// Graph 是位姿图：N 个节点（其位姿是待估状态）加一组边约束。
// 第一个节点（索引 0）作为锚点，通过先验固定，消除全局平移/旋转自由度。
type Graph struct {
	N     int
	Edges []Edge
}

// NewGraph 创建含 n 个节点的空图。
func NewGraph(n int) *Graph {
	if n < 1 {
		panic("graphslam: graph must have at least one node")
	}
	return &Graph{N: n}
}

// AddEdge 追加一条 i -> j 的相对位姿约束。
func (g *Graph) AddEdge(i, j int, kind EdgeKind, m Measurement, info Mat3) {
	if i < 0 || j < 0 || i >= g.N || j >= g.N || i == j {
		panic("graphslam: invalid edge endpoint")
	}
	g.Edges = append(g.Edges, Edge{I: i, J: j, Kind: kind, M: m, Info: info})
}

// LoopEdges 返回所有回环约束（错误回环检测只在这些边之间进行）。
func (g *Graph) LoopEdges() []int {
	var idx []int
	for k := range g.Edges {
		if g.Edges[k].Kind == Loop {
			idx = append(idx, k)
		}
	}
	return idx
}

// Clone 深拷贝图结构（剔除边时需要保留原图）。
func (g *Graph) Clone() *Graph {
	h := &Graph{N: g.N, Edges: make([]Edge, len(g.Edges))}
	copy(h.Edges, g.Edges)
	return h
}

// WithoutEdges 返回一个删除了指定边（按边索引）的新图。
func (g *Graph) WithoutEdges(drop map[int]bool) *Graph {
	h := NewGraph(g.N)
	for k, e := range g.Edges {
		if !drop[k] {
			h.Edges = append(h.Edges, e)
		}
	}
	return h
}
