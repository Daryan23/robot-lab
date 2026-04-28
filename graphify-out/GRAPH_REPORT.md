# Graph Report - .  (2026-04-28)

## Corpus Check
- 2 files · ~1,268 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 18 nodes · 19 edges · 4 communities detected
- Extraction: 100% EXTRACTED · 0% INFERRED · 0% AMBIGUOUS
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- [[_COMMUNITY_Community 0|Community 0]]
- [[_COMMUNITY_Community 1|Community 1]]
- [[_COMMUNITY_Community 2|Community 2]]
- [[_COMMUNITY_Community 3|Community 3]]

## God Nodes (most connected - your core abstractions)
1. `point_in_any_zone()` - 3 edges
2. `point_in_circle()` - 2 edges
3. `point_in_polygon()` - 2 edges
4. `fresh_other_robots()` - 2 edges
5. `nearest_other()` - 2 edges

## Surprising Connections (you probably didn't know these)
- None detected - all connections are within the same source files.

## Communities

### Community 0 - "Community 0"
Cohesion: 0.2
Nodes (0): 

### Community 1 - "Community 1"
Cohesion: 0.67
Nodes (0): 

### Community 2 - "Community 2"
Cohesion: 0.67
Nodes (3): point_in_any_zone(), point_in_circle(), point_in_polygon()

### Community 3 - "Community 3"
Cohesion: 1.0
Nodes (2): fresh_other_robots(), nearest_other()

## Knowledge Gaps
- **Thin community `Community 3`** (2 nodes): `fresh_other_robots()`, `nearest_other()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `point_in_any_zone()` connect `Community 2` to `Community 0`?**
  _High betweenness centrality (0.004) - this node is a cross-community bridge._