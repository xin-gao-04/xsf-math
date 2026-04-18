# 轨道动力学文档索引

本目录对应算法层的轨道动力学业务域。

## 代码入口

```mermaid
flowchart LR
    subgraph 入口["📦 业务域入口"]
        DOM["domains/orbital_dynamics.hpp"]
    end
    subgraph 轨道计算["🌍 轨道计算"]
        KEPLER["orbital/kepler.hpp"]
        J2["orbital/j2.hpp"]
    end
    subgraph 机动["🚀 轨道机动"]
        MAN["orbital/maneuvers.hpp"]
    end
    DOM --> KEPLER & J2 & MAN
    KEPLER --> J2 --> MAN

    style DOM fill:#dbeafe
    style MAN fill:#e1f5e1
```

- `include/xsf_math/domains/orbital_dynamics.hpp`
- `include/xsf_math/orbital/kepler.hpp`
- `include/xsf_math/orbital/j2.hpp`
- `include/xsf_math/orbital/maneuvers.hpp`

## 文档

- `基础知识整理.md`
- `轨道力学.md`
