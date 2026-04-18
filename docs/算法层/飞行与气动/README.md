# 飞行与气动文档索引

本目录对应算法层的飞行与气动业务域。

## 代码入口

```mermaid
flowchart LR
    subgraph 入口["📦 业务域入口"]
        DOM["domains/flight_dynamics.hpp"]
    end
    subgraph 环境["🌍 环境输入"]
        ATM["core/atmosphere.hpp<br/>密度 ρ, 音速 a, 动压 q"]
    end
    subgraph 气动["✈️ 气动计算"]
        AERO["aero/aerodynamics.hpp<br/>升阻, 过载, PS"]
    end
    DOM --> ATM & AERO
    ATM --> AERO

    style DOM fill:#dbeafe
    style AERO fill:#e1f5e1
```

- `include/xsf_math/domains/flight_dynamics.hpp`
- `include/xsf_math/core/atmosphere.hpp`
- `include/xsf_math/aero/aerodynamics.hpp`

## 文档

- `基础知识整理.md`
- `气动与飞行.md`
