# 基础支撑文档索引

本目录对应算法层的基础支撑业务域。

## 代码入口

```mermaid
flowchart LR
    subgraph 入口["📦 业务域入口"]
        DOM["domains/foundation.hpp"]
    end
    subgraph 数学基础["📐 数学基础"]
        CONST["core/constants.hpp"]
        VEC["core/vec3.hpp"]
        MAT["core/mat3.hpp"]
        INTERP["core/interpolation.hpp"]
    end
    subgraph 几何物理["🌍 几何与物理"]
        COORD["core/coordinate_transform.hpp"]
        ATM["core/atmosphere.hpp"]
    end
    DOM --> CONST & VEC & MAT & INTERP & COORD & ATM
    VEC & MAT --> COORD
    CONST --> ATM

    style DOM fill:#dbeafe
    style ATM fill:#e1f5e1
    style COORD fill:#e1f5e1
```

- `include/xsf_math/domains/foundation.hpp`
- `include/xsf_math/core/constants.hpp`
- `include/xsf_math/core/vec3.hpp`
- `include/xsf_math/core/mat3.hpp`
- `include/xsf_math/core/interpolation.hpp`
- `include/xsf_math/core/coordinate_transform.hpp`
- `include/xsf_math/core/atmosphere.hpp`

## 文档

- `基础知识整理.md`
- `坐标系统与变换.md`
