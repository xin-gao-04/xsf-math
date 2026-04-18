# 跟踪估计文档索引

本目录对应算法层的跟踪估计业务域。

## 代码入口

```mermaid
flowchart LR
    subgraph 入口["📦 业务域入口"]
        DOM["domains/tracking.hpp"]
    end
    subgraph 关联["🔗 量测关联"]
        ASSOC["tracking/track_association.hpp"]
    end
    subgraph 滤波["🎯 状态估计"]
        KF["tracking/kalman_filter.hpp"]
    end
    DOM --> ASSOC & KF
    ASSOC -.->|关联后的量测| KF

    style DOM fill:#dbeafe
    style KF fill:#e1f5e1
```

- `include/xsf_math/domains/tracking.hpp`
- `include/xsf_math/tracking/kalman_filter.hpp`
- `include/xsf_math/tracking/track_association.hpp`

## 文档

- `基础知识整理.md`
- `跟踪与滤波.md`
