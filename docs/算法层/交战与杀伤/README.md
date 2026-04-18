# 交战与杀伤文档索引

本目录对应算法层的交战与杀伤业务域。

## 代码入口

```mermaid
flowchart LR
    subgraph 入口["📦 业务域入口"]
        DOM["domains/engagement.hpp"]
    end
    subgraph 制导["🧭 制导"]
        GUIDE["guidance/proportional_nav.hpp"]
    end
    subgraph 杀伤["💥 杀伤评估"]
        FUZE["lethality/fuze.hpp"]
        PK1["lethality/pk_model.hpp"]
        PK2["lethality/launch_pk_table.hpp"]
    end
    DOM --> GUIDE & FUZE & PK1 & PK2
    GUIDE --> FUZE --> PK1 & PK2

    style DOM fill:#dbeafe
    style PK1 fill:#e1f5e1
    style PK2 fill:#e1f5e1
```

- `include/xsf_math/domains/engagement.hpp`
- `include/xsf_math/guidance/proportional_nav.hpp`
- `include/xsf_math/lethality/fuze.hpp`
- `include/xsf_math/lethality/pk_model.hpp`
- `include/xsf_math/lethality/launch_pk_table.hpp`

## 文档

- `基础知识整理.md`
- `比例导引.md`
- `引信与PCA.md`
- `杀伤效能与Pk.md`
