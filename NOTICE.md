# 第三方组件与许可声明

本仓库的**自研部分**（`code/`、`user/`、根目录文档）由 hhttiuui 编写，
整体以 [GNU GPL v3](LICENSE) 发布。

之所以选 GPLv3 而不是 MIT：`libraries/` 下的逐飞 TC264 开源库本身就是
GPLv3，任何链接它的程序都必须以 GPLv3 分发。下面逐项说明。

---

## 1. 逐飞 TC264 开源库 — GPL v3

路径：`libraries/zf_common/`、`libraries/zf_components/`、
`libraries/zf_device/`、`libraries/zf_driver/`

```
TC264 Opensourec Library
Copyright (C) 2022 SEEKFREE 逐飞科技

This program is free software: you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the Free
Software Foundation, either version 3 of the License, or (at your option)
any later version.
```

- 每个源文件头部都有完整声明，请勿删除。
- 英文版声明原文见 `libraries/doc/GPL3_permission_statement.txt`。
- 库版本：V3.4.3，版本历史见 `libraries/doc/version.txt`。
- 上游仓库：<https://gitee.com/seekfree/TC264_Opensource_Library>

**本仓库对逐飞库未作任何修改。** 如果你要改，请在自己的分支里改，
并保留版权声明（GPLv3 第 5 条要求标注修改）。

---

## 2. Infineon iLLD — Boost Software License 1.0

路径：`libraries/infineon_libraries/`

```
Copyright (c) 2019 Infineon Technologies AG. All rights reserved.
iLLD_1_0_1_11_0

Use of this file is subject to the terms of use agreed between (i) you or
the company in which ordinary course of business you are acting and (ii)
Infineon Technologies AG or its licensees. If and as long as no such
terms of use are agreed, use of this file is subject to following:

Boost Software License - Version 1.0 - August 17th, 2003
```

Boost 1.0 是宽松许可，与 GPLv3 兼容，因此可以整体按 GPLv3 分发。
每个头文件的完整声明请勿删除。

---

## 3. TASKING 链接脚本

路径：`Lcf_Tasking_Tricore_Tc.lsl`

随逐飞开源库一同发布的 TASKING 链接脚本，版权归 TASKING / Altium 所有。
它只在 TASKING 工具链下使用，不参与 GPLv3 的代码衍生关系。

---

## 4. 构建工具（不入库）

- **AURIX Development Studio / TASKING VX-toolset for TriCore** — 商业工具链，
  需自行安装并遵守其许可。本仓库不包含任何编译器或运行时文件。
- `Debug/` 目录下的 makefile、`.mdf`、`.o` 等均为工具自动生成的中间产物，
  已加入 `.gitignore`，首次构建时会自动重建。

---

## 关于本仓库代码

如果你要引用或二次分发 `code/`、`user/` 里的内容，请遵守 GPLv3：
保留版权声明、标注你的修改、并以同样的许可分发。

有疑问可以开 issue。
