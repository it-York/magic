# 第三方软件声明

Elevator-LIO 自有代码以 `GPL-2.0-or-later` 许可证发布。独立以 MIT License 提供的文件保留其
文件级 SPDX 标识；引入的第三方代码继续受各自上游许可证约束。

## ikd-Tree

- 上游项目：[hku-mars/ikd-Tree](https://github.com/hku-mars/ikd-Tree)
- 原作者：Yixi Cai、Wei Xu、Fu Zhang 及贡献者
- 上游许可证：GNU General Public License, version 2（上游原文为 `GPLv2`）
- 本仓库中的文件：`include/ikd_tree/ikd_Tree.h` 和 `include/ikd_tree/ikd_Tree.cpp`
- 本地修改：集成到 Elevator-LIO，适配路径和类型，并补充注释

上游 README 和许可证文件均将该项目描述为 GPLv2，但没有给出明确的文件级 SPDX
`only/or-later` 后缀。本说明因此只保留上游的 `GPLv2` 原始表述，不擅自将第三方授权扩展为
`GPL-2.0-or-later`。再发布本仓库时应同时保留上游来源、GPLv2 许可证文本和本地修改说明。

在学术工作中使用 ikd-Tree 时，应引用以下论文：

```bibtex
@article{cai2021ikd,
  title   = {ikd-Tree: An Incremental KD Tree for Robotic Applications},
  author  = {Cai, Yixi and Xu, Wei and Zhang, Fu},
  journal = {arXiv preprint arXiv:2102.10808},
  year    = {2021}
}
```

## ikd-Tree 中文注释

- 来源项目：[KennyWGH/ikd-Tree-detailed](https://github.com/KennyWGH/ikd-Tree-detailed)
- 关系：`hku-mars/ikd-Tree` 的 fork
- 许可证：GNU General Public License, version 2
- 使用内容：`include/ikd_tree/ikd_Tree.h` 和 `include/ikd_tree/ikd_Tree.cpp` 中的部分中文解释性注释

## 外部依赖

ROS、PCL、Eigen、OpenCV、yaml-cpp、TBB、OpenMP 及其传递依赖未内置到本仓库中。它们仍受各自许可证约束。
