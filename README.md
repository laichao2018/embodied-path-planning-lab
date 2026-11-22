# NAV STUDIO - 路径规划工作台

一个用于算法对比与调试的全屏交互式路径规划工作台，把障碍建模、搜索前沿、路径生成和机器人执行放进同一张可视化地图。

## 主要功能

- 有限栅格区域、边界障碍、静态障碍和持续运动的动态障碍
- 原始地图、障碍膨胀地图、渐变代价地图三层切换
- 在地图上交互设置起点与终点，并支持一键重置
- 同时运行 A*、Dijkstra、RRT 和 RRT*，以蓝、黄、红、紫四种高区分度颜色显示
- 动态展示 `GENERATE` 与 `PICK`：新节点、当前扩展节点、搜索树和最终路径
- 搜索完成后，机器人沿优先路径执行；动态障碍持续更新
- 可调障碍膨胀半径、动画速度和参与对比的算法

## 算法说明

- **A***：使用 `g(n) + h(n)` 进行目标导向搜索。
- **Dijkstra**：使用累计代价 `g(n)` 进行等代价扩展。
- **RRT**：通过随机采样、最近邻、Steer、连续边碰撞检测与 parent 回溯生成路径。
- **RRT***：在 RRT 树生长基础上执行邻域最优父节点选择和重连优化，持续降低累计路径代价。
- **动态环境**：动态障碍与局部代价地图持续更新，用于观察路径规划过程中的环境变化。

## 零依赖启动（推荐）

```bash
cd /Users/laichao/workspace/embodied-path-planning-lab
sh start.sh
```

`start.sh` 会直接用系统浏览器打开离线 Demo。运行过程不需要联网，也不需要安装 Node.js、npm、Python 或任何第三方软件包。

克隆 GitHub 项目后也可以直接启动：

```bash
git clone https://github.com/laichao2018/embodied-path-planning-lab.git
cd embodied-path-planning-lab
sh start.sh
```

离线程序位于 `standalone/`，只使用浏览器原生的 HTML、CSS、JavaScript 和 Canvas API。

## 开发模式（可选）

如需修改 React / TypeScript 开发版，再使用 Node.js：

```bash
npm install
npm run dev
```

生产构建：

```bash
npm run build
```

自动验证代价地图和四种规划算法：

```bash
npm test
```

一次完成算法测试与生产构建：

```bash
npm run verify
```

## 交互说明

1. 选择“原始 / 膨胀 / 代价”图层观察地图预处理结果。
2. 按住地图中的绿色起点箭头或橙色终点旗帜，将图标拖到目标栅格；障碍区域会自动拒绝放置。
3. 勾选需要比较的算法，调整膨胀半径与推演速度。
4. 点击“启动路径规划”。亮环表示当前 `PICK` 节点，微光节点或树枝表示 `GENERATE`。
5. 拖动地图下方的 `REPLAY` 时间轴，可前后查看任意阶段的搜索推演过程。

## 技术栈

- 零依赖离线版：原生 HTML + CSS + JavaScript + Canvas 2D
- 开发版：React 19 + TypeScript + Vinext / Vite
- Canvas 2D 自绘地图、搜索过程与机器人执行动画
- Cloudflare Workers 兼容构建（可选）
