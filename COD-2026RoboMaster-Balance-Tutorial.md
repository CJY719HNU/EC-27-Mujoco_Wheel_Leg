# MuJoCo MJCF 模型文件分层教程

> **模型名称**：COD-2026-Balance-2.0（RoboMaster 平衡步兵机器人）
> **文件**：[COD-2026RoboMaster-Balance.xml](COD-2026RoboMaster-Balance.xml)

---

## 目录

1. [第 1 层：XML 声明与根元素](#第-1-层xml-声明与根元素)
2. [第 2 层：全局默认设置 `<default>`](#第-2-层全局默认设置-default)
3. [第 3 层：编译器选项 `<compiler>`](#第-3-层编译器选项-compiler)
4. [第 4 层：资源管理 `<asset>`](#第-4-层资源管理-asset)
5. [第 5 层：物理世界 `<worldbody>`](#第-5-层物理世界-worldbody)
   - [5.1 基座几何体](#51-基座几何体)
   - [5.2 刚体 `<body>`](#52-刚体-body)
   - [5.3 关节 `<joint>`](#53-关节-joint)
   - [5.4 惯性属性 `<inertial>`](#54-惯性属性-inertial)
   - [5.5 几何体 `<geom>`](#55-几何体-geom)
   - [5.6 标记点 `<site>`](#56-标记点-site)
   - [5.7 完整嵌套层次结构](#57-完整嵌套层次结构)
6. [第 6 层：约束系统 `<equality>`](#第-6-层约束系统-equality)
7. [第 7 层：驱动器 `<actuator>`](#第-7-层驱动器-actuator)
8. [第 8 层：关键帧 `<keyframe>`](#第-8-层关键帧-keyframe)
9. [附录：完整属性速查表](#附录完整属性速查表)

---

## 第 1 层：XML 声明与根元素

### 1.1 XML 声明

```xml
<?xml version='1.0' encoding='utf-8'?>
```

| 属性 | 含义 |
|------|------|
| `version='1.0'` | XML 版本号，固定为 1.0 |
| `encoding='utf-8'` | 文件编码格式，支持中文注释 |

> **注意**：MuJoCo 也支持深色主题的 `.xml` 文件（如 `model.xml`），它们语法完全相同，只是 MuJoCo 会在保存时自动格式化。

### 1.2 根元素 `<mujoco>`

```xml
<mujoco model="COD-2026-Balance-2.0">
  ...
</mujoco>
```

| 属性 | 含义 | 示例值 |
|------|------|--------|
| `model` | 模型名称（标识符） | `"COD-2026-Balance-2.0"` |

所有 MJCF 内容必须包裹在 `<mujoco>` 标签内。`model` 属性用于在 MuJoCo 运行时标识此模型。

---

## 第 2 层：全局默认设置 `<default>`

### 2.1 语法与作用

```xml
<default>
  <geom contype="1" conaffinity="0"/>
</default>
```

`<default>` 元素用于设置**全局默认属性**——所有匹配类型的子元素会**自动继承**这些属性，除非元素自身显式覆写。

### 2.2 本例中的默认值

```xml
<geom contype="1" conaffinity="0"/>
```

这意味着：**所有几何体默认参与碰撞检测（contype=1），但不主动产生碰撞（conaffinity=0）**。

#### `contype`（碰撞类型）
- 值域：0~65535（bitmask）
- 含义：该 geom 的碰撞"身份标签"
- 当 geom A 的 `contype` 与 geom B 的 `conaffinity` 按位与（AND）结果非零时，二者之间发生碰撞

#### `conaffinity`（碰撞亲和性）
- 值域：0~65535（bitmask）
- 含义：该 geom "愿意"与哪些类型碰撞
- 默认设为 0 表示该 geom 不与任何东西碰撞

> **设计意图**：此模型中大部分结构件之间不需要碰撞检测（因为它们被关节约束在一起），设为 0/0 可以禁用自碰撞，提升仿真性能。如需启用某些部件之间的碰撞，可在对应 `<geom>` 中单独覆写。

### 2.3 其他常见默认设置

`<default>` 还可以嵌套 `<class>` 来创建可复用的属性宏：

```xml
<default>
  <geom contype="1" conaffinity="0"/>
  <!-- 定义一个 class，后续 geom 可通过 class="wheel" 引用 -->
  <default class="wheel">
    <geom rgba="0.1 0.1 0.1 1" friction="1.0 0.005 0.0001"/>
  </default>
</default>
```

---

## 第 3 层：编译器选项 `<compiler>`

### 3.1 语法

```xml
<compiler angle="radian" />
```

`<compiler>` 控制 MJCF 解析器如何处理数值和坐标。

### 3.2 本例设置

| 属性 | 值 | 含义 |
|------|-----|------|
| `angle` | `"radian"` | 所有角度值使用**弧度**（而非角度） |

如果不设置 `angle="radian"`，默认单位是**角度（degree）**。例如：
- `range="-1 1"` 在弧度模式下表示约 ±57.3° 的运动范围
- 若使用默认角度模式，`range="-1 1"` 仅表示 ±1°

### 3.3 其他常用编译选项

| 属性 | 可选值 | 说明 |
|------|--------|------|
| `angle` | `"radian"` / `"degree"` | 角度单位 |
| `coordinate` | `"local"` / `"global"` | 坐标系模式 |
| `autolimits` | `"true"` / `"false"` | 是否自动从网格推断关节范围 |
| `boundmass` | 正浮点数 | 无惯性体的默认质量（默认 0） |
| `boundinertia` | 正浮点数 | 无惯性体的默认惯性（默认 0） |
| `inertiafromgeom` | `"true"` / `"false"` | 是否从 geom 形状自动计算惯性 |
| `meshdir` | 路径字符串 | 网格文件的搜索目录 |

---

## 第 4 层：资源管理 `<asset>`

### 4.1 概述

```xml
<asset>
  <mesh name="base_link" content_type="model/stl" file="base_link.STL" />
  <mesh name="Left_front_link" content_type="model/stl" file="Left_front_link.STL" />
  ...
</asset>
```

`<asset>` 是模型的**资源仓库**，用于声明外部文件（网格、纹理等）和内置材质。定义在此处的资源被后续的 `<geom>`、`<site>` 等元素通过 `name` 引用。

### 4.2 `<mesh>` —— 网格资源

```xml
<mesh name="base_link" content_type="model/stl" file="base_link.STL" />
```

| 属性 | 含义 | 示例 |
|------|------|------|
| `name` | 资源的唯一标识名，后续通过此名引用 | `"base_link"` |
| `content_type` | 文件 MIME 类型 | `"model/stl"` |
| `file` | 网格文件路径（相对路径或绝对路径） | `"base_link.STL"` |

### 4.3 本模型中的网格清单

本模型共 **15 个 STL 网格文件**，构成了完整的 RoboMaster 平衡步兵机器人外形：

| Name | 对应部件 | 左右 |
|------|---------|------|
| `base_link` | 底盘 / 基座 | 中心 |
| `Left_front_link` | 左前腿大腿 | 左 |
| `Left_front_child1_link` | 左前腿小腿1 | 左 |
| `Left_front_child2_link` | 左前腿小腿2 | 左 |
| `Left_front_child3_link` | 左前腿小腿3（末端） | 左 |
| `Left_rear_link` | 左后腿大腿 | 左 |
| `Left_rear_child1_link` | 左后腿小腿 | 左 |
| `Left_Wheel_link` | 左轮 | 左 |
| `Right_front_link` | 右前腿大腿 | 右 |
| `Right_front_child1_link` | 右前腿小腿1 | 右 |
| `Right_front_child2_link` | 右前腿小腿2 | 右 |
| `Right_front_child3_link` | 右前腿小腿3（末端） | 右 |
| `Right_rear_link` | 右后腿大腿 | 右 |
| `Right_rear_child1_link` | 右后腿小腿 | 右 |
| `Right_Wheel_link` | 右轮 | 右 |

### 4.4 组织结构示意

```
base_link（底盘）
├── Left_front_link → child1 → child2 → child3（左前腿链）
├── Left_rear_link → child1 → Left_Wheel_link（左后腿链 + 轮子）
├── Right_front_link → child1 → child2 → child3（右前腿链）
└── Right_rear_link → child1 → Right_Wheel_link（右后腿链 + 轮子）
```

### 4.5 其他可用资源类型

| 标签 | 用途 |
|------|------|
| `<mesh>` | 三角网格文件（STL / OBJ / MSH） |
| `<texture>` | 纹理贴图 |
| `<material>` | 材质定义（反射率、光泽度等） |
| `<hfield>` | 高度场（地形） |

---

## 第 5 层：物理世界 `<worldbody>`

### 5.1 概述

```xml
<worldbody>
  <!-- 所有物理实体都在此定义 -->
</worldbody>
```

`<worldbody>` 是 MJCF 中**最重要的部分**——在此定义机器人的完整运动学树（kinematic tree），包括：
- 身体的嵌套父子关系
- 每个身体的惯性、关节、几何外观、碰撞形状
- 用于传感器/约束/控制的标记点（site）

### 5.2 基座几何体

```xml
<geom type="mesh" rgba="1 1 1 1" mesh="base_link" />
```

这是直接放在 `<worldbody>` 下的一个**无父 body 的 geom**——它没有关节、没有惯性体，意味着它是**固定在地面上的静态装饰/参考物**。

> **为什么这样设计？** 如果需要在仿真中让机器人整体可以移动，通常会在此处定义一个带有 `free` joint 的根 body。但在此模型中，`base_link` 的 geom 更像是场景中的参考底座（或简化后直接固定在世界上），实际可动的腿直接挂在 worldbody 下的各个 `<body>` 上。

### 5.3 刚体 `<body>`

```xml
<body name="Left_front_link" pos="-0.0193914 -0.1837 -0.05">
  ...
</body>
```

`<body>` 表示一个**刚体**（rigid body），是 MuJoCo 动力学计算的基本单元。

#### 核心属性

| 属性 | 含义 | 数据类型 | 示例 |
|------|------|---------|------|
| `name` | 刚体的唯一名称 | 字符串 | `"Left_front_link"` |
| `pos` | 相对于**父 body** 的平移偏移 | 3 个浮点数（X Y Z） | `"-0.019 -0.183 -0.05"` |
| `quat` | 相对于父 body 的四元数旋转 | 4 个浮点数（w x y z） | `"1 0 0 0"`（无旋转） |
| `euler` | 相对于父 body 的欧拉角旋转 | 3 个浮点数 | `"0 0 1.57"` |
| `mocap` | 是否为动捕刚体（受外部控制） | `"true"` / `"false"` | `"false"` |
| `childclass` | 子 body 的默认 class | 字符串 | — |

> **坐标系说明**：在 MuJoCo 中，**X 轴向前，Y 轴向左，Z 轴向上**（右手坐标系）。

#### 本例中的 body 定位

从根 body 的 pos 值分析（以 `Left_front_link` 为例）：
```
pos="-0.0193914 -0.1837 -0.05"
       ↑X         ↑Y      ↑Z
     前/后      左/右    上/下
```
- `X = -0.019`：略微向后偏移
- `Y = -0.1837`：向左侧约 18cm（左前腿）
- `Z = -0.05`：向下偏移 5cm

右侧对称的 `Right_front_link`：`pos="-0.0193914 0.1817 -0.05"`（Y 为正值表示右侧）。

### 5.4 关节 `<joint>`

```xml
<joint name="Left_front_joint" pos="0 0 0" axis="0 1 0" damping="0.1" range="-1 1"/>
```

关节连接父子 body，定义了它们之间的**运动自由度**。

#### 属性详解

| 属性 | 含义 | 本例值 | 说明 |
|------|------|--------|------|
| `name` | 关节名称 | `"Left_front_joint"` | 被 actuator 和 keyframe 引用 |
| `pos` | 关节在**父 body 坐标系**中的位置 | `"0 0 0"` | 与父 body 原点重合 |
| `axis` | 关节旋转轴（单位向量） | `"0 1 0"` | 绕 Y 轴旋转（水平铰链） |
| `damping` | 关节阻尼系数 | `0.1` | 模拟摩擦力/阻力 |
| `range` | 关节运动范围（弧度） | `"-1 1"` | 约 ±57.3° 范围 |
| `ref` | 关节参考位置（零位偏移） | `"-0.1"` | 见下文 |
| `type` | 关节类型（默认 `"hinge"`） | （默认） | 旋转铰链关节 |

> **关节类型默认为 `hinge`（铰链/旋转关节）**——这也是本模型所有关节的类型。其他常见类型：`slide`（滑动）、`ball`（球窝）、`free`（六自由度）。

#### `range` —— 关节限位

| 表达式 | 含义 |
|--------|------|
| `range="-1 1"` | 允许在 ±1 弧度（约 ±57.3°）范围内旋转 |
| `range="-1.35 0"` | 只能从 -1.35 rad 旋转到 0 rad（单向弯曲） |
| `range="0 0.98"` | 只能从 0 rad 旋转到 0.98 rad（正向弯曲） |
| `range="-1.3 0"` | 只能从 -1.3 rad 旋转到 0 rad |

> 注意：本模型使用 `compiler angle="radian"`，所有角度均为弧度。

#### `ref` —— 参考偏移

```xml
<joint name="Right_front_joint" ... ref="-0.1"/>
```

`ref` 设定关节的**初始位置（零位参考）**，单位与 `range` 一致。例如 `ref="-0.1"` 表示关节在仿真开始时的初始角度为 -0.1 弧度。

本模型中带有 `ref` 的关节：

| 关节 | ref 值 | 含义 |
|------|--------|------|
| `Right_front_joint` | `-0.1` | 右前腿初始偏转 -0.1 rad |
| `Right_rear_joint` | `0.0` | 右后腿初始位置为 0 rad |

#### `axis` —— 旋转轴方向分析

| 轴值 | 方向 | 出现位置 |
|------|------|---------|
| `"0 1 0"` | 绕 Y 轴正方向旋转 | 左前腿、左后小腿、左后轮 |
| `"0 -1 0"` | 绕 Y 轴负方向旋转 | 左后腿、右前腿、右后小腿、右后轮 |

> **为什么左右腿的 axis 方向不同？** 这是为了保证在相同的控制信号下，左右腿的弯曲方向相对于机器人是**物理对称**的。左侧关节绕 +Y 弯曲时向内，右侧关节绕 -Y 弯曲时也向内。

### 5.5 惯性属性 `<inertial>`

```xml
<inertial pos="0.032009 -0.002932 -0.003563"
          quat="0.472786 0.528055 0.525808 0.470275"
          mass="0.7"
          diaginertia="0.000152893 0.000125485 2.9469e-05" />
```

`<inertial>` 定义了刚体的**质量和惯性张量**——这是动力学仿真的核心参数。

#### 属性详解

| 属性 | 含义 | 数据类型 |
|------|------|---------|
| `mass` | 质量（kg） | 1 个正浮点数 |
| `pos` | 质心在 body 坐标系中的位置 | 3 个浮点数（X Y Z） |
| `quat` | 惯性主轴相对于 body 坐标系的旋转（四元数） | 4 个浮点数（w x y z） |
| `diaginertia` | 对角化后的惯性张量 | 3 个正浮点数（Ixx Iyy Izz） |
| `fullinertia` | 完整惯性张量（替代 diaginertia） | 6 个浮点数 |

#### 质量分布分析

```xml
<!-- 大腿（front/rear link）：较重 -->
mass="0.7"   diaginertia="0.00015 0.00012 2.9e-05"

<!-- 小腿（child link）：较轻 -->
mass="0.25"  diaginertia="5.3e-05 5.2e-05 1.5e-06"

<!-- 中段：中等 -->
mass="0.45"  diaginertia="0.00027 0.00026 1.7e-05"

<!-- 轮子：最重 -->
mass="0.9"   diaginertia="0.00043 0.00023 0.00022"
```

> **设计洞察**：大腿和轮子质量最大（0.7~0.9 kg），小腿质量较轻（0.25~0.45 kg）。这符合实际 RoboMaster 机器人的质量分布——电机和轮毂集中在近端。

#### `diaginertia` vs `fullinertia`

- **`diaginertia`**（对角惯性）：假设惯性张量已经对角化（惯性主轴与 body 坐标系对齐），只需提供 `(Ixx, Iyy, Izz)` 三个值。
- **`fullinertia`**（完整惯性）：提供完整的 3×3 对称惯性矩阵的 6 个独立分量 `(Ixx, Iyy, Izz, Ixy, Ixz, Iyz)`。

本模型使用 `diaginertia` + 适当的 `quat` 来旋转惯性主轴，这是一种更简洁的表示方式。

### 5.6 几何体 `<geom>`

```xml
<geom type="mesh" rgba="0.89804 0.91765 0.92941 1" mesh="Left_front_link" />
```

`<geom>` 定义 body 的**视觉外观**和**碰撞形状**。

#### 属性详解

| 属性 | 含义 | 示例 |
|------|------|------|
| `type` | 几何体类型 | `"mesh"`（三角形网格）、`"box"`、`"sphere"`、`"cylinder"`、`"capsule"`、`"ellipsoid"` |
| `mesh` | 引用的网格资源名（需在 `<asset>` 中定义） | `"Left_front_link"` |
| `rgba` | 颜色：红 绿 蓝 透明度（0~1） | `"0.898 0.918 0.929 1"`（浅蓝灰） |
| `contype` | 碰撞类型（覆写 default） | 见第 2 层 |
| `conaffinity` | 碰撞亲和性（覆写 default） | 见第 2 层 |
| `pos` | 相对于 body 坐标系的偏移 | 3 个浮点数 |
| `quat` | 相对于 body 坐标系的旋转 | 4 个浮点数 |
| `size` | 几何体尺寸（取决于 type） | 如 `"0.05 0.1"`（圆柱体半径和半高） |
| `friction` | 摩擦系数 | `"1.0 0.005 0.0001"`（滑动、扭转、滚动摩擦） |
| `mass` | 可选：为此 geom 单独设质量 | 正浮点数 |
| `group` | 可见性分组（0~5） | 整数 |

#### 本例中的颜色编码

| rgba | 颜色 | 部件 |
|------|------|------|
| `"1 1 1 1"` | 白色 | base_link、多数 child link |
| `"0.898 0.918 0.929 1"` | 浅蓝灰 | 左右前腿大腿 |
| `"0.792 0.820 0.933 1"` | 淡蓝 | 左右轮子 |
| `"0.776 0.757 0.737 1"` | 灰褐色 | 右前小腿2 |

### 5.7 标记点 `<site>`

```xml
<site name="Left_front_site1" pos="-0.098 -0.002 0.0665" size="0.005" rgba="1 0 0 1"/>
```

`<site>` 是附着在 body 上的**虚拟标记点**——它们不参与碰撞，但对约束、传感器和控制至关重要。

#### 属性详解

| 属性 | 含义 | 示例 |
|------|------|------|
| `name` | 标记点名称 | `"Left_front_site1"` |
| `pos` | 在 body 坐标系中的位置 | `"-0.098 -0.002 0.0665"` |
| `size` | 渲染时的可视化球体半径 | `"0.005"`（5mm） |
| `rgba` | 颜色 | `"1 0 0 1"`（红色） |
| `type` | 视觉形状 | `"sphere"`（默认）、`"box"`、`"ellipsoid"` 等 |

#### 本模型中的 Site 布局

每个 site 的用途是作为**闭环约束的锚点**（详见第 6 层）：

```
左前腿链                     左后腿链
  Left_front_site1 ●---------● Left_rear_site1   (闭环 loop1)
  Left_front_site2 ●---------● Left_rear_site2   (闭环 loop2)

右前腿链                     右后腿链
  Right_front_site1 ●--------● Right_rear_site1  (闭环 loop1)
  Right_front_site2 ●--------● Right_rear_site2  (闭环 loop2)
```

| Site 名称 | 所在 Body | 颜色 | 用途 |
|-----------|----------|------|------|
| `Left_front_site1` | `Left_front_child2_link` | 红色 | 前-后连接点1 |
| `Left_front_site2` | `Left_front_child3_link` | 红色 | 前-后连接点2 |
| `Left_rear_site1` | `Left_rear_link` | 黄色 | 前-后连接点1 |
| `Left_rear_site2` | `Left_rear_child1_link` | 黄色 | 前-后连接点2 |
| `Right_front_site1` | `Right_front_child2_link` | 红色 | 前-后连接点1 |
| `Right_front_site2` | `Right_front_child3_link` | 红色 | 前-后连接点2 |
| `Right_rear_site1` | `Right_rear_link` | 黄色 | 前-后连接点1 |
| `Right_rear_site2` | `Right_rear_child1_link` | 黄色 | 前-后连接点2 |

### 5.8 完整嵌套层次结构

```
<mujoco>
  <worldbody>
    ├── <geom name="base_link" />                    ← 固定底座（无 body）
    │
    ├── <body name="Left_front_link">                ← 左前腿根（大腿）
    │   ├── <inertial ... />
    │   ├── <joint name="Left_front_joint" />        ← 第1关节
    │   ├── <geom mesh="Left_front_link" />
    │   │
    │   └── <body name="Left_front_child1_link">     ← 小腿1
    │       ├── <joint name="Left_front_child1_joint" /> ← 第2关节
    │       │
    │       └── <body name="Left_front_child2_link"> ← 小腿2
    │           ├── <joint name="Left_front_child2_joint" /> ← 第3关节
    │           ├── <site name="Left_front_site1" />  ← 闭环锚点
    │           │
    │           └── <body name="Left_front_child3_link"> ← 小腿3（末端）
    │               ├── <joint name="Left_front_child3_joint" /> ← 第4关节
    │               └── <site name="Left_front_site2" /> ← 闭环锚点
    │
    ├── <body name="Left_rear_link">                 ← 左后腿根（大腿）
    │   ├── <joint name="Left_rear_joint" />         ← 第1关节
    │   ├── <site name="Left_rear_site1" />           ← 闭环锚点
    │   │
    │   └── <body name="Left_rear_child1_link">      ← 后小腿
    │       ├── <joint name="Left_rear_child1_joint" /> ← 第2关节
    │       ├── <site name="Left_rear_site2" />       ← 闭环锚点
    │       │
    │       └── <body name="Left_Wheel_link">         ← 左轮
    │           └── <joint name="Left_Wheel_joint" /> ← 轮关节
    │
    ├── <body name="Right_front_link">               ← [右侧对称结构]
    │   └── ...（与左侧前腿链对称）
    │
    └── <body name="Right_rear_link">                ← [右侧对称结构]
        └── ...（与左侧后腿链对称）
  </worldbody>
</mujoco>
```

#### 运动学链条数统计

| 腿链 | 关节数 | 最高嵌套深度 |
|------|--------|------------|
| 左前腿链 | 4（front_joint → child1 → child2 → child3） | body 嵌套 4 层 |
| 左后腿链 | 3（rear_joint → child1 → wheel） | body 嵌套 3 层 |
| 右前腿链 | 4（front_joint → child1 → child2 → child3） | body 嵌套 4 层 |
| 右后腿链 | 3（rear_joint → child1 → wheel） | body 嵌套 3 层 |

> 每个关节对着一个自由度，**共 14 个关节（含 2 个轮关节）**，对应 `keyframe` 中的 14 个 `qpos` 值。

---

## 第 6 层：约束系统 `<equality>`

### 6.1 概述

```xml
<equality>
  <connect name="Left_loop1" site1="Left_front_site1" site2="Left_rear_site1"
           solref="0.001 1" solimp="0.99 0.99 0.01 0.5 2"/>
  ...
</equality>
```

`<equality>` 定义各种**运动学约束**（equality constraints），用于将两个点/body/自由度绑定在一起。

本例使用 `<connect>` 约束来实现**并联腿的闭环机构**。

### 6.2 `<connect>` —— 位置连接约束

```xml
<connect name="Left_loop1"
         site1="Left_front_site1"
         site2="Left_rear_site1"
         solref="0.001 1"
         solimp="0.99 0.99 0.01 0.5 2"/>
```

`<connect>` 强制 `site1` 和 `site2` 两个点保持重合，从而形成一个**闭环运动链**（closed kinematic loop）。

#### 属性详解

| 属性 | 含义 | 本例值 |
|------|------|--------|
| `name` | 约束名称 | `"Left_loop1"` |
| `site1` | 第一个锚点（site 名称） | `"Left_front_site1"` |
| `site2` | 第二个锚点（site 名称） | `"Left_rear_site1"` |
| `body1` | 替代 site 方式：直接绑 body | — |
| `body2` | 替代 site 方式：直接绑 body | — |
| `anchor` | 连接点的全局坐标（若不使用 body/site） | — |

> **两种连接方式**：
> - **site 方式**：`site1="..." site2="..."`——推荐，site 有可视化，方便调试
> - **body 方式**：`body1="..." body2="..." anchor="x y z"`——直接绑定两个 body 的相对位置

#### `solref` —— 约束刚度与阻尼

```
solref="timeconst damping_ratio"
solref="0.001 1"
```

| 参数 | 值 | 含义 |
|------|-----|------|
| `timeconst` | `0.001` | 时间常数（秒），越小约束越"硬" |
| `damping_ratio` | `1` | 阻尼比（临界阻尼 = 1） |

`timeconst` 决定了约束纠正位置误差的速度。0.001s 是一个**非常刚硬**的设置——误差将在约 1ms 内被纠正。

#### `solimp` —— 约束阻抗（软约束特性）

```
solimp="dmin dmax width midpoint power"
solimp="0.99 0.99 0.01 0.5 2"
```

| 参数 | 值 | 含义 |
|------|-----|------|
| `dmin` | `0.99` | 最小阻尼系数（距离为 0 时） |
| `dmax` | `0.99` | 最大阻尼系数（距离 ≥ width 时） |
| `width` | `0.01` | 过渡宽度 |
| `midpoint` | `0.5` | 过渡中点（归一化） |
| `power` | `2` | 过渡曲线指数 |

> 这些参数定义了约束的**软特性**——当约束即将被违反时，如何平滑地施加力来纠正。

### 6.3 本模型的闭环结构

```
左前腿链（4关节开链）             左后腿链（3关节开链）
  child2_link 上有 site1  ●~~~~~~~~●  rear_link 上有 site1     ← Left_loop1
  child3_link 上有 site2  ●~~~~~~~~●  rear_child1 上有 site2  ← Left_loop2

右前腿链（4关节开链）             右后腿链（3关节开链）
  child2_link 上有 site1  ●~~~~~~~~●  rear_child1 上有 site1  ← Right_loop1
  child2_link 上有 site2  ●~~~~~~~~●  rear_child2 上有 site2  ← Right_loop2
```

通过这 4 个 connect 约束，每条腿前-后两条开链被连接成一个**并联闭环机构**——这正是 RoboMaster 平衡步兵机器人的关键机械结构特征。

> **并联机构的优势**：更高的刚度、更好的负载能力，适合高动态平衡控制。

### 6.4 其他 equality 类型

| 类型 | 语法 | 用途 |
|------|------|------|
| `connect` | `<connect site1="..." site2="..."/>` | 绑定两个点 |
| `weld` | `<weld body1="..." body2="..."/>` | 完全固定两个 body |
| `joint` | `<joint joint1="..." joint2="..."/>` | 耦合两个关节 |
| `tendon` | `<tendon tendon1="..." tendon2="..."/>` | 耦合两根肌腱 |
| `distance` | `<distance geom1="..." geom2="..."/>` | 固定两 geom 间距 |

---

## 第 7 层：驱动器 `<actuator>`

### 7.1 概述

```xml
<actuator>
  <general name="Left_front_joint_actuator" joint="Left_front_joint"
           gainprm="1" ctrlrange="-3.14 3.14" />
  ...
</actuator>
```

`<actuator>` 定义机器的**驱动器（执行器）**——它们是控制信号的入口，将控制力/扭矩施加到关节或 body 上。

### 7.2 `<general>` —— 通用驱动器

本模型全部使用 `<general>` 类型驱动器。

```xml
<general name="Left_front_joint_actuator"
         joint="Left_front_joint"
         gainprm="1"
         ctrlrange="-3.14 3.14" />
```

#### 属性详解

| 属性 | 含义 | 示例值 |
|------|------|--------|
| `name` | 驱动器名称 | `"Left_front_joint_actuator"` |
| `joint` | 驱动目标关节名称 | `"Left_front_joint"` |
| `gainprm` | 控制增益乘数（增益参数） | `1` |
| `ctrlrange` | 控制信号范围（最小值 最大值） | `"-3.14 3.14"`（±π 弧度） |
| `forcerange` | 力/力矩输出范围 | 如 `"-10 10"` |
| `gear` | 传动比（齿轮比） | 正浮点数数组 |

#### `gainprm` —— 增益参数

`gainprm="1"` 表示控制信号直接传递，没有缩放。若设为 `gainprm="2"`，则相同的控制信号会产生两倍的力/力矩。

#### `ctrlrange` —— 控制信号范围

| 驱动器 | ctrlrange | 含义 |
|--------|-----------|------|
| 腿关节 | `"-3.14 3.14"` | 控制信号限制在 ±π（完整一周旋转） |
| 轮关节 | 无 ctrlrange | 无限制（可连续旋转） |
| `Left_Wheel_joint_actuator` | 无 ctrlrange | 轮子可以无限旋转 |

### 7.3 本模型中的驱动器清单

| 驱动器名称 | 目标关节 | 限制 | 类型 |
|-----------|---------|------|------|
| `Left_front_joint_actuator` | `Left_front_joint` | ±π | 腿关节 |
| `Left_rear_joint_actuator` | `Left_rear_joint` | ±π | 腿关节 |
| `Left_Wheel_joint_actuator` | `Left_Wheel_joint` | 无限制 | 轮关节 |
| `Right_front_joint_actuator` | `Right_front_joint` | ±π | 腿关节 |
| `Right_rear_joint_actuator` | `Right_rear_joint` | ±π | 腿关节 |
| `Right_Wheel_joint_actuator` | `Right_Wheel_joint` | ±π | 轮关节 |

**共 6 个驱动器**——每条腿链的**根关节**各 1 个（4 个腿驱动器）+ 2 个轮驱动器。注意前腿的 `child1`、`child2`、`child3` 关节和后腿的 `child1` 关节**没有独立驱动器**——它们通过闭环约束被**被动驱动**。

> **被动关节 vs 主动关节**：只有 6 个主动关节（有 actuator），其余 8 个关节是无驱动器的被动关节，它们的运动由闭环约束和动力学决定。这反映了实际机器人的驱动方案——每个腿链只有根部电机主动驱动，通过并联机构传递到末端。

### 7.4 其他 actuator 类型

| 类型 | 用途 |
|------|------|
| `<motor>` | 标准电机驱动器 |
| `<position>` | 位置伺服驱动器 |
| `<velocity>` | 速度伺服驱动器 |
| `<general>` | 通用驱动器（本模型使用） |
| `<cylinder>` | 液压/气动缸 |
| `<muscle>` | 肌肉模型（肌腱驱动） |

---

## 第 8 层：关键帧 `<keyframe>`

### 8.1 概述

```xml
<keyframe>
  <key name="home" qpos="0 0 0 0 0 0 0 0 0 0 0 0 0 0" />
</keyframe>
```

`<keyframe>` 定义模型的**预置姿态**，可以在仿真运行时快速设置所有关节位置。

### 8.2 `<key>` —— 单个关键帧

```xml
<key name="home" qpos="0 0 0 0 0 0 0 0 0 0 0 0 0 0" />
```

| 属性 | 含义 | 示例 |
|------|------|------|
| `name` | 关键帧名称 | `"home"` |
| `qpos` | 所有关节的目标位置（空格分隔） | 14 个零值 |
| `qvel` | 所有关节的目标速度 | 如 `"0 0 ..."` |
| `act` | 所有驱动器的目标激活值 | 如 `"0 0 ..."` |

### 8.3 qpos 与关节的对应关系

`qpos` 的顺序与**模型中关节的声明顺序**一致：

| 序号 | 关节 | 所属 body | home 值 |
|------|------|----------|---------|
| 1 | `Left_front_joint` | Left_front_link | 0 |
| 2 | `Left_front_child1_joint` | Left_front_child1_link | 0 |
| 3 | `Left_front_child2_joint` | Left_front_child2_link | 0 |
| 4 | `Left_front_child3_joint` | Left_front_child3_link | 0 |
| 5 | `Left_rear_joint` | Left_rear_link | 0 |
| 6 | `Left_rear_child1_joint` | Left_rear_child1_link | 0 |
| 7 | `Left_Wheel_joint` | Left_Wheel_link | 0 |
| 8 | `Right_front_joint` | Right_front_link | 0 |
| 9 | `Right_front_child1_joint` | Right_front_child1_link | 0 |
| 10 | `Right_front_child2_joint` | Right_front_child2_link | 0 |
| 11 | `Right_front_joint3_joint` | Right_front_child3_link | 0 |
| 12 | `Right_rear_joint` | Right_rear_link | 0 |
| 13 | `Right_rear_child1_joint` | Right_rear_child1_link | 0 |
| 14 | `Right_Wheel_joint` | Right_Wheel_link | 0 |

> **home 姿态**：所有关节为 0，即机器人处于"中立站直"的姿态。在 MuJoCo 模拟器中可以通过 `mj_resetDataKeyframe` 函数重置到此姿态。

---

## 附录：完整属性速查表

### A.1 坐标系速查

```
MuJoCo 世界坐标系（右手系）：
  X → 前方（Forward）
  Y → 左方（Left）
  Z → 上方（Up）

旋转方向（右手定则）：
  绕 +X 轴 → 顺时针（从前方看）
  绕 +Y 轴 → 顺时针（从左侧看）
  绕 +Z 轴 → 逆时针（从上方看）
```

### A.2 四元数格式

```
quat="w x y z"    （MuJoCo 使用 wxyz 顺序）
```

与某些库的 `(x, y, z, w)` 顺序不同，注意转换。

### A.3 颜色 RGBA

```
rgba="R G B A"    每个通道 0.0 ~ 1.0
  R = Red（红）
  G = Green（绿）
  B = Blue（蓝）
  A = Alpha（透明度，1=不透明，0=完全透明）
```

### A.4 常用长度单位参考

MuJoCo 本身没有强制单位系统，但惯例使用：

| 物理量 | 习惯单位 |
|--------|---------|
| 长度 | 米（m） |
| 质量 | 千克（kg） |
| 时间 | 秒（s） |
| 角度 | 弧度（rad）或度（°），取决于 compiler angle 设置 |
| 力 | 牛顿（N） |
| 力矩 | 牛·米（N·m） |

### A.5 MJCF 顶级元素一览

| 元素 | 层级 | 用途 |
|------|------|------|
| `<mujoco>` | 根 | 模型根容器 |
| `<default>` | 顶级 | 全局默认属性 |
| `<compiler>` | 顶级 | 解析器编译选项 |
| `<option>` | 顶级 | 物理仿真参数（时间步、重力等） |
| `<asset>` | 顶级 | 外部资源声明 |
| `<worldbody>` | 顶级 | 物理世界及刚体树 |
| `<equality>` | 顶级 | 运动学约束 |
| `<tendon>` | 顶级 | 肌腱/软体约束 |
| `<actuator>` | 顶级 | 驱动器 |
| `<sensor>` | 顶级 | 传感器 |
| `<keyframe>` | 顶级 | 预置姿态 |
| `<contact>` | 顶级 | 接触对排除/包含 |
| `<size>` | 顶级 | 内存分配大小 |

---

> **文档版本**：v1.0 | **基于文件**：`COD-2026RoboMaster-Balance.xml` | **更新日期**：2026-06-11
