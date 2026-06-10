# 卫星星座轨道控制与碰撞预警系统

## 系统架构

```
┌─────────────────┐     UDP/9090      ┌──────────────────────────┐
│  卫星星座模拟器   │────遥测──────────▶│  constellation_receiver   │
│  (Python, 80星)  │     UDP/9091      │  UDP接收 + 重排缓冲       │
│                  │────TLE───────────▶│  序列号排序 + gap fill     │
└─────────────────┘                   └──────────┬───────────────┘
                                                  │ mpsc channel
                                                  ▼
                                       ┌──────────────────────────┐
                                       │  telemetry_fanout_task    │
                                       │  ClickHouse写入 + 状态缓存 │
                                       └─────┬────────────┬───────┘
                                             │            │
                                    mpsc      │            │ mpsc
                                             ▼            ▼
                                 ┌──────────────────┐  (TLE同路)
                                 │ collision_predictor│
                                 │ SGP4+数值双传播器  │
                                 │ TCA黄金分割搜索   │
                                 │ Chan碰撞概率模型  │
                                 └────────┬─────────┘
                                          │ mpsc (CollisionAnalysis)
                                          ▼
                                 ┌──────────────────┐
                                 │  alarm_commander  │────HTTP────▶ 地面站
                                 │  告警评估/分级    │
                                 │  level=2自动规避  │
                                 └────────┬─────────┘
                                          │ mpsc (OptimizerRequest)
                                          ▼
                                 ┌──────────────────┐
                                 │orbit_optimizer_svc│
                                 │协同进化(岛模型)   │
                                 │大气阻力+推进剂约束│
                                 └──────────────────┘

┌──────────┐  REST/WS   ┌──────────────────────────────────────┐
│  前端     │◀──────────▶│  Axum HTTP (8080)                    │
│ Three.js │             │  /metrics → Prometheus               │
│ +Canvas  │             │  /ws → WebSocket实时位置推送          │
└──────────┘             └──────────────────────────────────────┘
                                      │
                                      ▼
                           ┌──────────────────┐
                           │   ClickHouse      │
                           │ 5表+3物化视图      │
                           │ 按月分区+TTL       │
                           └──────────────────┘

┌──────────┐              ┌──────────┐
│Prometheus│◀──scrape────│  Backend  │
│  :9099   │              │  /metrics │
└────┬─────┘              └──────────┘
     │
     ▼
┌──────────┐
│ Grafana  │
│  :3000   │
└──────────┘
```

### 模块说明

| 模块 | 文件 | 职责 |
|------|------|------|
| constellation_receiver | `backend/src/constellation_receiver.rs` | UDP遥测/TLE接收，ReorderBuffer序列号排序 |
| collision_predictor | `backend/src/collision_predictor.rs` | SGP4+RK4双传播器，TCA搜索，Chan碰撞概率 |
| orbit_optimizer_service | `backend/src/orbit_optimizer_service.rs` | 协同进化岛模型轨道保持，大气阻力模型 |
| alarm_commander | `backend/src/alarm_commander.rs` | 告警评估分级，规避机动计算，地面站推送 |
| api | `backend/src/api.rs` | Axum REST+WebSocket+Prometheus /metrics |
| config | `backend/src/config.rs` + `backend/config.toml` | 全参数外置：SGP4/数值传播/碰撞/优化/网络 |
| models | `backend/src/models.rs` | 数据结构：TelemetryData, TleData, CollisionAlert等 |
| orbit_3d_viewer | `frontend/orbit_3d_viewer.js` | Three.js 3D地球/卫星InstancedMesh/轨道LOD |
| sat_detail | `frontend/sat_detail.js` | 详情面板/推进剂图表/告警卡片 |

## 部署

### Docker Compose（推荐）

```bash
# 启动所有服务
docker-compose up -d

# 查看日志
docker-compose logs -f backend
docker-compose logs -f simulator

# 停止
docker-compose down
```

服务端口：
- **前端**: http://localhost:8080
- **ClickHouse HTTP**: http://localhost:8123
- **Prometheus**: http://localhost:9099
- **Grafana**: http://localhost:3000 (admin/admin)

### 手动部署

```bash
# 1. 启动ClickHouse
clickhouse-server --config-file=/etc/clickhouse-server/config.xml
clickhouse-client < clickhouse/init.sql

# 2. 编译Rust后端
cd backend
cargo build --release
./target/release/satellite-constellation-system

# 3. 启动模拟器
cd simulator
python constellation_simulator.py --host 127.0.0.1
```

## 卫星星座模拟器用法

### 基本用法

```bash
# 默认：80颗卫星，30秒间隔
python constellation_simulator.py

# Docker环境
docker-compose up simulator
```

### 命令行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--num-sats` | 80 | 卫星数量 |
| `--interval` | 30.0 | 遥测上报间隔(秒) |
| `--host` | UDP_HOST环境变量或127.0.0.1 | 后端地址 |
| `--telemetry-port` | TELEMETRY_PORT环境变量或9090 | 遥测UDP端口 |
| `--tle-port` | TLE_PORT环境变量或9091 | TLE UDP端口 |
| `--threads` | 10 | 工作线程数 |
| `--perturbation` | None | 轨道摄动场景 |
| `--collision` | None | 碰撞交会场景 |
| `--collision-time` | 300 | 碰撞注入延迟(秒) |

### 轨道摄动注入

```bash
# 太阳风暴：60秒后大气阻力×10，B*增大，SMA随机扰动
python constellation_simulator.py --perturbation solar_storm

# 大气阻力渐增：60秒后5分钟内阻力从1×线性增至5×
python constellation_simulator.py --perturbation drag_increase
```

### 碰撞交会注入

```bash
# 对头交会：SAT-001和SAT-002移至近同轨道对向运行
python constellation_simulator.py --collision head_on --collision-time 120

# 交叉交会：SAT-001和SAT-041(SAT-041在第2轨道面)RAAN收敛
python constellation_simulator.py --collision crossing --collision-time 60

# 组合：太阳风暴+对头交会
python constellation_simulator.py --perturbation solar_storm --collision head_on
```

### Walker Delta星座构型

- 5个轨道面 × 16颗卫星/面
- 轨道高度：550km (SMA=6928.137km)
- 倾角：53°
- RAAN间隔：72° (360°/5面)
- 面内真近点角间隔：22.5° (360°/16星)
- NORAD ID：40001-40080

## ClickHouse数据模型

| 表 | 分区 | TTL | 说明 |
|----|------|-----|------|
| telemetry | toYYYYMM(timestamp) | 90天 | 遥测原始数据 |
| tle_data | 无 | 30天 | TLE轨道根数 |
| collision_alerts | toYYYYMM(timestamp) | 180天 | 碰撞预警记录 |
| orbit_maneuvers | 无 | 365天 | 轨道机动记录 |
| propellant_history | toYYYYMM(timestamp) | 180天 | 推进剂消耗历史 |

物化视图：
- `mv_latest_telemetry` → 每星最新遥测快照
- `mv_active_alert_stats` → 活跃告警分钟级统计
- `mv_propellant_hourly` → 推进剂小时级聚合

## Prometheus指标

| 指标 | 类型 | 说明 |
|------|------|------|
| `telemetry_received_total` | Counter | 接收遥测包总数 |
| `active_satellites` | Gauge | 活跃卫星数 |
| `active_alerts` | Gauge | 活跃告警数 |
| `collision_analysis_duration_seconds` | Histogram | 碰撞分析周期耗时 |
| `avoidance_computations_total` | Counter | 规避机动计算次数 |
| `http_requests_total` | Counter | HTTP请求总数 |

## 配置文件

`backend/config.toml` 包含8个配置节：

- `[sgp4]` — 引力常数(J2-J6)、开普勒求解器参数
- `[numerical_propagator]` — RK4步长、SRP、大气阻力F10.7、散度阈值
- `[collision]` — 扫描步数、黄金分割迭代、告警阈值(1e-4/1e-3)、sigma
- `[optimizer]` — 种群/代数/岛屿/迁移/ΔV范围/Isp
- `[atmosphere]` — 大气密度/标高/Cd
- `[ground_station]` — 地面站URL/超时
- `[network]` — 端口/ClickHouse连接
- `[reorder_buffer]` — 重排缓冲区大小

Docker部署使用 `backend/config.docker.toml`（clickhouse_url 指向 `http://clickhouse:8123`）。

## 前端技术

- Three.js r128 + OrbitControls
- 3个InstancedMesh替代240个独立Mesh（draw call 240→3）
- LOD轨道：safe=24段, warning=48段, danger=96段
- WebSocket实时位置推送（1秒/次）
- Gzip压缩（tower-http CompressionLayer）
- 代码拆分：`orbit_3d_viewer.js`（3D场景）+ `sat_detail.js`（详情面板）
