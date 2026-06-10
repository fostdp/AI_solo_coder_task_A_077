-- ============================================================
-- 卫星星座轨道控制与碰撞预警系统 - ClickHouse 初始化脚本
-- 适用于80颗LEO卫星星座的轨道管理、碰撞预警及推进剂跟踪
-- ============================================================

CREATE DATABASE IF NOT EXISTS satellite_constellation;

USE satellite_constellation;

-- ============================================================
-- 1. 遥测数据表 - 存储通过UDP接收的卫星遥测数据
--    包含轨道六根数、姿态四元数、推进剂余量、ECI位置速度
-- ============================================================
CREATE TABLE IF NOT EXISTS telemetry
(
    satellite_id        UInt16    COMMENT '卫星编号 1-80',
    sequence_number     UInt64    COMMENT '数据包序列号',
    timestamp           DateTime  COMMENT '遥测时间戳',
    semi_major_axis     Float64   COMMENT '半长轴 a (km)',
    eccentricity        Float64   COMMENT '偏心率 e',
    inclination         Float64   COMMENT '轨道倾角 i (rad)',
    raan                Float64   COMMENT '升交点赤经 Ω (rad)',
    arg_perigee         Float64   COMMENT '近地点幅角 ω (rad)',
    true_anomaly        Float64   COMMENT '真近点角 ν (rad)',
    quat_w              Float64   COMMENT '姿态四元数 w 分量',
    quat_x              Float64   COMMENT '姿态四元数 x 分量',
    quat_y              Float64   COMMENT '姿态四元数 y 分量',
    quat_z              Float64   COMMENT '姿态四元数 z 分量',
    propellant_remaining Float64  COMMENT '剩余推进剂 (kg)',
    position_x          Float64   COMMENT 'ECI位置 X (km)',
    position_y          Float64   COMMENT 'ECI位置 Y (km)',
    position_z          Float64   COMMENT 'ECI位置 Z (km)',
    velocity_x          Float64   COMMENT 'ECI速度 Vx (km/s)',
    velocity_y          Float64   COMMENT 'ECI速度 Vy (km/s)',
    velocity_z          Float64   COMMENT 'ECI速度 Vz (km/s)'
)
ENGINE = MergeTree()
PARTITION BY toYYYYMM(timestamp)
ORDER BY (satellite_id, timestamp);

-- 遥测表索引：按时间范围查询优化
ALTER TABLE telemetry ADD INDEX idx_timestamp timestamp TYPE minmax GRANULARITY 4;
-- 遥测表索引：按半长轴范围筛选（轨道高度异常检测）
ALTER TABLE telemetry ADD INDEX idx_semi_major_axis semi_major_axis TYPE minmax GRANULARITY 4;

-- ============================================================
-- 2. TLE轨道根数表 - 存储SGP4传播所需的TLE数据
--    用于轨道预报和空间目标编目
-- ============================================================
CREATE TABLE IF NOT EXISTS tle_data
(
    satellite_id        UInt16    COMMENT '卫星编号 1-80',
    timestamp           DateTime  COMMENT 'TLE入库时间',
    norad_id            String    COMMENT 'NORAD编目号',
    line1               String    COMMENT 'TLE第一行',
    line2               String    COMMENT 'TLE第二行',
    epoch_year          Float64   COMMENT '历元年份',
    epoch_day           Float64   COMMENT '历元日',
    mean_motion         Float64   COMMENT '平均运动 (rev/day)',
    eccentricity_tle    Float64   COMMENT 'TLE偏心率',
    inclination_tle     Float64   COMMENT 'TLE轨道倾角 (rad)',
    raan_tle            Float64   COMMENT 'TLE升交点赤经 (rad)',
    arg_perigee_tle     Float64   COMMENT 'TLE近地点幅角 (rad)',
    mean_anomaly_tle    Float64   COMMENT 'TLE平近点角 (rad)',
    bstar               Float64   COMMENT 'B*阻力系数'
)
ENGINE = MergeTree()
ORDER BY (satellite_id, timestamp);

-- TLE表索引：按NORAD编号快速检索
ALTER TABLE tle_data ADD INDEX idx_norad_id norad_id TYPE tokenbf_v1(512, 3, 0) GRANULARITY 1;

-- ============================================================
-- 3. 碰撞预警表 - 存储交会分析与碰撞告警记录
--    alert_level: 1=预警(碰撞概率>1e-4), 2=紧急(碰撞概率>1e-3)
-- ============================================================
CREATE TABLE IF NOT EXISTS collision_alerts
(
    alert_id            UUID      COMMENT '告警唯一标识',
    timestamp           DateTime  COMMENT '告警生成时间',
    satellite_id_1      UInt16    COMMENT '主星编号',
    satellite_id_2      UInt16    COMMENT '副星/空间目标编号',
    tca                 DateTime  COMMENT '最近交会时刻 TCA',
    miss_distance       Float64   COMMENT '最小距离 (km)',
    collision_probability Float64  COMMENT '碰撞概率',
    alert_level         UInt8     COMMENT '告警等级: 1=预警, 2=紧急',
    status              String    COMMENT '状态: active/acknowledged/resolved',
    maneuver_planned    UInt8     COMMENT '是否计划规避机动: 0=否, 1=是'
)
ENGINE = MergeTree()
ORDER BY (timestamp, alert_level);

-- 碰撞预警索引：按告警等级快速筛选紧急告警
ALTER TABLE collision_alerts ADD INDEX idx_alert_level alert_level TYPE set(4) GRANULARITY 1;
-- 碰撞预警索引：按主星编号检索该卫星的所有碰撞预警
ALTER TABLE collision_alerts ADD INDEX idx_satellite_id_1 satellite_id_1 TYPE set(80) GRANULARITY 1;
-- 碰撞预警索引：按状态筛选活跃告警
ALTER TABLE collision_alerts ADD INDEX idx_status status TYPE set(3) GRANULARITY 1;
-- 碰撞预警索引：按碰撞概率范围筛选高危预警
ALTER TABLE collision_alerts ADD INDEX idx_collision_probability collision_probability TYPE minmax GRANULARITY 4;

-- ============================================================
-- 4. 轨道机动表 - 存储轨道维持与规避机动记录
--    机动类型: station_keeping=位保, collision_avoidance=碰撞规避, phasing=相位调整
-- ============================================================
CREATE TABLE IF NOT EXISTS orbit_maneuvers
(
    maneuver_id             UUID      COMMENT '机动唯一标识',
    satellite_id            UInt16    COMMENT '卫星编号',
    timestamp               DateTime  COMMENT '机动时间',
    maneuver_type           String    COMMENT '机动类型: station_keeping/collision_avoidance/phasing',
    delta_v_x               Float64   COMMENT '速度增量 ΔVx (m/s)',
    delta_v_y               Float64   COMMENT '速度增量 ΔVy (m/s)',
    delta_v_z               Float64   COMMENT '速度增量 ΔVz (m/s)',
    fuel_cost               Float64   COMMENT '燃料消耗 (kg)',
    target_semi_major_axis  Float64   COMMENT '目标半长轴 (km)',
    target_inclination      Float64   COMMENT '目标轨道倾角 (rad)',
    executed                UInt8     COMMENT '是否已执行: 0=未执行, 1=已执行'
)
ENGINE = MergeTree()
ORDER BY (satellite_id, timestamp);

-- 轨道机动索引：按机动类型筛选
ALTER TABLE orbit_maneuvers ADD INDEX idx_maneuver_type maneuver_type TYPE set(3) GRANULARITY 1;
-- 轨道机动索引：按执行状态筛选
ALTER TABLE orbit_maneuvers ADD INDEX idx_executed executed TYPE set(2) GRANULARITY 1;

-- ============================================================
-- 5. 推进剂历史表 - 推进剂消耗跟踪与寿命预估
--    用于卫星寿命管理和补加规划
-- ============================================================
CREATE TABLE IF NOT EXISTS propellant_history
(
    satellite_id            UInt16    COMMENT '卫星编号',
    timestamp               DateTime  COMMENT '记录时间',
    propellant_remaining    Float64   COMMENT '剩余推进剂 (kg)',
    consumption_rate        Float64   COMMENT '消耗速率 (kg/hour)',
    estimated_lifetime_hours Float64  COMMENT '预估剩余寿命 (小时)'
)
ENGINE = MergeTree()
ORDER BY (satellite_id, timestamp);

-- 推进剂历史索引：按剩余量范围筛选（低燃料预警）
ALTER TABLE propellant_history ADD INDEX idx_propellant_remaining propellant_remaining TYPE minmax GRANULARITY 4;

-- ============================================================
-- 物化视图：每颗卫星最新遥测数据快照
--    用于实时监控面板，避免全表扫描
-- ============================================================
CREATE MATERIALIZED VIEW IF NOT EXISTS mv_latest_telemetry
TO satellite_constellation.latest_telemetry
AS
SELECT
    satellite_id,
    argMax(sequence_number, timestamp)     AS sequence_number,
    argMax(timestamp, timestamp)        AS timestamp,
    argMax(semi_major_axis, timestamp)  AS semi_major_axis,
    argMax(eccentricity, timestamp)     AS eccentricity,
    argMax(inclination, timestamp)      AS inclination,
    argMax(raan, timestamp)             AS raan,
    argMax(arg_perigee, timestamp)      AS arg_perigee,
    argMax(true_anomaly, timestamp)     AS true_anomaly,
    argMax(propellant_remaining, timestamp) AS propellant_remaining,
    argMax(position_x, timestamp)       AS position_x,
    argMax(position_y, timestamp)       AS position_y,
    argMax(position_z, timestamp)       AS position_z,
    argMax(velocity_x, timestamp)       AS velocity_x,
    argMax(velocity_y, timestamp)       AS velocity_y,
    argMax(velocity_z, timestamp)       AS velocity_z
FROM satellite_constellation.telemetry
GROUP BY satellite_id;

-- 物化视图目标表：每颗卫星最新遥测快照
CREATE TABLE IF NOT EXISTS latest_telemetry
(
    satellite_id         UInt16,
    sequence_number      UInt64,
    timestamp            DateTime,
    semi_major_axis      Float64,
    eccentricity         Float64,
    inclination          Float64,
    raan                 Float64,
    arg_perigee          Float64,
    true_anomaly         Float64,
    propellant_remaining Float64,
    position_x           Float64,
    position_y           Float64,
    position_z           Float64,
    velocity_x           Float64,
    velocity_y           Float64,
    velocity_z           Float64
)
ENGINE = AggregatingMergeTree()
ORDER BY satellite_id;

-- ============================================================
-- 物化视图：活跃碰撞告警统计（按告警等级和主星分组）
--    用于告警看板实时展示
-- ============================================================
CREATE MATERIALIZED VIEW IF NOT EXISTS mv_active_alert_stats
TO satellite_constellation.active_alert_stats
AS
SELECT
    toStartOfMinute(timestamp) AS minute,
    satellite_id_1,
    alert_level,
    count()                    AS alert_count,
    max(collision_probability) AS max_collision_prob,
    min(miss_distance)         AS min_miss_distance
FROM satellite_constellation.collision_alerts
WHERE status = 'active'
GROUP BY minute, satellite_id_1, alert_level;

-- 物化视图目标表：活跃告警统计
CREATE TABLE IF NOT EXISTS active_alert_stats
(
    minute               DateTime,
    satellite_id_1       UInt16,
    alert_level          UInt8,
    alert_count          UInt64,
    max_collision_prob   Float64,
    min_miss_distance    Float64
)
ENGINE = AggregatingMergeTree()
ORDER BY (minute, satellite_id_1, alert_level);

-- ============================================================
-- 物化视图：推进剂消耗趋势（小时级聚合）
--    用于燃料消耗异常检测和寿命预测
-- ============================================================
CREATE MATERIALIZED VIEW IF NOT EXISTS mv_propellant_hourly
TO satellite_constellation.propellant_hourly
AS
SELECT
    satellite_id,
    toStartOfHour(timestamp)            AS hour,
    avg(propellant_remaining)           AS avg_propellant,
    max(propellant_remaining)           AS max_propellant,
    min(propellant_remaining)           AS min_propellant,
    avg(consumption_rate)               AS avg_consumption_rate,
    min(estimated_lifetime_hours)       AS min_estimated_lifetime
FROM satellite_constellation.propellant_history
GROUP BY satellite_id, hour;

-- 物化视图目标表：推进剂小时级聚合
CREATE TABLE IF NOT EXISTS propellant_hourly
(
    satellite_id            UInt16,
    hour                    DateTime,
    avg_propellant          Float64,
    max_propellant          Float64,
    min_propellant          Float64,
    avg_consumption_rate    Float64,
    min_estimated_lifetime  Float64
)
ENGINE = AggregatingMergeTree()
ORDER BY (satellite_id, hour);
