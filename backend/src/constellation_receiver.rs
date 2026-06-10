use std::collections::HashMap;

use anyhow::Result;
use chrono::Utc;
use serde::Deserialize;
use tokio::net::UdpSocket;
use tokio::sync::mpsc;
use tracing::{debug, warn, info};

use crate::models::{TelemetryData, TleData};
use crate::config::ReorderBufferConfig;

#[derive(Deserialize)]
struct UdpTelemetryPacket {
    pub sequence_number: u64,
    pub satellite_id: u16,
    pub timestamp: String,
    pub semi_major_axis: f64,
    pub eccentricity: f64,
    pub inclination: f64,
    pub raan: f64,
    pub arg_perigee: f64,
    pub true_anomaly: f64,
    pub quat_w: f64,
    pub quat_x: f64,
    pub quat_y: f64,
    pub quat_z: f64,
    pub propellant_remaining: f64,
    pub position_x: f64,
    pub position_y: f64,
    pub position_z: f64,
    pub velocity_x: f64,
    pub velocity_y: f64,
    pub velocity_z: f64,
}

impl UdpTelemetryPacket {
    fn into_telemetry_data(self) -> Result<TelemetryData> {
        let timestamp = self.timestamp.parse::<chrono::DateTime<Utc>>()?;
        Ok(TelemetryData {
            satellite_id: self.satellite_id,
            sequence_number: self.sequence_number,
            timestamp,
            semi_major_axis: self.semi_major_axis,
            eccentricity: self.eccentricity,
            inclination: self.inclination,
            raan: self.raan,
            arg_perigee: self.arg_perigee,
            true_anomaly: self.true_anomaly,
            quat_w: self.quat_w,
            quat_x: self.quat_x,
            quat_y: self.quat_y,
            quat_z: self.quat_z,
            propellant_remaining: self.propellant_remaining,
            position_x: self.position_x,
            position_y: self.position_y,
            position_z: self.position_z,
            velocity_x: self.velocity_x,
            velocity_y: self.velocity_y,
            velocity_z: self.velocity_z,
        })
    }
}

#[derive(Deserialize)]
struct UdpTlePacket {
    pub satellite_id: u16,
    pub timestamp: String,
    pub norad_id: String,
    pub line1: String,
    pub line2: String,
    pub epoch_year: f64,
    pub epoch_day: f64,
    pub mean_motion: f64,
    pub eccentricity_tle: f64,
    pub inclination_tle: f64,
    pub raan_tle: f64,
    pub arg_perigee_tle: f64,
    pub mean_anomaly_tle: f64,
    pub bstar: f64,
}

impl UdpTlePacket {
    fn into_tle_data(self) -> Result<TleData> {
        let timestamp = self.timestamp.parse::<chrono::DateTime<Utc>>()?;
        Ok(TleData {
            satellite_id: self.satellite_id,
            timestamp,
            norad_id: self.norad_id,
            line1: self.line1,
            line2: self.line2,
            epoch_year: self.epoch_year,
            epoch_day: self.epoch_day,
            mean_motion: self.mean_motion,
            eccentricity_tle: self.eccentricity_tle,
            inclination_tle: self.inclination_tle,
            raan_tle: self.raan_tle,
            arg_perigee_tle: self.arg_perigee_tle,
            mean_anomaly_tle: self.mean_anomaly_tle,
            bstar: self.bstar,
        })
    }
}

struct ReorderBuffer {
    buffers: HashMap<u16, Vec<(u64, TelemetryData)>>,
    max_buffer_size: usize,
    last_delivered: HashMap<u16, u64>,
}

impl ReorderBuffer {
    fn new(max_buffer_size: usize) -> Self {
        Self {
            buffers: HashMap::new(),
            max_buffer_size,
            last_delivered: HashMap::new(),
        }
    }

    fn insert(&mut self, data: TelemetryData) -> Vec<TelemetryData> {
        let sat_id = data.satellite_id;
        let seq = data.sequence_number;

        let buffer = self.buffers.entry(sat_id).or_default();
        buffer.push((seq, data));

        if buffer.len() > self.max_buffer_size {
            buffer.sort_by_key(|(s, _)| *s);
            let (_, oldest) = buffer.remove(0);
            let oldest_seq = oldest.sequence_number;
            warn!(
                satellite_id = sat_id,
                sequence_number = oldest_seq,
                "Buffer overflow, force-delivering oldest packet (gap fill)"
            );
            let last = self.last_delivered.entry(sat_id).or_insert(0);
            if oldest_seq > *last {
                *last = oldest_seq;
            }
            let mut delivered = vec![oldest];
            delivered.extend(self.drain_contiguous(sat_id));
            return delivered;
        }

        self.drain_contiguous(sat_id)
    }

    fn drain_contiguous(&mut self, sat_id: u16) -> Vec<TelemetryData> {
        let buffer = match self.buffers.get_mut(&sat_id) {
            Some(b) => b,
            None => return Vec::new(),
        };

        if buffer.is_empty() {
            return Vec::new();
        }

        buffer.sort_by_key(|(s, _)| *s);

        let last = self.last_delivered.entry(sat_id).or_insert(0);
        let next_expected = *last + 1;

        let mut delivered = Vec::new();

        let first_seq = buffer.first().map(|(s, _)| *s).unwrap();
        if first_seq <= next_expected {
            let mut i = 0;
            while i < buffer.len() {
                let (seq, _) = &buffer[i];
                if *seq <= *last {
                    i += 1;
                    continue;
                }
                if *seq == *last + 1 {
                    let (_, data) = buffer.remove(i);
                    *last = data.sequence_number;
                    delivered.push(data);
                } else {
                    break;
                }
            }
        }

        if buffer.is_empty() {
            self.buffers.remove(&sat_id);
        }

        delivered
    }
}

pub struct ConstellationReceiver {
    telemetry_port: u16,
    tle_port: u16,
    reorder_buffer_config: ReorderBufferConfig,
}

impl ConstellationReceiver {
    pub fn new(telemetry_port: u16, tle_port: u16, reorder_buffer_config: ReorderBufferConfig) -> Self {
        Self {
            telemetry_port,
            tle_port,
            reorder_buffer_config,
        }
    }

    pub async fn run(self, telemetry_tx: mpsc::Sender<TelemetryData>, tle_tx: mpsc::Sender<TleData>) {
        let telemetry_handle = Self::run_telemetry_receiver(self.telemetry_port, self.reorder_buffer_config.max_buffer_size, telemetry_tx);
        let tle_handle = Self::run_tle_receiver(self.tle_port, tle_tx);

        tokio::select! {
            result = telemetry_handle => {
                if let Err(e) = result {
                    warn!(error = %e, "Telemetry receiver exited with error");
                }
            }
            result = tle_handle => {
                if let Err(e) = result {
                    warn!(error = %e, "TLE receiver exited with error");
                }
            }
        }
    }

    async fn run_telemetry_receiver(port: u16, max_buffer_size: usize, tx: mpsc::Sender<TelemetryData>) -> Result<()> {
        let addr = format!("0.0.0.0:{}", port);
        let socket = UdpSocket::bind(&addr).await?;
        info!("Telemetry UDP receiver listening on {}", addr);

        let mut buf = [0u8; 4096];
        let mut reorder_buffer = ReorderBuffer::new(max_buffer_size);

        loop {
            let (len, _addr) = socket.recv_from(&mut buf).await?;
            let data = &buf[..len];

            match serde_json::from_slice::<UdpTelemetryPacket>(data) {
                Ok(packet) => {
                    debug!(
                        satellite_id = packet.satellite_id,
                        sequence_number = packet.sequence_number,
                        "Received UDP telemetry packet"
                    );

                    match packet.into_telemetry_data() {
                        Ok(telemetry) => {
                            let ready = reorder_buffer.insert(telemetry);
                            for data in ready {
                                if tx.send(data).await.is_err() {
                                    warn!("Telemetry receiver channel closed, shutting down");
                                    return Ok(());
                                }
                            }
                        }
                        Err(e) => {
                            warn!(error = %e, "Failed to parse timestamp in telemetry packet");
                        }
                    }
                }
                Err(e) => {
                    warn!(error = %e, "Failed to parse telemetry JSON packet");
                }
            }
        }
    }

    async fn run_tle_receiver(port: u16, tx: mpsc::Sender<TleData>) -> Result<()> {
        let addr = format!("0.0.0.0:{}", port);
        let socket = UdpSocket::bind(&addr).await?;
        info!("TLE UDP receiver listening on {}", addr);

        let mut buf = [0u8; 8192];

        loop {
            let (len, _addr) = socket.recv_from(&mut buf).await?;
            let data = &buf[..len];

            match serde_json::from_slice::<UdpTlePacket>(data) {
                Ok(packet) => {
                    debug!(
                        satellite_id = packet.satellite_id,
                        norad_id = %packet.norad_id,
                        "Received UDP TLE packet"
                    );

                    match packet.into_tle_data() {
                        Ok(tle) => {
                            if tx.send(tle).await.is_err() {
                                warn!("TLE receiver channel closed, shutting down");
                                return Ok(());
                            }
                        }
                        Err(e) => {
                            warn!(error = %e, "Failed to parse timestamp in TLE packet");
                        }
                    }
                }
                Err(e) => {
                    warn!(error = %e, "Failed to parse TLE JSON packet");
                }
            }
        }
    }
}
