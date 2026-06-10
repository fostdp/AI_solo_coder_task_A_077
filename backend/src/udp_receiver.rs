use anyhow::Result;
use tokio::net::UdpSocket;
use tokio::sync::mpsc;
use tracing::{debug, warn};

use crate::models::TelemetryData;

pub async fn start_udp_receiver(tx: mpsc::Sender<TelemetryData>) -> Result<()> {
    let socket = UdpSocket::bind("0.0.0.0:9090").await?;
    tracing::info!("UDP receiver listening on 0.0.0.0:9090");

    let mut buf = [0u8; 4096];

    loop {
        let (len, _addr) = socket.recv_from(&mut buf).await?;
        let data = &buf[..len];

        match serde_json::from_slice::<TelemetryData>(data) {
            Ok(telemetry) => {
                debug!(satellite_id = telemetry.satellite_id, "Received telemetry packet");
                if tx.send(telemetry).await.is_err() {
                    warn!("Receiver channel closed, shutting down UDP receiver");
                    break;
                }
            }
            Err(e) => {
                warn!(error = %e, "Failed to parse telemetry JSON packet");
            }
        }
    }

    Ok(())
}
