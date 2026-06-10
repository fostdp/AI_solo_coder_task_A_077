FROM rust:1.75-slim AS builder
RUN apt-get update && apt-get install -y musl-tools && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY Cargo.toml Cargo.lock ./
COPY src/ ./src/
COPY config.toml ./
RUN cargo build --release

FROM debian:bookworm-slim
ENV RUST_LOG=info
WORKDIR /app
COPY --from=builder /app/target/release/satellite-constellation-system ./
COPY config.toml ./
COPY frontend/ ./frontend/
EXPOSE 8080 9090 9091
CMD ["./satellite-constellation-system"]
