# Complete Docker build for UAV Swarm Coordinator
FROM ubuntu:22.04

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Install build AND runtime dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libboost-all-dev \
    pkg-config \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python visualization dependencies
RUN pip3 install matplotlib pandas numpy

# Copy source code
COPY . /app
WORKDIR /app

# Build the application inside Docker
RUN mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_GUI=OFF && \
    make swarmgrid_app

# Create non-root user
RUN useradd -m -s /bin/bash swarmgrid

# Create startup script
RUN echo '#!/bin/bash\n\
cd /app\n\
echo "🚁 Running UAV Swarm Simulation..."\n\
./build/swarmgrid_app --out-metrics /tmp/metrics.json --out-trace /tmp/trace.csv "$@"\n\
echo "📊 Generating visualization..."\n\
python3 visualize.py /tmp/trace.csv /tmp/metrics.json /tmp/output.png\n\
echo "✅ Simulation complete!"\n\
echo "📈 Files saved in /tmp/"\n\
ls -la /tmp/*.{json,csv,png} 2>/dev/null || echo "Check /tmp/ directory for output files"\n\
' > /usr/local/bin/run-simulation.sh && chmod +x /usr/local/bin/run-simulation.sh

USER swarmgrid
WORKDIR /app

CMD ["run-simulation.sh", "--map", "maps/demo.txt", "--agents", "8"]