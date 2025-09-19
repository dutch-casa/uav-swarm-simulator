# 🚁 UAV Swarm Coordinator - Docker Deployment

This project includes complete Docker containerization for cross-platform deployment with **automated visualization**. Run realistic multi-agent pathfinding simulations anywhere!

## 🚀 Quick Start

### Method 1: Docker Compose (Recommended)

```bash
# Clone the repository
git clone <repository-url>
cd uav-swarm-coordinator

# Create output directory
mkdir -p output

# Run demo simulation with visualization
docker-compose up swarmgrid-viz

# View results
open output/output.png  # macOS
# or
xdg-open output/output.png  # Linux
```

### Method 2: Direct Docker

```bash
# Build the image
docker build -t swarmgrid .

# Run simulation
docker run --rm -v $(pwd)/output:/tmp swarmgrid

# View generated visualization
open output/output.png
```

## 📊 What You Get

After running, check the `output/` directory for:

- **📈 `output.png`** - Beautiful visualization showing agent paths and metrics
- **📋 `metrics.json`** - Complete simulation statistics
- **📄 `trace.csv`** - Detailed agent movement traces

## 🎯 Available Scenarios

### Demo Scenario (Balanced)
```bash
docker-compose up swarmgrid-viz
```
- 10x10 grid with 8 agents
- Demonstrates coordination and pathfinding

### Stress Test (High Complexity)
```bash
docker-compose up swarmgrid-stress
```
- Narrow corridor forcing agent coordination
- 6 agents navigating bottlenecks

### Custom Scenarios
```bash
# Large complex scenario
docker-compose run swarmgrid-custom run-simulation.sh --map maps/large.txt --agents 12

# High drop rate test
docker-compose run swarmgrid-custom run-simulation.sh --map maps/demo.txt --agents 8 --drop 0.2

# Low latency test
docker-compose run swarmgrid-custom run-simulation.sh --map maps/demo.txt --agents 8 --latency 10
```

## 🛠️ Advanced Usage

### Available Maps
- `maps/demo.txt` - 10x10 balanced scenario (8 agents)
- `maps/corridor.txt` - Narrow passage stress test (6 agents)
- `maps/small.txt` - Simple 5x5 grid (4 agents)
- `maps/large.txt` - Complex 15x15 scenario (12 agents)

### CLI Parameters
```bash
run-simulation.sh [OPTIONS]

Options:
  --map PATH              Map file path (required)
  --agents N              Number of agents (default: 8)
  --seed N                Random seed for reproducibility
  --drop RATE             Message drop probability [0-1] (default: 0.05)
  --latency MS            Network latency in ms (default: 40)
  --jitter MS             Network jitter in ms (default: 10)
  --max-steps N           Maximum simulation steps (default: 300)
  --verbose               Enable verbose logging
```

### Examples

```bash
# Reproducible run with specific seed
docker-compose run swarmgrid-custom run-simulation.sh --map maps/demo.txt --seed 12345

# High network issues simulation
docker-compose run swarmgrid-custom run-simulation.sh --map maps/demo.txt --drop 0.3 --latency 100

# Quick test with fewer steps
docker-compose run swarmgrid-custom run-simulation.sh --map maps/small.txt --max-steps 50
```

## 📈 Understanding the Visualization

The generated `output.png` contains:

### Left Panel: Agent Paths
- **Colored lines**: Each agent's complete path
- **Circles**: Starting positions
- **Squares**: Final positions
- **Grid**: Environment layout with obstacles

### Right Panel: Performance Metrics
- **Mission Summary**: Agents, duration, execution time
- **Communication**: Message statistics and drop rates
- **Planning**: Replanning events and efficiency
- **Safety**: Collision detection results
- **Overall Score**: Performance rating (0-100)

## 🔧 System Requirements

### Minimum Requirements
- **Docker**: 20.10+
- **Memory**: 1GB available
- **Disk**: 500MB for image + output space
- **CPU**: 2+ cores recommended

### Host System
- **Linux**: Docker + Docker Compose
- **macOS**: Docker Desktop
- **Windows**: Docker Desktop or WSL2

## 🚨 Troubleshooting

### Build Issues
```bash
# Clean build
docker-compose down
docker system prune -f
docker-compose build --no-cache
```

### Permission Issues (Linux)
```bash
# Fix output directory permissions
sudo chown -R $USER:$USER output/
```

### Out of Memory
```bash
# Reduce agent count or use smaller maps
docker-compose run swarmgrid-custom run-simulation.sh --map maps/small.txt --agents 4
```

### No Output Files
```bash
# Check if output directory exists and is writable
mkdir -p output
ls -la output/

# Run with verbose logging
docker-compose run swarmgrid-custom run-simulation.sh --map maps/demo.txt --verbose
```

## 🎨 Customization

### Adding Custom Maps
1. Create new map file in `maps/` directory
2. Use format: `.` for open space, `#` for obstacles
3. Place start/goal positions as needed

### Modifying Visualization
Edit `visualize.py` to customize:
- Colors and styling
- Additional metrics
- Animation parameters
- Output formats

### Performance Tuning
```bash
# For production deployments, consider:
docker run --cpus="2.0" --memory="1g" --rm -v $(pwd)/output:/tmp swarmgrid
```

## 📚 Research Applications

This simulation is ideal for:
- **Swarm Robotics Research**: Multi-agent coordination algorithms
- **Aerospace Applications**: UAV fleet management
- **Algorithm Development**: Pathfinding and communication protocols
- **Educational Demonstrations**: Distributed systems concepts
- **Performance Benchmarking**: Scalability and efficiency testing

## 🏆 Professional Showcase

Perfect for demonstrating:
- **Modern C++20**: Advanced language features
- **Software Architecture**: Ports & Adapters pattern
- **Test-Driven Development**: Comprehensive test coverage
- **DevOps Practices**: Containerization and reproducible builds
- **Algorithm Implementation**: A* with spatial-temporal reservations
- **System Design**: Scalable, maintainable codebase

---

**Ready to deploy realistic swarm simulations anywhere!** 🚁✨