# SWARMz4: Docker

## Prerequisites

Docker installed [Docker Engine](https://docs.docker.com/engine/install/ubuntu/)

For NVIDIA GPU, install the NVIDIA Container Toolkit:
[NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

## Use Image from Docker Hub

### Swarmz4 Image

Get already built image from Docker Hub:

```bash
docker pull crybakowski/swarmz4sim
```

Or build fresh image (from SWARMz4 parent directory):

```bash
docker build --no-cache -t crybakowski/swarmz4sim:latest -f Docker/Dockerfile .
```

### Single container launch

2. Launch Container

```bash
docker exec -it crybakowski/swarmz4sim /bin/bash
```

### Docker Compose

Launch container throught Docker Compose with NoVNC (Headless VM)
Start containers (CPU) :

```bash
docker-compose up -d    
```


Start containers (GPU) :

```bash
docker-compose -f docker-compose-nvidia.yml up -d    
```

Connect to running container :

```bash
docker exec -it swarmz4sim /bin/bash
```

Shutdown containers :

```bash
docker-compose down -d    
```


