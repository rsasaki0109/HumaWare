# Containers

HumaWare uses ROS 2 Jazzy as the default development baseline.

Build the Jazzy image:

```bash
docker build -f containers/Dockerfile.jazzy -t humaware:jazzy .
```

Build the Humble compatibility image:

```bash
docker build -f containers/Dockerfile.humble -t humaware:humble .
```

Run a shell:

```bash
docker run --rm -it --net=host -v "$PWD":/workspaces/humaware humaware:jazzy
```
