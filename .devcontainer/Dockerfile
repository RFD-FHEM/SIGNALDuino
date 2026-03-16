FROM mcr.microsoft.com/vscode/devcontainers/python:3.10
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        cmake \
        python3-pip \
    && pip3 install --no-cache-dir platformio \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
