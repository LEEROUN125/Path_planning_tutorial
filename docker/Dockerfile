# Base image with GUI support
FROM ubuntu:24.04

# Set environment variables for UTF-8
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV DISPLAY=${DISPLAY}

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libsfml-dev \
    python3 \
    python3-pip \
    x11-apps \
    libxext6 \
    libxrandr2 \
    libxfixes3 \
    libxinerama1 \
    libgl1 \
    libgl1-mesa-dri \
    libfreetype6 \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /project

# Copy project files
COPY . /project/