FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

RUN apt-get update && apt-get install -y sudo git vim lsb-release software-properties-common 
ENV PATH="$PATH:/usr/local/cuda/bin"
