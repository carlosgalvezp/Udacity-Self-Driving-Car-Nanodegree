#!/bin/bash
set -e

# Download simulator
wget https://d17h27t6h515a5.cloudfront.net/topher/2016/November/5831f0f7_simulator-linux/simulator-linux.zip
unzip simulator-linux.zip -d simulator

# Download training data
wget https://d17h27t6h515a5.cloudfront.net/topher/2016/December/584f6edd_data/data.zip
unzip data.zip

# Clean up
rm -r simulator-linux.zip data.zip __MACOSX



