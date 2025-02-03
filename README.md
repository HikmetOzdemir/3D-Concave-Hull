# 3D Concave Hull

![3D Concave Hull Visualization](demo.gif)

## Overview

This repository contains a simple Python demonstration that I created for fun. It dynamically generates random 3D points and computes their concave hull (using an alpha shape algorithm) in real time. The visualization features a slowly rotating, transparent 3D plot.

## What It Does

- **Dynamic Point Generation:** Generates up to 128 random 3D points.
- **Real-Time Concave Hull Calculation:** Uses an alpha shape algorithm to compute and update the concave hull as points are added.
- **Rotating 3D Visualization:** Displays the point cloud and hull in a 3D plot that rotates slowly to give a complete perspective.

## Getting Started

### Requirements

- Python 3
- [NumPy](https://numpy.org/)
- [Matplotlib](https://matplotlib.org/)
- [SciPy](https://www.scipy.org/)

### Installation

Clone the repository and install the required dependencies:

```bash
git clone https://github.com/HikmetOzdemir/3D-Concave-Hull.git
cd 3D-Concave-Hull
pip install numpy matplotlib scipy
python main.py
