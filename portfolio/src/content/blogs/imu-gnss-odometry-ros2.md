---
title: "My Journey to Building Stable Global Odometry Using Only IMU and GNSS in ROS2"
date: 2026-05-25
description: "Can two sensors — an IMU and a GNSS receiver — produce usable global odometry? Here's my first attempt at sensor fusion using robot_localization in ROS2."
tags: ["ros2", "robotics", "sensor-fusion", "gnss", "imu", "localization"]
---

## 1. Motivation

Many outdoor robots start with only two sensors: a GNSS receiver and an IMU. Before adding wheel encoders, cameras, or LiDAR, I wanted to understand whether these two sensors alone could produce usable global odometry.

This is a common starting point in field robotics. You have a relatively cheap setup, and you want to know: how far can you get with just position from GNSS and motion from an IMU? The answer turns out to be more nuanced than I expected — and the journey taught me more about ROS2 localization than any tutorial I had read.

This post documents what I tried, what broke, and what I learned along the way. It is not a polished definitive guide. Some questions are still open.

---

## 2. The Question

The entire experiment revolves around one central question:

> **Can I obtain stable and meaningful odometry using only an IMU and GNSS?**

Everything else — the architecture choices, the debugging detours, the config tuning — is in service of answering this question honestly.

---

## 3. Understanding the Sensors

Before fusing anything, it helps to understand what each sensor actually gives you and where it falls short.

### GNSS

A GNSS receiver provides:

- **Latitude, Longitude, Altitude** — absolute position on Earth
- **Velocity** — some modules estimate velocity from Doppler shift

This sounds great on paper. In practice, raw GNSS has real problems:

- **Noise** — position jumps on the order of 1–5 meters are common with low-cost receivers
- **Multipath** — signals bouncing off buildings or terrain corrupt the measurement
- **Low update rate** — most modules output at 1–10 Hz, too slow for smooth motion estimation
- **No heading** — a single GNSS antenna cannot tell you which way the robot is pointing

If you plot raw GNSS position for a stationary robot, you will see it wander. That wandering is not the robot moving. It is measurement noise.

### IMU

An IMU provides:

- **Angular velocity** (gyroscope) — how fast the robot is rotating
- **Linear acceleration** (accelerometer) — the net acceleration including gravity

Limitations:

- **Bias** — the sensor reads a non-zero value even when stationary
- **Drift** — integrating angular velocity accumulates error over time
- **Gravity** — you must remove the gravitational component from acceleration before integrating

The most important thing to understand about IMUs for localization: **you cannot reliably integrate acceleration into position**. The bias causes the estimated velocity to grow over time, and integrated twice into position, the error becomes enormous within seconds.

Here is what happens when you naively double-integrate raw accelerometer data for a robot that is completely stationary:

```
t=0s    position error: ~0 m
t=5s    position error: ~0.5 m
t=30s   position error: several meters
t=60s   position error: tens of meters
```

The robot hasn't moved. The error is entirely due to bias drift. This is why IMU alone cannot give you position — it can only give you short-term motion changes.

---

## 4. Why Sensor Fusion Is Needed

Given the weaknesses of each sensor individually, the idea behind fusion is straightforward:

```
GNSS  →  absolute position (noisy, slow)
 +
IMU   →  relative motion (fast, drifts over time)
 =
EKF   →  better estimate than either alone
```

GNSS anchors the estimate to a real-world coordinate. It prevents the IMU drift from accumulating unboundedly. The IMU, in turn, fills in the gaps between GNSS updates and smooths out the noise in individual GNSS fixes.

The algorithm that does this combination is the **Extended Kalman Filter (EKF)**. The EKF maintains a state estimate (position, velocity, orientation) and a covariance matrix that represents how confident it is in each component. Every sensor measurement updates the state according to how trustworthy that measurement is relative to the current uncertainty.

The key intuition: when the GNSS noise is high, the EKF trusts the IMU more. When IMU drift is accumulating, the GNSS measurement pulls the estimate back toward ground truth. Neither sensor dominates; they are weighted by their respective uncertainties.

I won't go deep into the matrix math here — there are better resources for that. What matters practically is understanding that you control the filter's behavior through **covariance values**, which tell the filter how much to trust each input. Getting these wrong is the most common source of bad results, and I spent most of my debugging time on exactly this.

---

## 5. ROS2 Architecture

Here is the system I ended up building:

```
[IMU]                [GNSS]
  |                    |
  | /imu/data          | /fix (NavSatFix)
  |                    |
  v                    v
[robot_localization]  [navsat_transform_node]
  ekf_node               |
  ^                      | /odometry/gps
  |______________________|
            |
            v
    /odometry/filtered
            |
            v
      [TF: odom → base_link]
```

Two nodes from the `robot_localization` package do the work:

**`ekf_node`**
The EKF filter itself. It takes IMU data and the GPS-derived odometry, fuses them, and publishes `/odometry/filtered` — a smoothed pose estimate in the `odom` frame. It also broadcasts the `odom → base_link` TF transform.

**`navsat_transform_node`**
A helper that converts raw GNSS fixes (latitude/longitude/altitude from `/fix`) into odometry messages in the robot's local frame. It handles the datum (the reference origin point), the projection from spherical to Cartesian coordinates, and the heading offset between GNSS and the robot's body frame. Its output feeds into `ekf_node`.

The reason this two-node split exists: the EKF works in a local Cartesian frame (meters), not in GPS coordinates (degrees). `navsat_transform_node` does that conversion so the EKF doesn't have to care about geodesy.

---

*This post covers sections 1–5. The next section will document the actual hardware setup, initial configuration, and what the first trajectory looked like — including what went wrong.*
