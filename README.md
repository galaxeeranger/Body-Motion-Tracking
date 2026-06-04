# Mocap - Multi-Camera AI Motion Capture System

![Mocap Demo](./Mocap_image.png)

A real-time AI-powered motion capture system built using Python, MediaPipe, OpenCV, and multi-camera 3D triangulation. The system captures human motion from multiple cameras, reconstructs a 3D skeleton in real time, and provides a foundation for animation retargeting, game development, virtual production, and character rigging workflows.

## 🚀 Features

- Multi-camera real-time motion capture
- 3D human pose reconstruction
- MediaPipe-based body tracking
- Robust RANSAC triangulation
- Camera calibration using ChArUco boards
- Real-time OpenGL 3D visualization
- Temporal smoothing and jump rejection
- Multi-camera scalability (2–10+ cameras)
- Distortion-aware triangulation
- Bundle Adjustment for accurate camera alignment
- Ready for animation retargeting pipelines

---

## Demo

Repository:

👉 https://github.com/galaxeeranger/Mocap

### Real-Time 3D Pose Visualization

The system reconstructs a live 3D skeleton from multiple camera feeds.

![Mocap Visualization](./Mocap_image.png)

---

## System Architecture

```text
Multiple Cameras
        │
        ▼
Camera Calibration
(ChArUco + Bundle Adjustment)
        │
        ▼
MediaPipe Pose Detection
(33 Keypoints per Camera)
        │
        ▼
Multi-View Triangulation
(RANSAC + Reprojection Filtering)
        │
        ▼
Temporal Smoothing
(One Euro Filter)
        │
        ▼
3D Skeleton Reconstruction
        │
        ▼
Animation / Retargeting Pipeline
```

---

## Technologies Used

### Computer Vision

- OpenCV
- MediaPipe
- NumPy

### Motion Capture

- Multi-view geometry
- Triangulation
- Bundle Adjustment
- Camera calibration

### Visualization

- OpenGL
- Custom 3D Skeleton Renderer

### Backend

- Python

---

## Project Structure

```text
Mocap/
│
├── calib/
│   ├── pipeline.py
│   ├── capture.py
│   └── graph.py
│
├── triangulation/
│   └── triangulator.py
│
├── utils/
│   ├── smoothing.py
│   └── visualizer_3d.py
│
├── viz/
│   └── app.py
│
├── output/
│   └── multi_cam_calibration.npz
│
├── config.py
├── run_pose_3d.py
└── main.py
```

---

## Installation

### Clone Repository

```bash
git clone https://github.com/galaxeeranger/Mocap.git

cd Mocap
```

### Create Virtual Environment

```bash
python -m venv venv
```

### Activate Environment

Windows

```bash
venv\Scripts\activate
```

Linux/Mac

```bash
source venv/bin/activate
```

### Install Dependencies

```bash
pip install -r requirements.txt
```

---

## Camera Calibration

Generate calibration data using a ChArUco board.

```bash
python main.py
```

This produces:

```text
output/multi_cam_calibration.npz
```

which contains:

- Camera Intrinsics
- Camera Extrinsics
- Distortion Coefficients
- Bundle Adjusted Camera Poses

---

## Running Motion Capture

Start the real-time mocap system:

```bash
python run_pose_3d.py
```

The application will:

1. Load camera calibration
2. Start all connected cameras
3. Run MediaPipe pose estimation
4. Triangulate 3D joints
5. Apply smoothing
6. Render the live 3D skeleton

---

## Multi-Camera Support

Current architecture supports:

- 2 Cameras
- 3 Cameras
- 5 Cameras
- 8 Cameras
- 10+ Cameras

Configure in:

```python
NUM_CAMERAS = 5
```

inside:

```python
config.py
```

---

## Advanced Features

### Robust RANSAC Triangulation

Rejects:

- Occluded joints
- Incorrect detections
- Left/right swaps

before final 3D reconstruction.

### Temporal Smoothing

Reduces:

- Jitter
- Frame-to-frame noise
- Sudden skeleton jumps

### Bundle Adjustment

Global optimization refines all camera poses simultaneously, improving overall reconstruction accuracy.

---

## Applications

### Animation

- Character animation
- Motion retargeting
- Pre-visualization

### Game Development

- NPC animation
- Indie mocap pipeline
- Unreal Engine integration

### Virtual Production

- Live character driving
- Real-time performance capture

### Research

- Human motion analysis
- Sports biomechanics
- AI pose estimation research

---

## Future Roadmap

- Face capture integration
- Hand tracking
- Full-body IK solver
- Unreal Engine Live Link support
- Blender integration
- FBX/BVH export
- Real-time avatar driving
- Multi-person tracking
- Hardware synchronized camera support

---

## Performance

Current testing:

| Cameras | FPS    |
| ------- | ------ |
| 2       | 30+    |
| 3       | 25–30 |
| 5       | 20–25 |
| 8       | 15–20 |

Performance depends on:

- CPU
- GPU
- Camera resolution
- USB bandwidth

---

## Author

**Shivmurat Gupta**

AI Engineer | Computer Vision Developer | Motion Capture Enthusiast

Portfolio: https://shivmuratgupta.com

GitHub: https://github.com/galaxeeranger

LinkedIn: https://linkedin.com/in/shivmuratg

---

## License

MIT License

---

⭐ If you find this project useful, consider starring the repository.
