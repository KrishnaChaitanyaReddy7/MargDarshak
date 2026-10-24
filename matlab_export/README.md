# MATLAB Perception Export (`/matlab_export`)

## Overview
Connects our trained DeepLabv3+ semantic segmentation networks (trained on the Indian Driving Dataset `idd-lite` with ResNet-18 backbone) to the browser-based simulation engine.

## Contents
- **`export_for_sim.m`**: Batch processing script that evaluates all 82 1080p video frames in `Video_Project_1_frames` through the perception pipeline.
  - Extracts drivable road boundary polygons using `bwboundaries` and simplifies them to $\le 60$ points with `reducepoly` / decimation.
  - Extracts dynamic and static Indian road obstacles (cars, trucks, auto-rickshaws, two-wheelers, pedestrians, animals, debris).
  - Emits one JSON file per frame in `/public/replay/frame_XXX.json` alongside `index.json`.

## Usage
Run in MATLAB:
```matlab
% Process all 82 frames
export_for_sim();

% Process specific frame (e.g. Frame 15 for judge testing)
export_for_sim(15);

% Process a range of frames
export_for_sim(1:20);
```

## JSON Output Schema (`/public/replay/frame_XXX.json`)
```json
{
  "frame": 15,
  "timestamp": 0.933,
  "image": "frames/frame_015.jpg",
  "road": [ [x1, y1], [x2, y2], ... ],
  "obstacles": [
    {
      "cls": "car",
      "bbox": [421, 391, 553, 524]
    }
  ]
}
```
All bounding boxes are formatted as `[x1, y1, x2, y2]` in 1920x1080 pixel space.
Road polygon coordinates are formatted as `[x, y]` in pixel space.
