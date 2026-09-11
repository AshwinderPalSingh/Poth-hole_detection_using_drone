# Pothole Detection Models

The perception stack runs **without any downloaded model**. By default it uses
a built-in classical computer-vision detector, which on this simulated world
achieves precision 1.00 and recall 1.00 against the ground truth (see
`scripts/score_map.py`). Drop a trained model in here only if you want to run
on real imagery, where the classical detector's assumptions break down.

## Using a trained YOLOv8 model

Place a `.pt` file in this directory and launch with:

```bash
ros2 launch phoenix_drone_bringup full_simulation.launch.py \
    use_yolo:=true \
    model_path:=$(ros2 pkg prefix phoenix_drone_perception)/share/phoenix_drone_perception/models/best.pt
```

Any path works — the file does not have to live here.

## Where to get one

* **Roboflow Universe** hosts several public pothole datasets and
  ready-to-use YOLOv8 weights: <https://universe.roboflow.com/>
  (search "pothole detection").
* **Train your own** from a labelled dataset:

  ```bash
  pip install ultralytics
  yolo detect train data=pothole.yaml model=yolov8n.pt epochs=100 imgsz=640
  # weights land in runs/detect/train/weights/best.pt
  ```

The node reads class names from the model itself, so any single-class
"pothole" model works unchanged. With a multi-class model, every detected
class is published; filter downstream if that is not what you want.

## Detector parameters

Set in `config/detector_params.yaml`, or overridden in
`launch/perception.launch.py`:

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `use_yolo` | `false` | Use YOLOv8 instead of the classical detector |
| `model_path` | `''` | Path to a `.pt` model (empty → YOLOv8n placeholder) |
| `confidence_threshold` | `0.35` | Minimum detection confidence |
| `max_inference_fps` | `6.0` | Inference rate cap |
| `min_area_px` | `150` | Classical: smallest blob to consider |
| `min_circularity` | `0.45` | Classical: how round a pothole must be |
| `darkness_ratio` | `0.35` | Classical: how much darker than the road |

> Note: weights are excluded from version control by `.gitignore`.
