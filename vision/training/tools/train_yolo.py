"""Train the first steel-ball YOLO model from a reviewed scene split."""

import argparse
from pathlib import Path

from ultralytics import YOLO


def _arguments():
    project_dir = Path(__file__).resolve().parents[1]
    workspace_dir = project_dir.parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--data",
        type=Path,
        required=True,
        help="Combined dataset data.yaml",
    )
    parser.add_argument(
        "--weights",
        type=Path,
        default=(
            workspace_dir
            / "contest_library"
            / "01_open_source_projects"
            / "k230_steel_ball_detection_xhz2127"
            / "best.pt"
        ),
        help="Initial YOLO weights",
    )
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=320)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--device", default="0")
    parser.add_argument(
        "--seed",
        type=int,
        default=2026,
        help="Random seed used for a reproducible comparison run.",
    )
    parser.add_argument(
        "--no-amp",
        action="store_true",
        help="Disable CUDA automatic mixed precision for compatibility.",
    )
    parser.add_argument(
        "--non-deterministic",
        action="store_true",
        help="Allow non-deterministic CUDA kernels when deterministic cuDNN fails.",
    )
    parser.add_argument(
        "--rect",
        action="store_true",
        help="Use rectangular batches for wide pipe images.",
    )
    parser.add_argument(
        "--no-mosaic",
        action="store_true",
        help="Disable mosaic augmentation to reduce host memory usage.",
    )
    parser.add_argument(
        "--no-plots",
        action="store_true",
        help="Disable training plots to reduce host memory usage.",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=0,
        help="Use 0 on Windows to avoid one CUDA import per worker process",
    )
    parser.add_argument(
        "--project",
        type=Path,
        default=project_dir / "training_runs",
    )
    parser.add_argument("--name", default="steel_ball_320_v1")
    parser.add_argument(
        "--resume",
        type=Path,
        help="Resume an interrupted Ultralytics run from its last.pt.",
    )
    return parser.parse_args()


def main():
    args = _arguments()
    if args.resume is not None:
        if not args.resume.is_file():
            raise RuntimeError("Resume checkpoint not found: {}".format(
                args.resume
            ))
        model = YOLO(str(args.resume))
        model.train(resume=True)
        return

    if not args.data.is_file():
        raise RuntimeError("Dataset YAML not found: {}".format(args.data))
    if not args.weights.is_file():
        raise RuntimeError("Initial weights not found: {}".format(args.weights))
    if args.epochs <= 0 or args.imgsz <= 0 or args.batch <= 0:
        raise ValueError("epochs, imgsz and batch must be positive")

    model = YOLO(str(args.weights))
    model.train(
        data=str(args.data.resolve()),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        project=str(args.project.resolve()),
        name=args.name,
        exist_ok=False,
        pretrained=True,
        patience=30,
        cache=False,
        plots=not args.no_plots,
        seed=args.seed,
        deterministic=not args.non_deterministic,
        amp=not args.no_amp,
        rect=args.rect,
        mosaic=0.0 if args.no_mosaic else 1.0,
        close_mosaic=10,
    )


if __name__ == "__main__":
    main()
