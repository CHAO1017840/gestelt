#!/usr/bin/env python3
"""Generate a deterministic high-contrast texture for Gazebo VIO tests."""

from pathlib import Path
import random

from PIL import Image, ImageDraw


TEXTURE_SIZE = 1024
CELLS_PER_SIDE = 16
CELL_SIZE = TEXTURE_SIZE // CELLS_PER_SIDE


def generate_texture(output_path: Path) -> None:
    rng = random.Random(3576)
    image = Image.new("RGB", (TEXTURE_SIZE, TEXTURE_SIZE), "#eceff1")
    draw = ImageDraw.Draw(image)

    dark = "#253238"
    light = "#f1f3f4"
    accent_colors = ("#d84315", "#00838f", "#7b1fa2", "#2e7d32")

    for row in range(CELLS_PER_SIDE):
        for column in range(CELLS_PER_SIDE):
            x0 = column * CELL_SIZE
            y0 = row * CELL_SIZE
            x1 = x0 + CELL_SIZE
            y1 = y0 + CELL_SIZE
            color = dark if (row + column) % 2 == 0 else light
            draw.rectangle((x0, y0, x1, y1), fill=color)

    # Color cells break checkerboard periodicity and provide distinctive
    # landmarks at different image locations.
    for _ in range(24):
        column = rng.randrange(CELLS_PER_SIDE)
        row = rng.randrange(CELLS_PER_SIDE)
        x0 = column * CELL_SIZE + 7
        y0 = row * CELL_SIZE + 7
        x1 = (column + 1) * CELL_SIZE - 7
        y1 = (row + 1) * CELL_SIZE - 7
        draw.rectangle((x0, y0, x1, y1), fill=rng.choice(accent_colors))

    # Asymmetric multi-scale markers remain easy to see after texture
    # repetition and create corners at several spatial frequencies.
    draw.ellipse((96, 96, 320, 320), outline="#ffffff", width=18)
    draw.ellipse((132, 132, 284, 284), outline=dark, width=12)
    draw.line((160, 208, 256, 208), fill="#ffb300", width=20)

    draw.rectangle((680, 128, 928, 224), fill="#1565c0")
    draw.line((704, 176, 904, 176), fill="#ffffff", width=18)
    draw.line((804, 144, 804, 208), fill="#ffffff", width=18)

    draw.polygon(
        ((128, 704), (320, 800), (128, 896)),
        fill="#ef6c00",
        outline="#ffffff",
    )
    draw.ellipse((704, 704, 896, 896), fill="#fafafa")
    draw.line((704, 800, 896, 800), fill=dark, width=24)
    draw.line((800, 704, 800, 896), fill=dark, width=24)

    # Thin borders make cell corners robust to bilinear filtering at distance.
    for coordinate in range(0, TEXTURE_SIZE + 1, CELL_SIZE):
        draw.line((coordinate, 0, coordinate, TEXTURE_SIZE), fill="#546e7a", width=3)
        draw.line((0, coordinate, TEXTURE_SIZE, coordinate), fill="#546e7a", width=3)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path, format="PNG", optimize=True)


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    output_path = (
        repo_root
        / "simulation"
        / "models"
        / "ground_plane"
        / "materials"
        / "textures"
        / "vio_ground.png"
    )
    generate_texture(output_path)
    print(output_path)


if __name__ == "__main__":
    main()
