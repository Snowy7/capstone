#!/usr/bin/env python3
import os
import math
import cv2
from moms_apriltag import TagGenerator2

# ----- CONFIG -----
FAMILY = "tag36h11"
TAG_ID = 0

TAG_SIZE_M = 0.03       # desired physical size: 0.08 m = 80 mm
PRINT_DPI = 300         # target print DPI
MARGIN_MM = 5           # white margin around tag
OUT_FILE = "tag36h11_0_80mm_300dpi.png"
# -------------------

def main():
    # Compute tag size in pixels for chosen DPI
    tag_size_inch = TAG_SIZE_M / 0.0254  # meters -> inches
    tag_px = int(round(tag_size_inch * PRINT_DPI))

    # Margin in pixels
    margin_inch = MARGIN_MM / 25.4
    margin_px = int(round(margin_inch * PRINT_DPI))

    img_size = tag_px + 2 * margin_px

    print(f"Target: {TAG_SIZE_M*1000:.1f} mm at {PRINT_DPI} DPI "
          f"-> tag_px={tag_px}, total_image={img_size}x{img_size}")

    tg = TagGenerator2(FAMILY)
    tag = tg.generate(TAG_ID)  # numpy array, values 0/255 (or 0/1 scaled)

    # Ensure 8-bit grayscale
    if tag.dtype != "uint8":
        tag = (tag.astype("float32") * 255).clip(0, 255).astype("uint8")

    # Resize tag region to desired pixel size
    tag_resized = cv2.resize(tag,
                             (tag_px, tag_px),
                             interpolation=cv2.INTER_NEAREST)

    # White canvas
    canvas = 255 * \
        np.ones((img_size, img_size), dtype="uint8")

    # Paste centered
    y0 = margin_px
    x0 = margin_px
    canvas[y0:y0 + tag_px, x0:x0 + tag_px] = tag_resized

    os.makedirs(os.path.dirname(OUT_FILE) or ".", exist_ok=True)
    cv2.imwrite(OUT_FILE, canvas)
    print(f"Saved {OUT_FILE}")
    print("In your PDF/image viewer print dialog:")
    print("- Disable 'fit to page' / 'shrink to fit'.")
    print("- Set scale to 100%.")
    print(f"- Ensure printer is effectively at {PRINT_DPI} DPI (most are 300/600).")

if __name__ == "__main__":
    import numpy as np
    main()