from pathlib import Path
import pdfplumber

for name in ("ref_2021_delivery_car.pdf", "ref_2023_tracking.pdf"):
    chunks = []
    with pdfplumber.open(name) as pdf:
        for i, page in enumerate(pdf.pages, 1):
            chunks.append(f"\n===== PAGE {i} =====\n")
            chunks.append(page.extract_text(x_tolerance=2, y_tolerance=3) or "")
    Path(name).with_suffix(".txt").write_text("\n".join(chunks), encoding="utf-8")
    print(name, "pages=", len(pdf.pages), "chars=", sum(map(len, chunks)))
