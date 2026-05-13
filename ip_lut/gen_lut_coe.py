import math
from pathlib import Path

OUT_DIR = Path(__file__).resolve().parent
N = 4096
SCALE = 32767

def q15(v: float) -> int:
    x = int(round(v * SCALE))
    if x > 32767:
        x = 32767
    if x < -32768:
        x = -32768
    return x

def write_coe(filename: str, values):
    path = OUT_DIR / filename
    with open(path, "w", encoding="ascii") as f:
        f.write("memory_initialization_radix=10;\n")
        f.write("memory_initialization_vector=\n")
        for i, value in enumerate(values):
            end = ";\n" if i == len(values) - 1 else ",\n"
            f.write(f"{value}{end}")
    print(f"wrote {path}")

sin_values = [q15(math.sin(2.0 * math.pi * i / N)) for i in range(N)]
cos_values = [q15(math.cos(2.0 * math.pi * i / N)) for i in range(N)]

write_coe("sin_lut_4096_q15.coe", sin_values)
write_coe("cos_lut_4096_q15.coe", cos_values)