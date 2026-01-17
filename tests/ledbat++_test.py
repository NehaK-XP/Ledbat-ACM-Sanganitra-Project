import subprocess
import re
import sys

NS3_CMD = ["./ns3", "run", "scratch/MultiplicativeDecreaseScratch"]

W1_RE = re.compile(r"W:\s*([0-9.]+)")
R1_RE = re.compile(r"delay_ratio:\s*([0-9.]+)")

W2_RE = re.compile(r"W=([0-9.]+)")
R2_RE = re.compile(r"ratio=([0-9.]+)")

proc = subprocess.run(
    NS3_CMD,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True
)

with open("ledbat_test.log", "w") as f:
    f.write(proc.stdout)

samples = []

pending_w = None
pending_r = None

for line in proc.stdout.splitlines():
    if "TcpLedbatPlusPlus" not in line and "base_delay" not in line and "W=" not in line:
        continue

    m1w = W1_RE.search(line)
    m1r = R1_RE.search(line)

    m2w = W2_RE.search(line)
    m2r = R2_RE.search(line)

    if m1w:
        pending_w = float(m1w.group(1))
    if m1r:
        pending_r = float(m1r.group(1))

    if m2w:
        pending_w = float(m2w.group(1))
    if m2r:
        pending_r = float(m2r.group(1))

    if pending_w is not None and pending_r is not None:
        samples.append((pending_w, pending_r))
        pending_w = None
        pending_r = None

if len(samples) < 2:
    print("FAIL: insufficient LEDBAT samples")
    sys.exit(1)

violations = []

for (w0, r0), (w1, r1) in zip(samples, samples[1:]):
    if r0 > 1.0 and w1 < w0:
        if w1 < w0 / 2.0 - 1e-6:
            violations.append(f"decrease exceeded W/2: {w0} -> {w1}")

        if w1 < 2.0:
            violations.append(f"cwnd dropped below 2: {w1}")

if violations:
    print("FAIL: RFC 4.2 violation detected")
    for v in violations:
        print(v)
    sys.exit(1)

print("PASS: LEDBAT++ multiplicative decrease conforms to RFC 4.2")
sys.exit(0)

