import subprocess
import sys
import re

NS3_CMD = ["./ns3", "run", "scratch/MultiplicativeDecreaseScratch"]

SS_RE = re.compile(r"\s*Queue delay\s*:\s*(\d+)\s*Target delay\s*:\s*(\d+)")
EXIT_RE = re.compile(r"Exiting initial slow start due to exceeding 3/4 of target delay...")

proc = subprocess.run(
    NS3_CMD,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True
)

log = proc.stdout.splitlines()

last_ss_before_exit = []
current_ss = None

for line in log:
    ss_match = SS_RE.search(line)
    exit_match = EXIT_RE.search(line)

    if ss_match:
        qd = int(ss_match.group(1))
        td = int(ss_match.group(2))
        current_ss = (qd, td)

    if exit_match:
        if current_ss:
            last_ss_before_exit.append(current_ss)
        current_ss = None

if not last_ss_before_exit:
    print("FAIL: No early slow start exit detected")
    sys.exit(1)

all_pass = True
for idx, (qd, td) in enumerate(last_ss_before_exit):
    threshold = 0.75 * td
    if qd > threshold:
        print(f"FAIL: Row {idx} exited slow start too late "
              f"(queue_delay before exit={qd}, threshold={threshold})")
        all_pass = False
    else:
        print(f"INFO: Row {idx} passed early SS exit check "
              f"(queue_delay before exit={qd}, threshold={threshold})")

if all_pass:
    print("PASS: LEDBAT++ modified slow start conforms to RFC 4.3")
    sys.exit(0)
else:
    sys.exit(1)