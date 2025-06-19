import hashlib
import sys

h = hashlib.new('sha256')
h.update((sys.argv[1] if len(sys.argv) > 1 else "Test123").encode("utf-8"))
res = h.digest()[:6]
for i in range(6):
    sys.stdout.write("%02x " % res[i])
sys.stdout.write("\n")
