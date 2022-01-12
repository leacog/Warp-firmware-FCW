import cmath
import math
import sys

N = int(sys.argv[1])
outArray = [0]
print(N/2)

for i in range(int(N/2)):
    angle = -(2*math.pi / N) * i
    w = cmath.rect(1, angle)
    print("{:.0f}\t + {:.0f}\t *I,".format(1000*w.real, 1000*w.imag))
