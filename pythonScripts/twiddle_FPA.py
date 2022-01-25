import cmath
import math
import sys

N = int(sys.argv[1])
outArray = [0]
print(N/2)

for i in range(int(N/2)):
    angle = -(2*math.pi / N) * i
    w = cmath.rect(1, angle)
    print("{" + ".real = {:.0f}\t , .imag ={:.0f}".format((2**15)*w.real, (2**15)*w.imag) + "},")
