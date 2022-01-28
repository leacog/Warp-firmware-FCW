import sys

N = 128
for i in range(N):
    print(int('{:06b}'.format(i)[::-1], 2), end=',')
