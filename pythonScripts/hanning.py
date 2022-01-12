import numpy as np

win = np.hanning(32)

for i in range(32):
    print(int(win[i]*8192), end=',')
