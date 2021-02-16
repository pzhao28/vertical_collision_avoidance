import math
import numpy as np
import matplotlib.pyplot as plt

sigma = 40 #TCAS traffic alert
t = np.arange(0, 200)
pdf = np.exp(-0.5*(t/sigma)**2) / (np.sqrt(2*np.pi)*sigma)
markers_on = [90, 100]
plt.plot(pdf, 'k-', markevery = markers_on, marker = 'o')
#plt.plot([0,90],[pdf[90],pdf[90]],'k--', [0, 100], [pdf[100], pdf[100]], 'k--')
for tt in range(90,110,10):
    label = "({},".format(tt) + "{0:.5f})".format(pdf[tt])
    plt.annotate(label, (tt, pdf[tt]), textcoords = "offset points", xytext=(0,5), ha='left')
plt.ylabel('Probability density function')
plt.xlabel('Time(s)')
plt.xlim(0, 200)
plt.show()