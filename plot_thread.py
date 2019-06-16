import matplotlib.pyplot as plt
from matplotlib import colors
import csv
import numpy as np

num = []
executive = []

N_point = 0
n_bins = 100
no = 0

################# CALCULATE RESULT #################
with open('kdl_test_time','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        N_point = N_point + 1
        num.append(N_point)
        executive.append(int(row[0]))

executive_max = max(executive)
executive_min = min(executive)
executive_avr = np.mean(executive)
executive_std = np.std(executive)

################# SAVE RESULT #################
result = open('result.txt','w')
result.write('Result: Max - Min - Avr - Standard Deviation\n')
result.write('Executive: ' + str(executive_max) + ' - '  + str(executive_min) + ' - ' +str(executive_avr) + ' - ' +str(executive_std) + '\n')
result.close()

################# PLOT HISTOGRAM #################
N, bins, patches = plt.hist(executive, bins=n_bins)
fracs = N.astype(float) / N.max()
norm = colors.Normalize(fracs.min(), fracs.max())
for thisfrac, thispatch in zip(fracs, patches):
    color = plt.cm.viridis(norm(thisfrac))
    thispatch.set_facecolor(color)
plt.xlabel('executive [ns] - Max: '+ str(executive_max) + ' - Avr: ' + str(executive_avr) + ' - Standard Deviation: ' +str(executive_std))
plt.ylabel('number of sample')
plt.title('Executive Histogram - Only Orocos')
plt.legend()
plt.savefig('his_executive.jpg', transparent=False, bbox_inches='tight', pad_inches=0)
plt.show()

################# PLOT TIME #################
plt.plot(num,executive,label='Executive - Only Orocos')
plt.xlabel('sample - Max: '+ str(executive_max) + ' - Avr: ' + str(executive_avr) + ' - Standard Deviation: ' +str(executive_std))
plt.ylabel('executive [ns]')
plt.title('Executive')
plt.legend()
plt.savefig('plot_executive.jpg', transparent=False, bbox_inches='tight', pad_inches=0)
plt.show()
