from matplotlib import pyplot as plt
from matplotlib import style
import numpy as np

style.use('ggplot')

y = np.loadtxt('result_preempt.txt', unpack = True)
x = np.arange(y.size)
print(np.amax(y))

plt.hist(y,200)

plt.title('Benchmark Orocos KDL')
plt.ylabel('Number of Sample')
plt.xlabel('Excution time (us), max = ' + str(np.amax(y)))

#plt.title('Benchmark Orocos KDL')
#plt.ylabel('Excution time (us), max = ' + str(np.amax(y)))
#plt.xlabel('Sample')

plt.show()
