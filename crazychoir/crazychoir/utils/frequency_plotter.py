import numpy as np
from matplotlib import pyplot as plt

crazyradio_limits = [2400,2525]

# Scan occupied frequencies running 'sudo iwlist scan | grep Frequency | sort | uniq -c | sort -n'

# Occupied frequencies example
not_avaliable_freq = [
    [2,2412],
    [2,5320], 
    [2,5500],
    [2,5660], 
    [3,2462],
]

# Filter 2.4GHz signals
filtered_2400Mhz_not_avaliable_freq = [freq for freq in not_avaliable_freq if freq[1] < 3000]

# Plot settings
linewidths = [5/signal_quality[0] for signal_quality in filtered_2400Mhz_not_avaliable_freq]
fig,ax = plt.subplots(1)
colors = []

# Plot carrier frequencies and its harmonics
for freq in filtered_2400Mhz_not_avaliable_freq:
    line,  = ax.plot([freq[1],freq[1]],[0,4/freq[0]])
    for i in range(-12,13,1):
        ax.plot([freq[1]-i,freq[1]-i],[0, (12-(abs(i)))**2/freq[0] ], color = line.get_color())

x_tiks = [freq[1] for freq in filtered_2400Mhz_not_avaliable_freq]
for limit in crazyradio_limits:
    x_tiks.append(limit)

print(x_tiks)
plt.xticks(x_tiks)
ax.set_xlim(crazyradio_limits)
plt.show()

# Select your radio channel choosing free frequency slots
