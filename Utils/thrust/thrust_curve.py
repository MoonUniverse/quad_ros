# import numpy as np
# import matplotlib as mpl
# import matplotlib.pyplot as plt
# from numpy import polyfit, poly1d
# fig, ax = plt.subplots()
# ax.plot(x, y, 'rx')
# ax.set_xlabel('x')
# ax.set_ylabel('y')

# coeff = polyfit(x, y, 1)

# print(coeff)

# p = plt.plot(x, coeff[0] * x + coeff[1], 'k-')

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from numpy import polyfit, poly1d

x = [0.5,  0.55,  0.6,   0.65,  0.7,   0.75,  0.8,   0.85,  0.9,   0.95,  1.0]
y = [2.77, 3.183, 3.629, 4.030, 4.547, 4.847, 5.194, 5.570, 5.962, 6.311, 6.536]

p = plt.plot(y, x, 'rx')

coeff = polyfit(y, x, 1)
print(coeff)


# p = plt.plot(x, coeff[0] * x + coeff[1], 'k-')
# p = plt.plot(x, y, 'b--')


plt.show()
