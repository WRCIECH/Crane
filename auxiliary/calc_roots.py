#!/usr/bin/python

import numpy as np

om = 1.0
# g = [-1, 1, 1, -10, 1, -1, 10, 1, -300]
g = [-1, 1, 1, -10, 1, -1, 10, 1, -300]
print -g[8]-g[5]
print g[5]*g[8] - g[6]*g[7] - g[3] + om*om
print g[4]*g[7] + g[3]*g[8] - om*om*g[5] - om*om*g[8] + om*om*g[1]
print om*om*g[5]*g[8] + g[2]*g[7] - om*om*g[6]*g[7] - om*om*g[1]*g[8]

coeff = [1,
         -g[8]-g[5],
         g[5]*g[8] - g[6]*g[7] - g[3] + om*om,
         g[4]*g[7] + g[3]*g[8] - om*om*g[5] - om*om*g[8] + om*om*g[1],
         om*om*g[5]*g[8] + g[2]*g[7] - om*om*g[6]*g[7] - om*om*g[1]*g[8]]

print np.roots(coeff)


f1 = -1
f2 = -1000
c2 = [1, -f2, -f1]
print np.roots(c2)
