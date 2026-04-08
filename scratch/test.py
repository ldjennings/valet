import itertools
import math


headings = [
    # i * (math.pi / 4)
    # for i in range(round(2 * math.pi / (math.pi / 4)))
]

offsets = [
    (dx * .5, dy * .5)
    for dx, dy in itertools.product(range(-2, 3), repeat=2)
    if (dx, dy) != (0, 0)
]

sum = 0
for (dx, dy), h_to in itertools.product(offsets, headings):
    sum += 1

print(sum)