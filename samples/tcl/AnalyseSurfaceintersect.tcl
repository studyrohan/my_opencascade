# Test for IntPatch_ALine.

# Geometry surfaces.
cylinder s1 0 0 0 1 1 1 2
cone s2 0 0 0 0 0 1 1 0 0 30 3

# Topology faces
mkface f1 s1 0 2*pi -8 8
mkface f2 s2 0 2*pi -5 5

# Intersection.
bop f1 f2
bopsection r

# Display result.
vdisplay f1 f2 r