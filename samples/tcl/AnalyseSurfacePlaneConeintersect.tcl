# Test for IntPatch_GLine.

cone s1 0 0 0 0 0 1 30 3
plane s2 0 0 0 0 0 1 

mkface f1 s1 0 2*pi -5 5
mkface f2 s2 -8 8 -8 8

bop f1 f2
bopsection r

vdisplay f1 f2 r