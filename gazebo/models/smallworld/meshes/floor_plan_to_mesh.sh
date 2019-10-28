#!/bin/bash

# We need to upsample (NN interpolation) for potrace to make proper corners
convert -resize 200% -filter point floor_plan.png tmp.bmp 

# Run potrace without smoothing
potrace tmp.bmp -t 0 -a 0 -n -O 0 -u 1 --eps

# Convert the eps to dxf for OpenScad. Needs latest pstoedit 3.71 (incl libpstoedit).
width=$(identify -format "%w" "floor_plan.png")
halfwidth=$(echo "scale=1; $width/2.0" | bc)
height=$(identify -format "%h" "floor_plan.png")
halfheight=$(echo "scale=1; $height/2.0" | bc)
pstoedit -dt -xshift -$width -yshift -$height -f "dxf: -polyaslines -mm" tmp.eps -rdb tmp.dxf

# Extrude the geometry
openscad -o tmp.off floor_plan.scad

# Convert to Collada file
name=$(echo "$inputPng" | cut -d'.' -f1)
ctmconv tmp.off floor_plan.stl
