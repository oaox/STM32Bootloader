#!/bin/sh
# To be run from the Debug or Release folder
fn='../Core/Inc/ver.h'

echo -n 'char gitrevision[]={"' > $fn
r=$(git rev-parse --short HEAD)
echo -n $r >> $fn
echo '"};' >> $fn

echo -n 'char date[]={"' >> $fn
d=$(date +%F)
echo -n $d >> $fn
echo '"};' >> $fn


