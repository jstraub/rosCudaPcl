#/usr/bin/env bash

git clone https://github.com/jstraub/cudaPcl
cd cudaPcl; make checkout; make configure; make -j; make install

echo "-------------------------"
echo "now ready to run rosmake"
echo "-------------------------"
