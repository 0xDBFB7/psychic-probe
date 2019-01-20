make clean || exit 1
make all -j16 || exit 1
make upload || exit 1
