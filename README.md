# Finite Model Infinite Image (FMII)

![demo](/demo.png)

This repository provides a simple C++ implementation of the FMII algorithm. It provides a close-form solution for estimating the rigid-body transformation between 2 sets of line segments, referred as model and image. FMII is a variant of the matching algorithms for the case when the endpoints of the image line segments cannot be reasonably estimated, and hence are considered as infinitely long. 

FMII can be used as a subroutine in line based registration algorithms (e.g. ICL). FMII works well when the correspondences between image and model are perfect. However, when the correspondences is imperfect, it may have trouble converging. 

Note that the implementation is for 2D line segments only. 3D support can be easily added. 

## Running the code

The header FMII.h only depends on Eigen3. main.cpp dependents on matplotlibcpp (which is header only) to plot the lines. 

```
make && ./main
```

## References

B. Kamgar-Parsi and B. Kamgar-Parsi, "Algorithms for matching 3D line sets," in IEEE Transactions on Pattern Analysis and Machine Intelligence, vol. 26, no. 5, pp. 582-593, May 2004, doi: 10.1109/TPAMI.2004.1273930.