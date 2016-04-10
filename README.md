ROS nodes for cudaPcl depth image and surface normal image smoothing.

### Install

Clone the cudaPcl repository into this folder
```
git clone https://github.com/jstraub/cudaPcl
```
and compile it
```
cd cudaPcl; make checkout; make configure; make -j; make install
```
now compile the ros code
```
rosmake
```
Done!

### Running it

Have a look at the launch files in the ./launch subfolder.

### Some Notes

Plotting timing stats:
```
rqt_plot /rt_mf/stats/data[0]  /rt_mf/stats/data[1]  /rt_mf/stats/data[2]  /rt_mf/stats/data[3] 
```

Plotting D_KL and residual:
```
rqt_plot /rt_mf/stats/data[4]  /rt_mf/stats/data[5]  
```

