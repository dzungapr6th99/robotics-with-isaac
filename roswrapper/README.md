# RosWrapper

The simple package to execute order in vda5050. when C#/ java receive and validate order of vda5050, you can execute order by calling wrapper in this package.

# Build

Build vda5050_msgs package first, it will build to file library and file .h. then config file .vs in vs code, to include file .h, you can continue implement for this package

# Use this Wrapper in C#

First, you need to LD_PATH to folder build (the file include file .so). then you can use wrapper from C#. Remember to spin node from C#, you should provide a thread to do this
