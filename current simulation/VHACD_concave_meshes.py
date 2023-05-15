import pybullet as p
import os

in_file = os.path.join("object models","wrench","wrench.obj")
out_file = os.path.join("object models","wrench","wrench_VHACD.obj")
log_file = os.path.join("object models","wrench","VHACD_log_file.txt")

print(in_file)
print(out_file)
print(log_file)


p.vhacd(in_file,out_file,log_file)