import matplotlib.pyplot as plt
import numpy as np
import os

def plot_data(x_axis_data, y_axis_data):
    plt.plot(x_axis_data, y_axis_data, 'b')
    plt.show()

def plot_data_two_curves(x_axis_data, y_axis_data1, y_axis_data2):
    plt.plot(x_axis_data, y_axis_data1, 'b', x_axis_data, y_axis_data2, 'g')
    plt.show()

def plot_data_three_curves(x_axis_data, y_axis_data1, y_axis_data2, y_axis_data3):
    plt.plot(x_axis_data, y_axis_data1, 'b', x_axis_data, y_axis_data2, 'g', x_axis_data, y_axis_data3, 'r')
    plt.show()

def plot_3D_data(x_axis_data, y_axis_data, z_1, z_2, z_3, z_4):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot_wireframe(x_axis_data, y_axis_data, z_1, color='b')
    ax.plot_wireframe(x_axis_data, y_axis_data, z_2, color='g')
    ax.plot_wireframe(x_axis_data, y_axis_data, z_3, color='r')
    ax.plot_wireframe(x_axis_data, y_axis_data, z_4, color='k')
    plt.show()

def plot_3D_loss_func(X_axis_data, y_axis_data, z_1):
    plt.contourf(X_axis_data, y_axis_data, z_1, 100, cmap='autumn')
    plt.colorbar()
    plt.show()




def plot_variable_vs_time(variable, variable_name, dt, out_dir=None, show=False):
    title = "time vs "+ variable_name
    plt.title(title)
    plt.xlabel("time")
    plt.ylabel(variable_name)
    plt.plot([i*dt for i in range(len(variable))], variable, 'ob')
    if out_dir is not None:
        plt.savefig(os.path.join(out_dir, title))
    if show:
        plt.show()
    plt.close("all")




def plot_variables(var1, var1_name, var2, var2_name, title_preamble="", line_data=None, out_dir=None, show=False):
    title = title_preamble + var1_name + " vs "+ var2_name
    plt.title(title)
    plt.xlabel(var1_name)
    plt.ylabel(var2_name)
    for i in np.arange(len(var1)):
        plt.plot(var1[i], var2[i], marker='o', color=(1.-(i/len(var1)),0.,i/len(var1),1.))
    if line_data is not None:
        plt.plot(var1,line_data, 'r-')
    if out_dir is not None:
        plt.savefig(os.path.join(out_dir, title))
    if show:
        plt.show()
    plt.close("all")


def plot_variables_plain(var1, var1_name, var2, var2_name, title_preamble="", out_dir=None, show=False):
    title = title_preamble + var1_name + " vs "+ var2_name
    plt.title(title)
    plt.xlabel(var1_name)
    plt.ylabel(var2_name)
    plt.plot(var1,var2, 'b-')
    if out_dir is not None:
        plt.savefig(os.path.join(out_dir, title))
    if show:
        plt.show()
    plt.close("all")

#plot_data([1,2,3,4], [1,4,9,16], [1,2,6,24])

#no overloading? These definitions are overriding each other? I do not like that.

'''size = 49
import numpy as np
loss_sweep_data = open("loss_sweep_file.csv", "r")
data = loss_sweep_data.read()
loss_sweep_data.close()
lines = data.split("\n")
X = np.zeros((size,size))
Y = np.zeros((size,size))
Z = np.zeros((size,size))
i = j = 0
for line in lines[1:-1]:
    x_next,y_next,z_next = line.strip().split(",")
    X[i,j] = x_next
    Y[i,j] = y_next
    Z[i,j] = z_next
    j += 1
    if j==size:
        i+=1
        j = 0

X = X.T
Y = Y.T
Z = Z.T

#X, Y = np.meshgrid(np.linspace(1., 11.5, 197), np.linspace(.5, 4., 197))

plot_3D_loss_func(X, Y, Z)'''