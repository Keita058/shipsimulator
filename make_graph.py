from collections import deque
import os
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import pandas as pd
import scienceplots
plt.style.use(['science','ieee'])


def control_graph(csv_path):
    f_name = os.path.splitext(os.path.basename(csv_path))[0]
    df = pd.read_csv(csv_path)
    save_dir = dir_name + "/images/"+f_name+"/"
    os.makedirs(save_dir, exist_ok=True)
    x=df.time

    fig, ax = plt.subplots()
    y=df.n_p
    ax.plot(x,y)
    ax.set_xlabel(r"$time$[s]")
    ax.set_ylabel(r"$n_p$[npm]")
    fig.savefig(save_dir+"n_p.png")

    fig, ax = plt.subplots()
    y=df.rudder
    ax.plot(x,y)
    ax.set_xlabel(r"$time$[s]")
    ax.set_ylabel(r"$rudder angle$[deg]")
    fig.savefig(save_dir+"rudder_angle.png")


def velocity_graph(csv_path):
    f_name = os.path.splitext(os.path.basename(csv_path))[0]
    df = pd.read_csv(csv_path)
    save_dir = dir_name + "/images/"+f_name+"/"
    os.makedirs(save_dir, exist_ok=True)
    x=df.time

    fig, ax = plt.subplots()
    y=df.u
    ax.plot(x,y)
    ax.set_xlabel(r"$time$[s]")
    ax.set_ylabel(r"$u$[m/s]")
    fig.savefig(save_dir+"u.png")

    fig, ax = plt.subplots()
    y=df.v
    ax.plot(x,y)
    ax.set_xlabel(r"$time$[s]")
    ax.set_ylabel(r"$v$[m/s]")
    fig.savefig(save_dir+"v.png")

    fig, ax = plt.subplots()
    y=df.yaw
    ax.plot(x,y)
    ax.set_xlabel(r"$time$[s]")
    ax.set_ylabel(r"$r$[rad/s]")
    fig.savefig(save_dir+"r.png")

    fig, ax = plt.subplots()
    y1=df.u
    y2=df.v
    ax.plot(x,y1,label="$u$[m/s]")
    ax.plot(x,y2,label="$v$[m/s]")
    ax.set_xlabel(r"$time$[s]")
    ax.set_ylabel(r"$velocity$[m/s]")
    ax.legend(frameon = True)
    fig.savefig(save_dir+"velocity_uv.png")

def pose_graph(csv_path):
    f_name = os.path.splitext(os.path.basename(csv_path))[0]
    df = pd.read_csv(csv_path)
    save_dir = dir_name + "/images/"+f_name+"/"
    os.makedirs(save_dir, exist_ok=True)
    x=df.time

    fig, ax = plt.subplots()
    y=df.x
    ax.plot(x,y)
    ax.set_xlabel(r"$time$[s]")
    ax.set_ylabel(r"$x$[m]")
    fig.savefig(save_dir+"x.png")

    fig, ax = plt.subplots()
    y=df.y
    ax.plot(x,y)
    ax.set_xlabel(r"$time$[s]")
    ax.set_ylabel(r"$y$[m]")
    fig.savefig(save_dir+"y.png")

    fig, ax = plt.subplots()
    y=df.yaw
    ax.plot(x,y)
    ax.set_xlabel(r"$time$[s]")
    ax.set_ylabel(r"$\psi$[deg]")
    fig.savefig(save_dir+"psi.png")

    fig, ax = plt.subplots()
    x=df.y
    y=df.x
    ax.plot(x,y)
    ax.set_xlabel(r"$y$[m]")
    ax.set_ylabel(r"$x$[m]")
    fig.savefig(save_dir+"trajectory.png")

def create_graph(csv_path):
    df = pd.read_csv(csv_path)
    if len(df.columns) == 3:
        control_graph(csv_path)
    else:
        if "x" in df.columns:
            pose_graph(csv_path)
        else:
            velocity_graph(csv_path)

#グラフを作るデータがあるCSVファイルのあるディレクトリ名を指定
dir_name="subset"

files = os.listdir(dir_name)

for file in files:
    if file.endswith('.csv'):
        #print(file+"はCSVです")
        f_path = os.path.join(dir_name,file)
        print(f_path)
        create_graph(f_path)
    else:
        #print(file+"はCSVではありません")
        continue