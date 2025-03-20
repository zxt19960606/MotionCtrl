# =========================
# -*- coding: utf-8 -*-
# @Time: 2025/3/18 20:54
# @Author：zhangxuetao
# @File：main_exe.py
# =========================
import copy

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional, Protocol
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
from mplcursors import cursor
from abc import ABC, abstractmethod
import math
import collections


# ======================
# 类型定义和配置结构
# ======================
@dataclass(frozen=True)
class MotionConfig:
    """
    轴的运动配置
    """
    Vm: float # mm/s
    Am: float # mm/s²
    Dm: float  # mm/s²
    Jm: float = 0.0 # mm/s³
    ADCAI_T1: float = 0.2 # s
    ADCAI_T2: float = 0.0  # s
# class MotionConfig

@dataclass(frozen=True)
class AxisConfig:
    """
    轴配置
    """
    x: MotionConfig
    y: MotionConfig
# class AxisConfig

@dataclass
class MotionProfile:
    """
    运动规划
    """
    t: np.ndarray # 时间序列 (s)
    Pt: np.ndarray # 各轴位置 (mm)
    Vt: np.ndarray # 各轴速度 (mm/s)
    At: np.ndarray # 各轴加速度 (mm/s²)
    T_acc: np.ndarray # 各轴最终后加减速时间
# class MotionProfile

class AccelerationProfile(ABC):
    """
    加减速曲线抽象基类
    """
    @abstractmethod
    def generate(self, Ps: float, Pe: float, Ts: float) -> MotionProfile:
        pass
    # def generate
# class AccelerationProfile

class MovingAverageFilter:
    """滑动平均滤波器"""

    def __init__(self, window_size):
        if window_size <= 0:
            window_size = 1
        # if
        self.window = collections.deque(maxlen = window_size)
    # def __init__
    def fill_window(self, val):
        while len(self.window) < self.window.maxlen:
            self.window.append(val)
        # while
    # def

    def update(self, value):
        if len(self.window) == 0:
            self.fill_window(0)
        # if

        self.window.popleft()
        self.window.append(value)
        res = sum(self.window) / len(self.window)
        return res
    # def update
# class MovingAverageFilter


def trapezoidal_integration(v, T, x0 = 0):
    """梯形积分法"""
    x = np.zeros(len(v))
    x[0] = x0
    for i in range(1, len(v)):
        avg_v = (v[i] + v[i - 1]) / 2
        x[i] = x[i - 1] + avg_v * T
    return x
# def trapezoidal_integration

def trapezoidal_diff(v, T, a0 = 0):
    """梯形差分法"""
    a = np.zeros(len(v))
    a[0] = a0
    for i in range(1, len(v)):
        a[i] = (v[i] - v[i - 1]) / T
    return a
# def trapezoidal_integration

class ADCAI_T_Profile(AccelerationProfile):
    """
    梯形后加减速算法
    """
    def __init__(self, config: AxisConfig):
        self.cfg = config
    # def __init__

    def generate(self, Ps: np.ndarray, Pe: np.ndarray, Ts: float) -> MotionProfile:
        """
        后加减速，T型规划
        :param Ps: 起点坐标
        :param Pe: 终点坐标
        :param Ts: 插补周期
        :return: 返回运动曲线
        """
        delta = Pe - Ps
        L = np.linalg.norm(delta)
        T = delta / L
        Vm = np.array([self.cfg.x.Vm, self.cfg.y.Vm])
        Am = np.array([self.cfg.x.Am, self.cfg.y.Am])
        T1 = np.array([self.cfg.x.ADCAI_T1, self.cfg.y.ADCAI_T1])
        Am_act = copy.deepcopy(Am)
        V = max(Vm) * 2
        for i in range(len(T)):
            if T[i] != 0:
                V = min(V, np.abs(Vm[i] / T[i])) #路径最大速度
                Am_act[i] = min(Am_act[i], np.abs(Vm[i] / T1[i])) #实际最大轴加速度
            # if
        # for
        V = min(L/Ts, V)
        Vm_act = V * T # 实际单轴最大速度
        T_total = L / V # 运行时间
        T1_act = np.abs(Vm_act / Am_act)  # 实际后加减速时间
        tt = 0
        if T_total < max(T1):
            for i in range(len(T)):
                if Am_act[i] != 0:
                    tt = max(tt, L / Am_act[i])  # 路径最大速度
                # if
            # for
            T1_act = np.array([np.sqrt(tt), np.sqrt(tt)])
            T_total = max(T1_act)
            Vm_act = Am_act * T1_act
            for i in range(len(T)):
                if T[i] == 0:
                    Am_act[i] = 0
                    Vm_act[i] = 0
                # if
            # for
        # if

        # 后加减速处理
        a0 = Vm_act[0] * np.ones(int(round(T_total / Ts)))
        a1 = Vm_act[1] * np.ones(int(round(T_total / Ts)))
        a2 = np.zeros(int(round(max(T1_act) / Ts)))
        a0_2 = np.concatenate([a0, a2])
        a1_2 = np.concatenate([a1, a2])
        V_old = np.column_stack([a0_2, a1_2])
        idx = int((round(T_total / Ts))) + int((round(max(T1_act) / Ts)))
        t_x = np.arange(1, idx+1, 1)
        # 滤波计算
        ma_filter_x = MovingAverageFilter(int(round(T1_act[0] / Ts)))
        ma_filter_y = MovingAverageFilter(int(round(T1_act[1] / Ts)))
        Vt_x = [ma_filter_x.update(v) for v in V_old[:,0]]
        Vt_y = [ma_filter_y.update(v) for v in V_old[:,1]]
        Pt_x = trapezoidal_integration(Vt_x, Ts, Ps[0])
        Pt_y = trapezoidal_integration(Vt_y, Ts, Ps[1])
        At_x = trapezoidal_diff(Vt_x, Ts)
        At_y = trapezoidal_diff(Vt_y, Ts)

        profile = MotionProfile(
            t=t_x,
            At=np.column_stack([At_x, At_y]),
            Vt=np.column_stack([Vt_x, Vt_y]),
            Pt=np.column_stack([Pt_x, Pt_y]),
            T_acc=T1_act,
        )
        return profile
    # def generate
# class ADCAI_T_Profile

class MotionPlanner:
    """
    运动规划引擎
    """
    def __init__(self, profile_type: type = ADCAI_T_Profile):
        self.profile_type = profile_type
    # def __init__

    def plan_trajectory(self, Ps: np.ndarray, Pe: np.ndarray, cfg: AxisConfig, Ts: float = 0.001 ) -> MotionProfile:
        """
        执行完整的运动规划流程
        """

        planner = self.profile_type(cfg)
        profile = planner.generate(Ps, Pe, Ts)
        return profile
    # def plan_trajectory
# class MotionPlanner

class MotionVisualizer:
    """
    运动可视化类
    """
    def __init__(self, profile: MotionProfile):
        self.profile = profile
    # def __init__

    def plot_kinematics(self, axis: int = 0):
        """绘制运动学曲线"""
        fig, axs = plt.subplots(3, 1, figsize=(10, 8))
        self._plot_axis(axs[0], 'Position (mm)', self.profile.Pt[:,axis])
        self._plot_axis(axs[1], 'Velocity (mm/s)', self.profile.Vt[:,axis])
        self._plot_axis(axs[2], 'Acceleration (mm/s²)', self.profile.At[:,axis])
        plt.tight_layout()
    # def plot_kinematics

    def _plot_axis(self, ax, ylabel, data):
        ax.plot(self.profile.t, data, marker='.', markersize=2)
        ax.set_ylabel(ylabel)
        ax.grid(True)
        cursor(ax, hover=True)
    # def _plot_axis

    def plot_trajectory_3d(self):
        """三维轨迹可视化"""
        if self.profile.Pt.shape[1] < 3:
            raise ValueError("Not enough axes for 3D visualization")
        # if

        ax = plt.figure().add_subplot(111, projection='3d')
        ax.plot(self.profile.Pt[:,0],
        self.profile.Pt[:,1],
        self.profile.Pt[:,2])
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        plt.title('3D Motion Trajectory')
    # def plot_trajectory_3d
# class MotionVisualizer

def concatenate_with_overlap(arr1, arr2, overlap):
    # 确保基础参数有效
    if overlap < 0:
        raise ValueError("重叠长度不能为负数")
    if arr1.shape[1:] != arr2.shape[1:]:
        raise ValueError("数组维度不匹配")
    # if

    # 处理arr1
    if arr1.shape[0] >= overlap:
        part1_non_overlap = arr1[:-overlap]
        part1_overlap = arr1[-overlap:]
    else:
        # 行数不足时，后面补零
        part1_non_overlap = np.zeros((0,) + arr1.shape[1:], dtype = arr1.dtype)
        padding_shape = (overlap - arr1.shape[0],) + arr1.shape[1:]
        part1_overlap = np.vstack([arr1, np.zeros(padding_shape, dtype = arr1.dtype)])
    # if

    # 处理arr2
    if arr2.shape[0] >= overlap:
        part2_overlap = arr2[:overlap]
        part2_remain = arr2[overlap:]
    else:
        # 行数不足时，后面补零
        padding_shape = (overlap - arr2.shape[0],) + arr2.shape[1:]
        part2_overlap = np.hstack([arr2, np.zeros(padding_shape, dtype = arr2.dtype)])
        part2_remain = np.zeros((0,) + arr2.shape[1:], dtype = arr2.dtype)
    # if

    # 重叠部分相加
    overlapped_sum = part1_overlap + part2_overlap

    # 拼接结果
    result = np.concatenate((part1_non_overlap, overlapped_sum, part2_remain))
    return result
# def concatenate_with_overlap

def concatenate_2d_with_overlap_cols(arr1, arr2, overlap):
    if overlap < 0:
        raise ValueError("重叠长度不能为负数")
    if arr1.shape[1] != arr2.shape[1]:
        raise ValueError("数组列数必须一致")

        # 处理arr1的重叠部分
    if arr1.shape[0] >= overlap:
        part1_non_overlap = arr1[:-overlap, :]
        part1_overlap = arr1[-overlap:, :]
    else:
        # 行数不足，补零在arr1后面
        part1_non_overlap = np.zeros((0, arr1.shape[1]))  # 无剩余非重叠部分
        padding = np.zeros((overlap - arr1.shape[0], arr1.shape[1]))
        part1_overlap = np.vstack([arr1, padding])

        # 处理arr2的重叠部分
    if arr2.shape[0] >= overlap:
        part2_overlap = arr2[:overlap, :]
        part2_remain = arr2[overlap:, :]
    else:
        # 行数不足，补零在arr2后面
        part2_remain = np.zeros((0, arr2.shape[1]))  # 无剩余部分
        padding = np.zeros((overlap - arr2.shape[0], arr2.shape[1]))
        part2_overlap = np.vstack([arr2, padding])

        # 合并重叠部分
    overlapped_sum = part1_overlap + part2_overlap
    result = np.vstack([part1_non_overlap, overlapped_sum, part2_remain])
    return result
# def concatenate_2d_with_overlap_cols


def path_overlap(prof1: MotionProfile, prof2: MotionProfile, overlap) -> MotionProfile:
    t = concatenate_with_overlap(prof1.t, prof2.t, overlap)
    a = concatenate_2d_with_overlap_cols(prof1.At, prof2.At, overlap)
    v = concatenate_2d_with_overlap_cols(prof1.Vt, prof2.Vt, overlap)
    # p = concatenate_2d_with_overlap_cols(prof1.Pt, prof2.Pt, overlap)
    p_x = trapezoidal_integration(v[:, 0], 0.001, 0)
    p_y = trapezoidal_integration(v[:, 1], 0.001, 0)
    p = np.column_stack([p_x, p_y])

    return MotionProfile(t=t, At=a, Vt=v, Pt=p, T_acc=prof1.T_acc)
# def vel_overlap




# ======================
# 示例使用
# ======================
if __name__ == "__main__":
    # 配置参数
    config = AxisConfig(x=MotionConfig(Vm=10000/60.0, Am=10000.0, Dm=10000.0, Jm=0.0, ADCAI_T1=0.2, ADCAI_T2=0.0),
                        y=MotionConfig(Vm=6000/60.0, Am=5000.0, Dm=5000.0, Jm=0.0, ADCAI_T1=0.2, ADCAI_T2=0.0))

    # 初始化运动规划器
    planner = MotionPlanner(profile_type=ADCAI_T_Profile)

    # 执行运动规划
    profile_1 = planner.plan_trajectory(Ps=np.array([0.0, 0.0]), Pe=np.array([100.0, 0.0]), cfg=config, Ts=0.001)
    profile_2 = planner.plan_trajectory(Ps=np.array([100.0, 0.0]), Pe=np.array([100.0, 40.0]), cfg=config, Ts=0.001)
    profile_3 = planner.plan_trajectory(Ps=np.array([100.0, 40.0]), Pe=np.array([101.0, 40.0]), cfg=config, Ts=0.001)
    profile_4 = planner.plan_trajectory(Ps=np.array([101.0, 40.0]), Pe=np.array([101.0, 41.0]), cfg=config, Ts = 0.001)
    profile_5 = planner.plan_trajectory(Ps=np.array([101.0, 41.0]), Pe = np.array([102.0, 41.0]), cfg=config, Ts=0.001)
    profile_6 = planner.plan_trajectory(Ps = np.array([102.0, 41.0]), Pe = np.array([102.0, 42.0]), cfg = config, Ts = 0.001)
    profile_7 = planner.plan_trajectory(Ps = np.array([102.0, 42.0]), Pe = np.array([103.0, 42.0]), cfg = config, Ts = 0.001)
    profile_8 = planner.plan_trajectory(Ps = np.array([103.0, 42.0]), Pe = np.array([103.0, 43.0]), cfg = config, Ts = 0.001)
    profile_9 = planner.plan_trajectory(Ps = np.array([103.0, 43.0]), Pe = np.array([102.0, 43.0]), cfg = config, Ts = 0.001)
    profile_10 = planner.plan_trajectory(Ps = np.array([102.0, 43.0]), Pe = np.array([0.0, 0.0]), cfg = config, Ts = 0.001)

    # 轨迹重叠
    # overlap = int(1 * config.x.ADCAI_T1 * 1000) # 叠加时间
    overlap = 200
    profile_overlap_1 = path_overlap(profile_1, profile_2, overlap)
    overlap = 200
    profile_overlap_2 = path_overlap(profile_overlap_1, profile_3, overlap)
    overlap = 130
    profile_overlap_3 = path_overlap(profile_overlap_2, profile_4, overlap)
    overlap = 140
    profile_overlap_4 = path_overlap(profile_overlap_3, profile_5, overlap)
    overlap = 40
    profile_overlap_5 = path_overlap(profile_overlap_4, profile_6, overlap)
    overlap = 40
    profile_overlap_6 = path_overlap(profile_overlap_5, profile_7, overlap)
    overlap = 100
    profile_overlap_7 = path_overlap(profile_overlap_6, profile_8, overlap)
    overlap = 140
    profile_overlap_8 = path_overlap(profile_overlap_7, profile_9, overlap)
    overlap = 1
    profile_overlap_9 = path_overlap(profile_overlap_8, profile_10, overlap)

    ##################################  拼接数据 ##############################################
    At = np.vstack((profile_1.At, profile_2.At, profile_3.At, profile_4.At, profile_5.At,
                    profile_6.At, profile_7.At, profile_8.At, profile_9.At, profile_10.At))
    Vt = np.vstack((profile_1.Vt, profile_2.Vt, profile_3.Vt, profile_4.Vt, profile_5.Vt,
                    profile_6.Vt, profile_7.Vt, profile_8.Vt, profile_9.Vt, profile_10.Vt))
    Pt = np.vstack((profile_1.Pt, profile_2.Pt, profile_3.Pt, profile_4.Pt, profile_5.Pt,
                    profile_6.Pt, profile_7.Pt, profile_8.Pt, profile_9.Pt, profile_10.Pt))
    At_lap = profile_overlap_8.At
    Vt_lap = profile_overlap_8.Vt
    Pt_lap = profile_overlap_8.Pt
    #######
    path_vel = (Vt[:, 0] ** 2 + Vt[:, 1] ** 2) ** 0.5
    path_vel_lap = (Vt_lap[:, 0] ** 2 + Vt_lap[:, 1] ** 2) ** 0.5
    #######################################################################################

    fig, axs = plt.subplots(3, 1, figsize = (10, 8))
    axs[0].plot(At[:,0], label = 'acc x', marker = '.', markersize = 4, linestyle = '-')
    axs[0].plot(At[:,1], label = 'acc y', marker = '.', markersize = 4, linestyle = '-')
    plt.grid()
    plt.title('axis Acc')
    plt.ylabel('a(mm/s^2)')
    axs[0].legend()
    # 使用mplcursors 添加鼠标事件监听功能
    cursor(axs[0], hover = True)

    axs[1].plot(Vt[:, 0] * 60, label = 'vel x', marker = '.', markersize = 4, linestyle = '-')
    axs[1].plot(Vt[:, 1] * 60, label = 'vel y', marker = '.', markersize = 4, linestyle = '-')
    # axs[0].plot(path_vel * 60, label = 'vel path', marker = '.', markersize = 4, linestyle = '-')
    plt.grid()
    plt.title('axis Velo')
    plt.ylabel('v(mm/min)')
    axs[1].legend()
    # 使用mplcursors 添加鼠标事件监听功能
    cursor(axs[1], hover = True)

    axs[2].plot(Vt_lap[:, 0] * 60, label = 'lap vel x', marker = '.', markersize = 4, linestyle = '-')
    axs[2].plot(Vt_lap[:, 1] * 60, label = 'lap vel y', marker = '.', markersize = 4, linestyle = '-')
    axs[2].plot(path_vel_lap * 60, label = 'lap vel path', marker = '.', markersize = 4, linestyle = '-')
    plt.grid()
    plt.title('lap axis Velo')
    plt.ylabel('v(mm/min)')
    axs[2].legend()
    # 使用mplcursors 添加鼠标事件监听功能
    cursor(axs[2], hover = True)

    fig, axs = plt.subplots(2, 1, figsize = (10, 8))
    axs[0].plot(At[:, 0], label = 'acc x', marker = '.', markersize = 4, linestyle = '-')
    axs[0].plot(At[:, 1], label = 'acc y', marker = '.', markersize = 4, linestyle = '-')
    plt.grid()
    plt.title('axis Acc')
    plt.ylabel('a(mm/s^2)')
    axs[0].legend()
    # 使用mplcursors 添加鼠标事件监听功能
    cursor(axs[0], hover = True)
    axs[1].plot(At_lap[:, 0], label = 'lap acc x', marker = '.', markersize = 4, linestyle = '-')
    axs[1].plot(At_lap[:, 1], label = 'lap acc y', marker = '.', markersize = 4, linestyle = '-')
    plt.grid()
    plt.title('lap axis Acc')
    plt.ylabel('a(mm/s^2)')
    axs[1].legend()
    # 使用mplcursors 添加鼠标事件监听功能
    cursor(axs[1], hover = True)


    # 程序搭接后的插补点
    fig, ax = plt.subplots()
    ax.plot([0, 100, 100, 101, 101, 102, 102, 103, 103, 102, 0],
                  [0, 0, 40, 40, 41, 41, 42, 42, 43, 43, 0], label = 'NC contour', marker = '.', markersize = 4,
            linestyle = '-')
    ax.plot(Pt_lap[:, 0], Pt_lap[:, 1], label = 'Lap contour', marker = '.', markersize = 4, linestyle = '-')
    plt.grid()
    plt.axis('equal')
    plt.title('XY Contour')
    plt.xlabel('x(mm)')
    plt.ylabel('y(mm)')
    plt.legend()
    # 使用mplcursors 添加鼠标事件监听功能
    cursor(ax, hover = True)

    # 结果可视化
    # visualizer = MotionVisualizer(profile)
    # visualizer.plot_kinematics()
    plt.show()

# if main()
